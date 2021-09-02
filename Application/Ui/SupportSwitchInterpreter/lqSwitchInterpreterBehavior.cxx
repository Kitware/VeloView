#include "lqSwitchInterpreterBehavior.h"

#include "lqHelper.h"
#include "pqLidarViewManager.h"
#include "vvCalibrationDialog.h"

#include <pqApplicationCore.h>
#include <pqServerManagerModel.h>

#include <vtkEventQtSlotConnect.h>
#include <vtkPVProgressHandler.h>
#include <vtkPVSession.h>
#include <vtkProcessModule.h>
#include <vtkSMProperty.h>
#include <vtkSMPropertyHelper.h>

#include "vtkVelodyneMetaPacketInterpreter.h"

//-----------------------------------------------------------------------------
lqSwitchInterpreterBehavior::lqSwitchInterpreterBehavior(QObject *parent) :
  Superclass(parent)
{
  //Monitor added sources
  pqServerManagerModel* smmodel = pqApplicationCore::instance()->getServerManagerModel();
  this->connect(smmodel, SIGNAL(sourceAdded(pqPipelineSource*)),
                SLOT(sourceAdded(pqPipelineSource*)));

  this->connect(this, SIGNAL(interpChange(vtkSMProxy*)),
                SLOT(uiSelectNewCalibrationFile(vtkSMProxy*)));
}

//-----------------------------------------------------------------------------
void lqSwitchInterpreterBehavior::sourceAdded(pqPipelineSource* src)
{
  if(IsLidarProxy(src->getProxy()))
  {
    this->ConnectionInterpreter = vtkSmartPointer<vtkEventQtSlotConnect>::New();

    this->ConnectionInterpreter->Connect(src->getProxy()->GetProperty("PacketInterpreter"),
                                         vtkCommand::ModifiedEvent, this,
                                         SLOT(onInterpreterUpdated(
                                                vtkObject*, unsigned long, void*))) ;
  }
}

//-----------------------------------------------------------------------------
void lqSwitchInterpreterBehavior::onInterpreterUpdated(vtkObject* caller,
                                                       unsigned long, void*)
{
  vtkSMProperty* interpreterProp = vtkSMProperty::SafeDownCast(caller);
  if(!interpreterProp)
  {
    return;
  }

  vtkSMProxy * interpreterProxy = vtkSMPropertyHelper(interpreterProp).GetAsProxy();
  if(!interpreterProxy)
  {
    return;
  }

  vtkSmartPointer<vtkVelodyneMetaPacketInterpreter> interp =
      vtkVelodyneMetaPacketInterpreter::SafeDownCast(interpreterProxy->GetClientSideObject());
  if(interp)
  {
    interp->AddObserver(FirstSwitchOfInterpreterEvent, this,
                        &lqSwitchInterpreterBehavior::onChangeFormatDetected);
  }
}

//-----------------------------------------------------------------------------
void lqSwitchInterpreterBehavior::uiSelectNewCalibrationFile(vtkSMProxy * lidarProxy)
{
  if(!lidarProxy)
  {
    std::cout << "Switch of interpreter was detected but calibration file not updated " << std::endl;
    return;
  }

  // In the OpenPcapReaction, a handler is created
  // This allows updating the progress bar so the user can see that LV is openning a pcap
  // Here if we let this handler waiting for event this will freeze the ui
  // And the user will not be able to select the new calibration file
  // Ths is why we clean the pending progress
  // This also means that the progress bar will not been updated
  // during the second part of the open pcap (after the switch)
  // For now we don't add it because it will complexe the code
  // for a feature that should not be used often
  vtkProcessModule* pm = vtkProcessModule::GetProcessModule();
  vtkPVSession* session = vtkPVSession::SafeDownCast(pm->GetSession());
  vtkSmartPointer<vtkPVProgressHandler> handler = session->GetProgressHandler();
  handler->LocalCleanupPendingProgress();

  // We launch the calibration dialog
  vvCalibrationDialog dialog(pqLidarViewManager::instance()->getMainWindow(), false);
  if(!dialog.exec())
  {
    return;
  }
  vtkSMPropertyHelper(lidarProxy, "CalibrationFileName").Set(dialog.selectedCalibrationFile().toStdString().c_str());
  lidarProxy->UpdateProperty("CalibrationFileName");

  // We need to call the start function of the stream to load the calibration file
  if(IsLidarStreamProxy(lidarProxy))
  {
    lidarProxy->InvokeCommand("Start");
  }

}

//-----------------------------------------------------------------------------
void lqSwitchInterpreterBehavior::onChangeFormatDetected(vtkObject* object,
                                                         unsigned long, void*)
{
  vtkSmartPointer<vtkVelodyneMetaPacketInterpreter> metaInterpreter =
      vtkVelodyneMetaPacketInterpreter::SafeDownCast(object);

  if(!metaInterpreter)
  {
    return;
  }

  // We can not set the calibration fileName to the interpreter directly
  // using (interp->SetCalibrationFileName) because
  // vtkLidarReader::RequestInformation will rewrite the calibration file
  // if it's not the same as its own.
  // So we need to search for the lidarReader that have the right interpreter
  // and set its calibration file to the new one.
  // We can not add the Observer to the lidarReader (or lidarStream) directly
  // because the event is sent by the interpreter
  vtkSMProxy * lidarProxy =
      GetLidarProxyOfInterpreter<vtkLidarStream,vtkVelodyneMetaPacketInterpreter>(metaInterpreter);

  if(!lidarProxy)
  {
    lidarProxy = GetLidarProxyOfInterpreter<vtkLidarReader,vtkVelodyneMetaPacketInterpreter>(metaInterpreter);
  }

  // There is 3 threads in LV when we are in stream mode :
  // - 1 receiver thread : that receive the packet on the network and put them in the queue
  // - 1 Consumer/interpreter thread : that get packet from the queue,
  //                                   interpret them and convert them into vtkpolyData
  // - 1 UI thread : that ask for UpdatePipeline to update the UI Regularly
  //
  // This callback is the direct consequence of the event send by an interpreter
  // So this function is execute in the consumer/interpreter thread
  // We can not display a dialog in that thread (cause segFault)
  // So we need to go back to the UI thread to display the dialog
  // This is why we need to emit a signal back to the ui thread
  emit interpChange(lidarProxy);
}
