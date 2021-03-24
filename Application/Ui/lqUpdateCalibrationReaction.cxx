#include "lqUpdateCalibrationReaction.h"

#include <pqActiveObjects.h>
#include <pqApplicationCore.h>
#include <pqDeleteReaction.h>
#include <pqObjectBuilder.h>
#include <pqServerManagerModel.h>

#include <vtkNew.h>
#include <vtkSMDomain.h>
#include <vtkSMParaViewPipelineControllerWithRendering.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMPropertyIterator.h>
#include <vtkSMProxy.h>
#include <vtkSMProxyListDomain.h>
#include <vtkSMProxyProperty.h>
#include <vtkSMSessionProxyManager.h>

#include "lqHelper.h"
#include "lqSensorListWidget.h"
#include "pqLidarViewManager.h"
#include "vvCalibrationDialog.h"

#include <cctype>
//-----------------------------------------------------------------------------
lqUpdateCalibrationReaction::lqUpdateCalibrationReaction(QAction *action) :
  Superclass(action)
{
  this->parentAction()->setEnabled(false);
  auto* core = pqApplicationCore::instance();

  pqServerManagerModel* smmodel = core->getServerManagerModel();
  this->connect(smmodel, SIGNAL(sourceAdded(pqPipelineSource*)), SLOT(onSourceAdded(pqPipelineSource*)));
  this->connect(smmodel, SIGNAL(sourceRemoved(pqPipelineSource*)), SLOT(onSourceRemoved(pqPipelineSource*)));

  foreach (pqPipelineSource* src, smmodel->findItems<pqPipelineSource*>())
    this->onSourceAdded(src);
}

//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::setTransform(vtkSMProxy * proxy,
                                               double x, double y, double z,
                                               double roll, double pitch, double yaw)
{
  vtkSMSessionProxyManager* pxm = pqActiveObjects::instance().proxyManager();

  vtkSMProperty * interpreterProp = proxy->GetProperty("PacketInterpreter");
  vtkSMProxy * interpreterProxy = vtkSMPropertyHelper(interpreterProp).GetAsProxy();

  // Create a transform proxy
  // For Transform2 : name "Position" = label "Translate
  // name "Rotation" = label "Rotate"
  // See ParaviewSrc/ParaViewCore/ServerManager/SMApplication/Resources/Utilities.xml
  vtkSmartPointer<vtkSMProxy> transformProxy = vtkSmartPointer<vtkSMProxy>::Take(pxm->NewProxy("extended_sources", "Transform2"));
  std::vector<std::string> translate;
  translate.push_back(std::to_string(x));
  translate.push_back(std::to_string(y));
  translate.push_back(std::to_string(z));
  std::vector<std::string> rotate;
  rotate.push_back(std::to_string(roll));
  rotate.push_back(std::to_string(pitch));
  rotate.push_back(std::to_string(yaw));
  UpdateProperty(transformProxy, "Position", translate);
  UpdateProperty(transformProxy, "Rotation", rotate);

  vtkSMProperty * TransformProp = interpreterProxy->GetProperty("Sensor Transform");
  vtkSMPropertyHelper(TransformProp).Set(transformProxy);
}

//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::setNetworkCalibration(vtkSMProxy * proxy, double listenningPort,
     double forwardingPort, bool isForwarding, QString ipAddressForwarding, bool isCrashAnalysing)
{
  if(IsStreamProxy(proxy))
  {
    UpdateProperty(proxy, "ListeningPort", std::to_string(listenningPort));
    UpdateProperty(proxy, "IsCrashAnalysing", std::to_string(isCrashAnalysing));

    if(isForwarding)
    {
      UpdateProperty(proxy, "ForwardedPort", std::to_string(forwardingPort));
      UpdateProperty(proxy, "IsForwarding", std::to_string(isForwarding));
      UpdateProperty(proxy, "ForwardedIpAddress", ipAddressForwarding.toStdString());
    }
  }
}

//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::setCalibrationFileAndDefaultInterpreter(vtkSMProxy * proxy,
                                                                          QString calibrationFile)
{
  if(IsLidarProxy(proxy))
  {
    // Set the calibration file
    UpdateProperty(proxy, "CalibrationFileName", calibrationFile.toStdString());

    // The default interpreter is the first one in the chronological order.
    // We need it to be the Meta one or the Special Velarray if the calibration file says so.
    vtkSMProperty * interpreterProp = proxy->GetProperty("PacketInterpreter");

    vtkSMProxyProperty* proxyProperty =  vtkSMProxyProperty::SafeDownCast(interpreterProp);
    if (!proxyProperty)
    {
      return;
    }

    vtkSMProxyListDomain * proxyListDomain = vtkSMProxyListDomain::SafeDownCast(proxyProperty->FindDomain("vtkSMProxyListDomain"));
    if (!proxyListDomain)
    {
      return;
    }

    vtkSMProxy* defaultProxy = proxyListDomain->FindProxy("LidarPacketInterpreter", "VelodyneMetaPacketInterpreter");
    if ((calibrationFile.contains("velarray", Qt::CaseInsensitive)))
    {
      defaultProxy = proxyListDomain->FindProxy("LidarPacketInterpreter", "VelodyneSpecialVelarrayPacketInterpreter");
    }

    // Set the found proxy in the proxy list domain to the lidar property
    // This allows to update the "drop down" menu in the interpreter ui property
    vtkSMPropertyHelper(interpreterProp).Set(defaultProxy);
  }
}

//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::UpdateCalibration(pqPipelineSource* & lidarSource,
                                                    pqPipelineSource* & posOrSource,
                                                    const vvCalibrationDialog& dialog)
{
  vtkSMProxy* lidarProxy = lidarSource->getProxy();
  if(!lidarProxy)
  {
    std::cerr << "Lidar proxy is null, calibration is cancelled" << std::endl;
    return;
  }

  // Set the calibration File and the lidar interpreter
  lqUpdateCalibrationReaction::setCalibrationFileAndDefaultInterpreter(lidarProxy, dialog.selectedCalibrationFile());


  // Set the transform of the lidar Sensor
  lqUpdateCalibrationReaction::setTransform(lidarProxy, dialog.lidarX(), dialog.lidarY(), dialog.lidarZ(),
                                            dialog.lidarRoll(), dialog.lidarPitch(), dialog.lidarYaw());


  // Set the Network Part
  lqUpdateCalibrationReaction::setNetworkCalibration(lidarProxy, dialog.lidarPort(), dialog.lidarForwardingPort(),
                                                     dialog.isForwarding(), dialog.ipAddressForwarding(), dialog.isCrashAnalysing());


  lidarProxy->UpdateSelfAndAllInputs();
  lidarSource->updatePipeline();

  // The user can chose to enable the gps packet interpretation in the dialog.
  // This is why we create here the gps source if it's not already exist
  // And we remove it if the user disable it.
  if (dialog.isEnableInterpretGPSPackets())
  {
    vtkSMProxy* posOrProxy = nullptr;

    if(posOrSource)
    {
      posOrProxy = posOrSource->getProxy();
    }
    else
    {
      // If the Gps proxy is not created. We created now.
      pqServer* server = pqActiveObjects::instance().activeServer();
      pqObjectBuilder* builder = pqApplicationCore::instance()->getObjectBuilder();
      posOrSource = builder->createSource("sources", "PositionOrientationStream", server);
      posOrProxy = posOrSource->getProxy();
    }

    // Set the Network Part
    lqUpdateCalibrationReaction::setNetworkCalibration(posOrProxy, dialog.gpsPort(),dialog.gpsForwardingPort(),
                                                       dialog.isForwarding(), dialog.ipAddressForwarding(), dialog.isCrashAnalysing());
    // Set the transform of the gps Sensor
    lqUpdateCalibrationReaction::setTransform(posOrProxy, dialog.gpsX(), dialog.gpsY(), dialog.gpsZ(),
                                              dialog.gpsRoll(), dialog.gpsPitch(), dialog.gpsYaw());

    posOrSource->updatePipeline();
    posOrProxy->UpdateSelfAndAllInputs();

    // Set the Position Orientation Stream Associated to the lidar
    lqSensorListWidget * listSensor = lqSensorListWidget::instance();
    listSensor->setPosOrSourceToLidarSourceWidget(lidarSource, posOrSource);
  }

  // If the user does not want the GPS Packet and one was created, we remove it
  if (posOrSource && !dialog.isEnableInterpretGPSPackets())
  {
    QSet<pqPipelineSource*> sources;
    sources.insert(posOrSource);
    pqDeleteReaction::deleteSources(sources);
    posOrSource = nullptr;
  }
}

//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::UpdateExistingSource(pqPipelineSource* & lidarSource,
                                                       pqPipelineSource* & posOrSource)
{
  vtkSMProxy * posOrProxy = nullptr;
  if(posOrSource)
  {
    posOrProxy = posOrSource->getProxy();
  }

  // Create the dialog with the proxy so the dialog has the proxy information
  vvCalibrationDialog dialog(lidarSource->getProxy(), posOrProxy,
                             pqLidarViewManager::instance()->getMainWindow());
  DisplayDialogOnActiveWindow(dialog);

  // Launch the calibration Dialog
  if (!dialog.exec())
  {
    return;
  }

  UpdateCalibration(lidarSource, posOrSource, dialog);

  // Update UI
  pqView* view = pqActiveObjects::instance().activeView();
  vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;

  if(IsLidarStream(lidarSource))
  {
    pqActiveObjects::instance().setActiveSource(lidarSource);
    controller->Show(lidarSource->getSourceProxy(), 0, view->getViewProxy());
    lidarSource->getProxy()->InvokeCommand("Start");
  }

  if(IsPositionOrientationStream(posOrSource))
  {
    controller->Show(posOrSource->getSourceProxy(), 0, view->getViewProxy());
    posOrSource->getProxy()->InvokeCommand("Start");
  }
}

//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::onTriggered()
{
  // The Calibration reaction only handle a single lidar
  // In case of multiple lidars the first one found in the pipeline browser is updated
  std::vector<vtkSMProxy*> lidarProxys = GetLidarsProxy();
  if(lidarProxys.empty())
  {
    return;
  }
  vtkSMProxy * proxyToUpdate = lidarProxys[0];

  // If the proxy to update is a reader
  // We call the applogic function for now
  if(IsLidarReaderProxy(proxyToUpdate))
  {
    pqLidarViewManager::instance()->runPython(QString("lv.onChooseCalibrationFile()\n"));
    return;
  }

  // Get the lidar source of the lidarProxy to update
  pqPipelineSource* lidarSource = GetPipelineSourceFromProxy(proxyToUpdate);
  if(!lidarSource)
  {
    return;
  }

  // Get the Position Orientation Stream Associated to the lidar
  lqSensorListWidget * listSensor = lqSensorListWidget::instance();
  pqPipelineSource* posOrSource = listSensor->getPosOrSourceAssociatedToLidarSource(lidarSource);

  lqUpdateCalibrationReaction::UpdateExistingSource(lidarSource, posOrSource);
}



//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::onSourceAdded(pqPipelineSource *src)
{
  if (!this->parentAction()->isEnabled() && IsLidarProxy(src->getProxy()))
  {
    this->parentAction()->setEnabled(true);
  }
}

//-----------------------------------------------------------------------------
void lqUpdateCalibrationReaction::onSourceRemoved(pqPipelineSource * vtkNotUsed(src)){
  if (this->parentAction()->isEnabled() && !HasLidarProxy())
  {
    this->parentAction()->setEnabled(false);
    this->parentAction()->setChecked(false);
  }
}
