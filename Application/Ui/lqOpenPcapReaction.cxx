#include "lqOpenPcapReaction.h"

#include <vtkNew.h>
#include <vtkSMSourceProxy.h>
#include <vtkSMParaViewPipelineControllerWithRendering.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMProxy.h>

#include <pqActiveObjects.h>
#include <pqPVApplicationCore.h>
#include <pqObjectBuilder.h>
#include <pqPipelineSource.h>
#include <pqSettings.h>
#include <pqView.h>

#include "lqHelper.h"
#include "lqUpdateCalibrationReaction.h"
#include "pqLidarViewManager.h"
#include "vvCalibrationDialog.h"
#include "lqSensorListWidget.h"

#include <QApplication>
#include <QFileDialog>
#include <QProgressDialog>
#include <QString>
#include <string>

#include <vtkCommand.h>

#include <vtkProcessModule.h>
#include <vtkPVSession.h>
#include <vtkPVProgressHandler.h>

//----------------------------------------------------------------------------
class lqOpenPcapReaction::vtkObserver : public vtkCommand
{
public:
  static vtkObserver* New()
  {
    vtkObserver* obs = new vtkObserver();
    return obs;
  }

  void Execute(vtkObject* , unsigned long eventId, void*) override
  {

      if (eventId == vtkCommand::ProgressEvent)
      {
        QApplication::instance()->processEvents();
      }
  }
};

//-----------------------------------------------------------------------------
lqOpenPcapReaction::lqOpenPcapReaction(QAction *action) :
  Superclass(action)
{
}

//-----------------------------------------------------------------------------
void lqOpenPcapReaction::onTriggered()
{
  // Get the pcap filename
  pqSettings* settings = pqApplicationCore::instance()->settings();
  QString defaultDir = settings->value("LidarPlugin/OpenData/DefaultDir", QDir::homePath()).toString();
  QString filename = QFileDialog::getOpenFileName(nullptr,
                                 QString("Open LiDAR File"),
                                 defaultDir, QString("Wireshark Capture (*.pcap)"));

  if (!filename.isNull() && !filename.isEmpty())
  {
    QFileInfo fileInfo(filename);
    settings->setValue("LidarPlugin/OpenData/DefaultDir", fileInfo.absolutePath());

    lqOpenPcapReaction::createSourceFromFile(filename);
  }
}

//-----------------------------------------------------------------------------
void lqOpenPcapReaction::createSourceFromFile(QString fileName)
{
  pqServer* server = pqActiveObjects::instance().activeServer();
  pqObjectBuilder* builder = pqApplicationCore::instance()->getObjectBuilder();
  vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;
  pqView* view = pqActiveObjects::instance().activeView();

  // Launch the calibration Dialog before creating the Source to allow to cancel the action
  // (with the "cancel" button in the dialog)
  vvCalibrationDialog dialog(pqLidarViewManager::instance()->getMainWindow());
  //DisplayDialogOnActiveWindow(dialog);
  if (!dialog.exec())
  {
    return;
  }

  // Create a progress bar so the user see that VeloView is running
  QProgressDialog progress("Reading pcap", "", 0, 0, pqLidarViewManager::getMainWindow());
  progress.setCancelButton(nullptr);
  progress.setModal(true);
  progress.show();

  vtkProcessModule* pm = vtkProcessModule::GetProcessModule();
  vtkPVSession* session = vtkPVSession::SafeDownCast(pm->GetSession());
  if(!session)
  {
    return;
  }
  vtkSmartPointer<vtkPVProgressHandler> handler = session->GetProgressHandler();
  handler->PrepareProgress();
  double interval = handler->GetProgressInterval();
  handler->SetProgressInterval(0.05);
  vtkNew<vtkObserver> obs;
  unsigned long tag = handler->AddObserver(vtkCommand::ProgressEvent, obs);


  // Remove all Streams (and every filter depending on them) from pipeline Browser
  // Thanks to the lqSensorListWidget,
  // if a LidarStream is delete, it will automatically delete its PositionOrientationStream.
  // So we just have to delete all lidarStream.
  RemoveAllProxyTypeFromPipelineBrowser<vtkLidarStream *>();

  if(!dialog.isEnableMultiSensors())
  {
    // We remove all lidarReader (and every filter depending on them) in the pipeline
    // Thanks to the lqSensorListWidget,
    // if a LidarReader is delete, it will automatically delete its PositionOrientationReader.
    // So we just have to delete all lidarReader.
    RemoveAllProxyTypeFromPipelineBrowser<vtkLidarReader *>();
  }

  // Create the lidar Reader
  // We have to use pqObjectBuilder::createSource to add the created source to the pipeline
  // The source will be created immediately so the signal "sourceAdded" of the pqServerManagerModel
  // is send during "create source".
  // To get the pqPipelineSource modified with the new property, you have to connect to the signal
  // "dataUpdated" of the pqServerManagerModel
  pqPipelineSource* lidarSource = builder->createSource("sources", "LidarReader", server);
  vtkSMPropertyHelper(lidarSource->getProxy(), "FileName").Set(fileName.toStdString().c_str());
  lidarSource->getProxy()->UpdateProperty("FileName");
  QString lidarName = lidarSource->getSMName();

  pqPipelineSource * posOrSource = nullptr;
  QString posOrName = "";

  // Update lidarSource and posOrSource
  // If the GPs interpretation is asked, the posOrsource will be created in the lqUpdateCalibrationReaction
  // because it has to manage it if the user enable interpreting GPS packet after the first instantiation
  lqUpdateCalibrationReaction::UpdateCalibration(lidarSource, posOrSource, dialog);

  if (posOrSource)
  {
    posOrName = posOrSource->getSMName();
    controller->Show(posOrSource->getSourceProxy(), 0, view->getViewProxy());
  }

  //Update applogic to be able to use function only define in applogic.
  pqLidarViewManager::instance()->runPython(QString("lv.UpdateApplogicReader('%1', '%2')\n").arg(lidarName, posOrName));

  // Remove the handler so the user can interact with VeloView again (pushing any button)
  handler->RemoveObserver(tag);
  handler->LocalCleanupPendingProgress();
  handler->SetProgressInterval(interval);
  progress.close();
}
