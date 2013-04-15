#include "pqVelodyneManager.h"
#include "vvLoadDataReaction.h"
#include "vvCalibrationDialog.h"

#include <pqActiveObjects.h>
#include <pqActiveView.h>
#include <pqApplicationCore.h>
#include <pqDataRepresentation.h>
#include <pqPipelineSource.h>
#include <pqPythonDialog.h>
#include <pqPythonManager.h>
#include <pqPVApplicationCore.h>
#include <pqRenderView.h>
#include <pqServer.h>
#include <pqServerManagerModel.h>
#include <pqSettings.h>
#include <pqView.h>

#include <vtkSMPropertyHelper.h>
#include <vtkSMSourceProxy.h>
#include <vtkSMViewProxy.h>
#include <vtkTimerLog.h>

#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMainWindow>
#include <QTimer>


//-----------------------------------------------------------------------------
class pqVelodyneManager::pqInternal
{
public:

  pqInternal()
  {
    this->Playing = false;
    this->LogoLabel = new QLabel;
    this->LogoLabel->setPixmap(QPixmap(":/VelodyneHDLPlugin/velodyne_logo.png"));
    this->LogoLabel->setScaledContents(true);
    this->FilenameLabel = new QLabel;
    this->StatusLabel = new QLabel;
    this->TimeLabel = new QLabel;
  }

  bool Playing;

  QAction* OpenFile;
  QAction* Close;
  QAction* OpenSensor;
  QAction* ChooseCalibrationFile;
  QAction* ResetView;
  QAction* Play;
  QAction* SeekForward;
  QAction* SeekBackward;
  QAction* GotoStart;
  QAction* GotoEnd;
  QAction* Record;
  QAction* MeasurementGrid;
  QAction* SaveCSV;

  QLabel* FilenameLabel;
  QLabel* LogoLabel;
  QLabel* StatusLabel;
  QLabel* TimeLabel;
};

//-----------------------------------------------------------------------------
QPointer<pqVelodyneManager> pqVelodyneManagerInstance = NULL;

//-----------------------------------------------------------------------------
pqVelodyneManager *pqVelodyneManager::instance()
{
  if (!pqVelodyneManagerInstance)
    {
    pqVelodyneManagerInstance = new pqVelodyneManager(pqApplicationCore::instance());
    }

  return pqVelodyneManagerInstance;
}

//-----------------------------------------------------------------------------
pqVelodyneManager::pqVelodyneManager(QObject *p) : QObject(p)
{
  this->Internal = new pqInternal;
}

//-----------------------------------------------------------------------------
pqVelodyneManager::~pqVelodyneManager()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::pythonStartup()
{
  QString pythonDir = QString("/source/velodyne/velodyneviewer/VelodyneHDL/python");
  if (!QDir(pythonDir).exists())
    {
    pythonDir = QCoreApplication::applicationDirPath()  + "/../Python";
    }

  this->runPython(QString(
      "import sys\n"
      "sys.path.insert(0, '%1')\n"
      "import veloview.applogic as _vv\n"
      "_vv.start()\n").arg(pythonDir));

  this->onMeasurementGrid();

  bool showDialogAtStartup = false;
  if (showDialogAtStartup)
    {
    pqPythonManager* manager = pqPVApplicationCore::instance()->pythonManager();
    pqPythonDialog* dialog = manager->pythonShellDialog();
    dialog->show();
    dialog->raise();
    dialog->activateWindow();
    }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::runPython(const QString& statements)
{
  //printf("runPython(\"%s\")\n", qPrintable(statements));
  pqPythonManager* manager = pqPVApplicationCore::instance()->pythonManager();
  pqPythonDialog* dialog = manager->pythonShellDialog();
  dialog->runString(statements);
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::setup(QAction* openFile, QAction* close, QAction* openSensor,
  QAction* chooseCalibrationFile, QAction* resetView, QAction* play, QAction* seekForward, QAction* seekBackward,  QAction* gotoStart, QAction* gotoEnd,
  QAction* record, QAction* measurementGrid, QAction* saveScreenshot, QAction* saveCSV)
{
  this->Internal->OpenFile = openFile;
  this->Internal->Close = close;
  this->Internal->OpenSensor = openSensor;
  this->Internal->ChooseCalibrationFile = chooseCalibrationFile;
  this->Internal->ResetView = resetView;
  this->Internal->Play = play;
  this->Internal->SeekForward = seekForward;
  this->Internal->SeekBackward = seekBackward;
  this->Internal->GotoStart = gotoStart;
  this->Internal->GotoEnd = gotoEnd;
  this->Internal->Record = record;
  this->Internal->MeasurementGrid = measurementGrid;
  this->Internal->SaveCSV = saveCSV;

  play->setEnabled(false);
  record->setEnabled(false);
  seekForward->setEnabled(false);
  seekBackward->setEnabled(false);
  gotoStart->setEnabled(false);
  gotoEnd->setEnabled(false);

  pqSettings* settings = pqApplicationCore::instance()->settings();
  bool gridVisible = settings->value("VelodyneHDLPlugin/MeasurementGrid/Visibility", true).toBool();
  measurementGrid->setChecked(gridVisible);

  this->connect(close, SIGNAL(triggered()), SLOT(onClose()));
  this->connect(openSensor, SIGNAL(triggered()), SLOT(onOpenSensor()));
  this->connect(chooseCalibrationFile, SIGNAL(triggered()), SLOT(onChooseCalibrationFile()));
  this->connect(resetView, SIGNAL(triggered()), SLOT(onResetView()));
  this->connect(play, SIGNAL(triggered()), SLOT(onPlay()));

  this->connect(seekForward, SIGNAL(triggered()), SLOT(onSeekForward()));
  this->connect(seekBackward, SIGNAL(triggered()), SLOT(onSeekBackward()));
  this->connect(gotoStart, SIGNAL(triggered()), SLOT(onGotoStart()));
  this->connect(gotoEnd, SIGNAL(triggered()), SLOT(onGotoEnd()));

  this->connect(record, SIGNAL(triggered()), SLOT(onRecord()));
  this->connect(measurementGrid, SIGNAL(triggered()), SLOT(onMeasurementGrid()));
  this->connect(saveScreenshot, SIGNAL(triggered()), SLOT(onSaveScreenshot()));
  this->connect(saveCSV, SIGNAL(triggered()), SLOT(onSaveCSV()));

  new vvLoadDataReaction(openFile);

  QTimer::singleShot(0, this, SLOT(pythonStartup()));
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::openData(const QString& filename)
{
  if (QFileInfo(filename).suffix() == "pcap")
    {
      vvCalibrationDialog dialog;
      int accepted = dialog.exec();

      if (!accepted)
        {
        return;
        }

      QString calibrationFile = dialog.selectedCalibrationFile();

      this->onClose();
      this->runPython(QString("_vv.openPCAP('%1', '%2')\n").arg(filename).arg(calibrationFile));

      if (pqActiveObjects::instance().activeSource())
        {
        this->filenameLabel()->setText(QString("File: %1.").arg(QFileInfo(filename).fileName()));


        this->Internal->Play->setEnabled(true);
        this->Internal->SeekForward->setEnabled(true);
        this->Internal->SeekBackward->setEnabled(true);
        this->Internal->GotoStart->setEnabled(true);
        this->Internal->GotoEnd->setEnabled(true);
        }

      this->updateTimeLabel();
    }
  else
    {
    this->runPython(QString("_vv.openData('%1')\n").arg(filename));
    }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onClose()
{
  this->Internal->Record->setChecked(false);
  this->setStreaming(false);
  this->runPython(QString("_vv.close()\n"));
  this->filenameLabel()->setText(QString());
  this->timeLabel()->setText(QString());
  this->statusLabel()->setText(QString());

  this->Internal->Play->setEnabled(false);
  this->Internal->Record->setEnabled(false);
  this->Internal->SeekForward->setEnabled(false);
  this->Internal->SeekBackward->setEnabled(false);
  this->Internal->GotoStart->setEnabled(false);
  this->Internal->GotoEnd->setEnabled(false);
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onPlay()
{
  this->setStreaming(!this->Internal->Playing);
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onSeekForward()
{
  if (this->Internal->Playing)
    {
    this->runPython(QString("_vv.playDirectionForward()\n"));
    }
  else if (!this->Internal->Playing)
    {
    this->runPython(QString("_vv.gotoNext()\n"));
    }

  this->updateTimeLabel();
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onSeekBackward()
{
  if (this->Internal->Playing)
    {
    this->runPython(QString("_vv.playDirectionReverse()\n"));
    }
  else if (!this->Internal->Playing)
    {
    this->runPython(QString("_vv.gotoPrevious()\n"));
    }

  this->updateTimeLabel();
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onGotoStart()
{
  this->runPython(QString("_vv.gotoStart()\n"));
  this->updateTimeLabel();
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onGotoEnd()
{
  this->runPython(QString("_vv.gotoEnd()\n"));
  this->updateTimeLabel();
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onSaveScreenshot()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  QString defaultDir = settings->value("VelodyneHDLPlugin/OpenData/DefaultDir", QDir::homePath()).toString();


  QString selectedFiler("*.png");
  QString fileName = QFileDialog::getSaveFileName(this->getMainWindow(), tr("Screenshot"),
                          defaultDir,
                          tr("png (*.png)"), &selectedFiler);

  if (fileName.isEmpty())
    {
    return;
    }

  settings->setValue("VelodyneHDLPlugin/OpenData/DefaultDir", QFileInfo(fileName).absoluteDir().absolutePath());

  this->runPython(QString("_vv.saveScreenshot('%1')\n").arg(fileName));
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onSaveCSV()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  QString defaultDir = settings->value("VelodyneHDLPlugin/OpenData/DefaultDir", QDir::homePath()).toString();


  QString selectedFiler("*.csv");
  QString fileName = QFileDialog::getSaveFileName(this->getMainWindow(), tr("Save CSV"),
                          defaultDir,
                          tr("csv (*.csv)"), &selectedFiler);

  if (fileName.isEmpty())
    {
    return;
    }

  settings->setValue("VelodyneHDLPlugin/OpenData/DefaultDir", QFileInfo(fileName).absoluteDir().absolutePath());

  this->runPython(QString("_vv.saveCSVCurrentFrame('%1')\n").arg(fileName));
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::setStreaming(bool streaming)
{
  pqPipelineSource* source = pqActiveObjects::instance().activeSource();
  vtkSMSourceProxy* sourceProxy = source ? vtkSMSourceProxy::SafeDownCast(source->getProxy()) : NULL;
  if (!sourceProxy)
    {
    return;
    }

  this->Internal->Playing = streaming;

  if (streaming)
    {
    this->runPython(QString("_vv.startStream()\n"));
    this->runPython(QString("_vv.playDirectionForward()\n"));
    this->Internal->Play->setIcon(QPixmap(":/VelodyneHDLPlugin/media-playback-pause.png"));
    QTimer::singleShot(33, this, SLOT(onPollSource()));
    }
  else
    {
    this->runPython(QString("_vv.stopStream()\n"));
    this->Internal->Play->setIcon(QPixmap(":/VelodyneHDLPlugin/media-playback-start.png"));
    }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onPollSource()
{
  if (this->Internal->Playing)
    {
    double startTime = vtkTimerLog::GetUniversalTime();

    /*
    static double lastTime = startTime;
    static int frameCounter = 0;
    if (startTime - lastTime > 1.0)
      {
      printf("%f fps\n", frameCounter / (startTime - lastTime));
      frameCounter = 0;
      lastTime = startTime;
      }
    ++frameCounter;
    */

    this->runPython("_vv.onPlayTimer()\n");

    int elapsedMilliseconds = static_cast<int>((vtkTimerLog::GetUniversalTime() - startTime)*1000);
    int waitMilliseconds = 33 - elapsedMilliseconds;
    QTimer::singleShot(waitMilliseconds > 0 ? waitMilliseconds : 1, this, SLOT(onPollSource()));

    this->updateTimeLabel();
    }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::updateTimeLabel()
{
  pqView* view = this->getRenderView();
  vtkSMViewProxy* viewProxy = view ? vtkSMViewProxy::SafeDownCast(view->getProxy()) : NULL;
  if (viewProxy)
    {
      int viewTime = static_cast<int>(vtkSMPropertyHelper(viewProxy, "ViewTime").GetAsDouble(0));
      this->Internal->TimeLabel->setText(QString("  Frame: %1").arg(viewTime));
    }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onMeasurementGrid()
{
  bool gridVisible = this->Internal->MeasurementGrid->isChecked();
  pqSettings* settings = pqApplicationCore::instance()->settings();
  settings->setValue("VelodyneHDLPlugin/MeasurementGrid/Visibility", gridVisible);

  if (gridVisible)
    {
    this->runPython("_vv.showMeasurementGrid()\n");
    }
  else
    {
    this->runPython("_vv.hideMeasurementGrid()\n");
    }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onOpenSensor()
{
  vvCalibrationDialog dialog;
  int accepted = dialog.exec();

  if (!accepted)
    {
    return;
    }

  QString calibrationFile = dialog.selectedCalibrationFile();

  this->onClose();
  this->runPython(QString("_vv.openSensor('%1')\n").arg(calibrationFile));

  this->filenameLabel()->setText(tr("Live sensor stream."));

  this->Internal->Play->setEnabled(true);
  this->Internal->Record->setEnabled(true);
  this->Internal->SeekForward->setEnabled(true);
  this->Internal->SeekBackward->setEnabled(true);
  this->Internal->GotoStart->setEnabled(true);
  this->Internal->GotoEnd->setEnabled(true);

  this->onPlay();
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onResetView()
{
  this->runPython(QString("_vv.resetCamera()\n"));
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onRecord()
{

  if (!this->Internal->Record->isChecked())
    {

    this->statusLabel()->setText(QString());
    this->runPython(QString("_vv.stopRecording()\n"));

    if (this->Internal->Playing)
      {
      this->runPython(QString("_vv.startStream()\n"));
      }

    }
  else
    {

    pqSettings* settings = pqApplicationCore::instance()->settings();
    QString defaultDir = settings->value("VelodyneHDLPlugin/OpenData/DefaultDir", QDir::homePath()).toString();

    QString selectedFiler("*.pcap");
    QString fileName = QFileDialog::getSaveFileName(this->getMainWindow(), tr("Choose Output File"),
                            defaultDir,
                            tr("pcap (*.pcap)"), &selectedFiler);

    if (fileName.isEmpty())
      {
      this->Internal->Record->setChecked(false);
      return;
      }

    settings->setValue("VelodyneHDLPlugin/OpenData/DefaultDir", QFileInfo(fileName).absoluteDir().absolutePath());

    this->statusLabel()->setText(QString("  Recording file: %1.").arg(QFileInfo(fileName).fileName()));
    this->runPython(QString("_vv.recordFile('%1')\n").arg(fileName));

    if (this->Internal->Playing)
      {
      this->runPython(QString("_vv.startStream()\n"));
      }
    }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onChooseCalibrationFile()
{
  vvCalibrationDialog dialog;
  int accepted = dialog.exec();

  if (!accepted)
    {
    return;
    }

  QString calibrationFile = dialog.selectedCalibrationFile();

  this->runPython(QString("_vv.setCalibrationFile('%1')\n").arg(calibrationFile));
}

//-----------------------------------------------------------------------------
QLabel* pqVelodyneManager::statusBarLogo()
{
  return this->Internal->LogoLabel;
}

//-----------------------------------------------------------------------------
QLabel* pqVelodyneManager::filenameLabel()
{
  return this->Internal->FilenameLabel;
}

//-----------------------------------------------------------------------------
QLabel* pqVelodyneManager::timeLabel()
{
  return this->Internal->TimeLabel;
}

//-----------------------------------------------------------------------------
QLabel* pqVelodyneManager::statusLabel()
{
  return this->Internal->StatusLabel;
}

//-----------------------------------------------------------------------------
pqServer *pqVelodyneManager::getActiveServer()
{
  pqApplicationCore *app = pqApplicationCore::instance();
  pqServerManagerModel *smModel = app->getServerManagerModel();
  pqServer *server = smModel->getItemAtIndex<pqServer*>(0);
  return server;
}

//-----------------------------------------------------------------------------
QWidget *pqVelodyneManager::getMainWindow()
{
  foreach(QWidget *topWidget, QApplication::topLevelWidgets())
    {
    if (qobject_cast<QMainWindow*>(topWidget))
      {
      return topWidget;
      }
    }
  return NULL;
}

//-----------------------------------------------------------------------------
pqView *pqVelodyneManager::findView(pqPipelineSource *source, int port, const QString &viewType)
{
  // Step 1, try to find a view in which the source is already shown.
  if (source)
    {
    foreach (pqView *view, source->getViews())
      {
      pqDataRepresentation *repr = source->getRepresentation(port, view);
      if (repr && repr->isVisible()) return view;
      }
    }

  // Step 2, check to see if the active view is the right type.
  pqView *view = pqActiveView::instance().current();
  if (view->getViewType() == viewType) return view;

  // Step 3, check all the views and see if one is the right type and not
  // showing anything.
  pqApplicationCore *core = pqApplicationCore::instance();
  pqServerManagerModel *smModel = core->getServerManagerModel();
  foreach (view, smModel->findItems<pqView*>())
    {
    if (   view && (view->getViewType() == viewType)
        && (view->getNumberOfVisibleRepresentations() < 1) )
      {
      return view;
      }
    }

  // Give up.  A new view needs to be created.
  return NULL;
}

//-----------------------------------------------------------------------------
pqView* pqVelodyneManager::getRenderView()
{
  return findView(0, 0, pqRenderView::renderViewType());
}
