// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "pqVelodyneManager.h"
#include "vvLoadDataReaction.h"
#include "vvCalibrationDialog.h"
#include "vvPythonQtDecorators.h"

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
#include <vtkPythonInterpreter.h>
#include <vtkTimerLog.h>
#include <vtkVelodyneHDLReader.h>

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

  }

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
  QStringList pythonDirs;
  pythonDirs << QCoreApplication::applicationDirPath()  + "/../Python" // MacOSX application bundle
             << QCoreApplication::applicationDirPath()  + "/../../../../lib/site-packages" // MacOSX application bundle in build directory
             << QCoreApplication::applicationDirPath()  + "/site-packages" // Windows NMake build directory and install tree
             << QCoreApplication::applicationDirPath()  + "/../lib/paraview-4.0" // Windows install tree
             << QCoreApplication::applicationDirPath()  + "/../lib/paraview-4.0/site-packages" // Windows install tree
             << QCoreApplication::applicationDirPath()  + "/../lib/paraview-4.0/site-packages/vtk"; // Windows install tree

  foreach (const QString& dirname, pythonDirs)
    {
      if (QDir(dirname).exists())
        {
        vtkPythonInterpreter::PrependPythonPath(dirname.toAscii().data());
        }
    }

  vtkPythonInterpreter::RunSimpleString("import PythonQt");
  PythonQt::self()->addDecorators(new vvPythonQtDecorators());
  vtkPythonInterpreter::RunSimpleString("import veloview");

  this->runPython(QString(
      "import PythonQt\n"
      "QtGui = PythonQt.QtGui\n"
      "QtCore = PythonQt.QtCore\n"
      "import veloview.applogic as vv\n"
      "vv.start()\n"));

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
void pqVelodyneManager::saveFramesToPCAP(vtkSMSourceProxy* proxy, int startFrame, int endFrame, const QString& filename)
{
  if (!proxy)
    {
    return;
    }

  vtkVelodyneHDLReader* reader = vtkVelodyneHDLReader::SafeDownCast(proxy->GetClientSideObject());
  if (!reader)
    {
    return;
    }

  reader->Open();
  reader->DumpFrames(startFrame, endFrame, filename.toAscii().data());
  reader->Close();
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

  pqSettings* settings = pqApplicationCore::instance()->settings();
  bool gridVisible = settings->value("VelodyneHDLPlugin/MeasurementGrid/Visibility", true).toBool();
  measurementGrid->setChecked(gridVisible);

  this->connect(openSensor, SIGNAL(triggered()), SLOT(onOpenSensor()));
  this->connect(chooseCalibrationFile, SIGNAL(triggered()), SLOT(onChooseCalibrationFile()));

  this->connect(measurementGrid, SIGNAL(triggered()), SLOT(onMeasurementGrid()));

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

      this->runPython(QString("vv.openPCAP('%1', '%2')\n").arg(filename).arg(calibrationFile));
    }
  else
    {
    this->runPython(QString("vv.openData('%1')\n").arg(filename));
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
    this->runPython("vv.showMeasurementGrid()\n");
    }
  else
    {
    this->runPython("vv.hideMeasurementGrid()\n");
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

  this->runPython(QString("vv.openSensor('%1')\n").arg(calibrationFile));
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

  this->runPython(QString("vv.setCalibrationFile('%1')\n").arg(calibrationFile));
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
