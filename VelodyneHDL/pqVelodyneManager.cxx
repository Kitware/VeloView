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

#include "vtkLASFileWriter.h"
#include "vtkPVConfig.h" //  needed for PARAVIEW_VERSION
#include "vtkLidarReader.h"
#include "vvPythonQtDecorators.h"

#include <pqActiveObjects.h>
#include <pqApplicationCore.h>
#include <pqDataRepresentation.h>
#include <pqPVApplicationCore.h>
#include <pqPersistentMainWindowStateBehavior.h>
#include <pqPipelineSource.h>
#include <pqPythonDialog.h>
#include <pqPythonManager.h>
#include <pqRenderView.h>
#include <pqServer.h>
#include <pqServerManagerModel.h>
#include <pqSettings.h>
#include <pqView.h>

#include <vtkSMPropertyHelper.h>
#include <vtkSMSourceProxy.h>
#include <vtkSMViewProxy.h>

#include <vtkFieldData.h>
#include <vtkPointData.h>
#include <vtkPythonInterpreter.h>
#include <vtkTimerLog.h>

#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QProcess>
#include <QProgressDialog>
#include <QTimer>

#include <sstream>

//-----------------------------------------------------------------------------
class pqVelodyneManager::pqInternal
{
};

//-----------------------------------------------------------------------------
QPointer<pqVelodyneManager> pqVelodyneManagerInstance = NULL;

//-----------------------------------------------------------------------------
pqVelodyneManager* pqVelodyneManager::instance()
{
  if (!pqVelodyneManagerInstance)
  {
    pqVelodyneManagerInstance = new pqVelodyneManager(pqApplicationCore::instance());
  }

  return pqVelodyneManagerInstance;
}

//-----------------------------------------------------------------------------
pqVelodyneManager::pqVelodyneManager(QObject* p)
  : QObject(p)
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
             << QCoreApplication::applicationDirPath()  + "/../../../../lib" // Mac OS X Plugin build
             << QCoreApplication::applicationDirPath()  + "/../../../../lib/site-packages" // MacOSX application bundle in build directory
             << QCoreApplication::applicationDirPath()  + "/site-packages" // Windows NMake build directory and install tree
             << QCoreApplication::applicationDirPath()  + "/../lib" // Linux build tree
             << QCoreApplication::applicationDirPath()  + "/../lib/site-packages" // Linux build tree
             << QCoreApplication::applicationDirPath()  + "/../lib/paraview-" + PARAVIEW_VERSION // Windows install tree
             << QCoreApplication::applicationDirPath()  + "/../lib/paraview-" + PARAVIEW_VERSION + "/site-packages" // Windows install tree
             << QCoreApplication::applicationDirPath()  + "/../lib/paraview-" + PARAVIEW_VERSION + "/site-packages/vtk" // Windows install tree
             << QCoreApplication::applicationDirPath()  + "/../paraview-" + PARAVIEW_VERSION // Linux 4.3+ install tree
             << QCoreApplication::applicationDirPath()  + "/../paraview-" + PARAVIEW_VERSION + "/site-packages" // Linux 4.3+ install tree
             << QCoreApplication::applicationDirPath()  + "/../paraview-" + PARAVIEW_VERSION + "/site-packages/vtk"; // Linux 4.3+ install tree

  foreach (const QString& dirname, pythonDirs)
  {
    if (QDir(dirname).exists())
    {
      vtkPythonInterpreter::PrependPythonPath(dirname.toLatin1().data());
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

  pqSettings* const settings = pqApplicationCore::instance()->settings();
  const QVariant& gridVisible =
    settings->value("VelodyneHDLPlugin/MeasurementGrid/Visibility", true);

  // Save the current main window state as its original state. This happens in
  // two cases: The first time launching VeloView or when launching VeloView
  // with older/wrong settings which were cleared right before.
  bool shouldSave = true;

  QStringList keys = settings->allKeys();
  for (int keyIndex = 0; keyIndex < keys.size(); ++keyIndex)
  {
    if (keys[keyIndex].contains("OriginalMainWindow"))
    {
      shouldSave = false;
      break;
    }
  }

  if (shouldSave)
  {
    std::cout << "First time launching VeloView, "
                 "saving current state as original state..."
              << std::endl;

    QMainWindow* mainWindow = qobject_cast<QMainWindow*>(getMainWindow());

    settings->saveState(*mainWindow, "OriginalMainWindow");

    // Saving an OriginalMainWondow state means that  wasn't created beforehand.
    new pqPersistentMainWindowStateBehavior(mainWindow);
  }

  this->onMeasurementGrid(gridVisible.toBool());

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
  // printf("runPython(\"%s\")\n", qPrintable(statements));
  pqPythonManager* manager = pqPVApplicationCore::instance()->pythonManager();
  pqPythonDialog* dialog = manager->pythonShellDialog();
  dialog->runString(statements);
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onEnableCrashAnalysis(bool crashAnalysisEnabled)
{
  pqSettings* const Settings = pqApplicationCore::instance()->settings();
  Settings->setValue("VelodyneHDLPlugin/MainWindow/EnableCrashAnalysis", crashAnalysisEnabled);
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onResetDefaultSettings()
{
  QMessageBox messageBox;
  messageBox.setIcon(QMessageBox::Warning);
  std::stringstream ss;
  ss << "This action will reset " << SOFTWARE_NAME << " settings. "
     << "Some settings will need " << SOFTWARE_NAME << " to restart to be completly reset. "
     << "Every unsaved change will be lost. Are you sure you want to reset " << SOFTWARE_NAME << " settings?";
  messageBox.setText(ss.str().c_str());
  messageBox.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);

  if (messageBox.exec() == QMessageBox::Ok)
  {
    pqApplicationCore* const app = pqApplicationCore::instance();
    pqSettings* const settings = app->settings();
    QMainWindow* const mainWindow = qobject_cast<QMainWindow*>(getMainWindow());

    // Restore the original main window state before clearing settings, as clearing
    // settings doesn't update the UI.
    settings->restoreState("OriginalMainWindow", *mainWindow);

    settings->clear();

    // Resave the current main window state as the original main window state in
    // the settings
    settings->saveState(*mainWindow, "OriginalMainWindow");

    // Quit the current VeloView instance and restart a new one.
    qApp->quit();
    QProcess::startDetached(qApp->arguments()[0]);
  }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::saveFramesToPCAP(
  vtkSMSourceProxy* proxy, int startFrame, int endFrame, const QString& filename)
{
  if (!proxy)
  {
    return;
  }

  vtkLidarReader* reader = vtkLidarReader::SafeDownCast(proxy->GetClientSideObject());
  if (!reader)
  {
    return;
  }

  reader->Open();
  reader->SaveFrame(startFrame, endFrame, filename.toLatin1().data());
  reader->Close();
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::saveFramesToLAS(vtkLidarReader* reader, vtkPolyData* position,
  int startFrame, int endFrame, const QString& filename, int positionMode)
{
  if (!reader || (positionMode > 0 && !position))
  {
    return;
  }

  // initialize origin point
  double northing, easting, height;
  easting = northing = height = 0;

  // projection transform parameters
  int gcs, in, out, utmZone;
  gcs = in = out = utmZone = 0;

  // data accuracy
  double neTol, hTol;
  hTol = neTol = 1e-3;

  bool isLatLon = false;

  vtkLASFileWriter writer(qPrintable(filename));

  // not sensor relative; it can be
  // relative registered data or
  // georeferenced data
  if (positionMode > 0)
  {

    // Georeferenced data
    if (positionMode > 1)
    {
      // Since the data are georeferenced here, we must
      // check that a position reader is provided
      if (position)
      {
        vtkDataArray* const zoneData = position->GetFieldData()->GetArray("zone");
        vtkDataArray* const eastingData = position->GetPointData()->GetArray("easting");
        vtkDataArray* const northingData = position->GetPointData()->GetArray("northing");
        vtkDataArray* const heightData = position->GetPointData()->GetArray("height");

        if (zoneData && zoneData->GetNumberOfTuples() && eastingData &&
          eastingData->GetNumberOfTuples() && northingData && northingData->GetNumberOfTuples() &&
          heightData && heightData->GetNumberOfTuples())
        {
          // We assume that eastingData, norhtingData and heightData are in system reference
          // coordinates (srs) of UTM zoneData
          utmZone = static_cast<int>(zoneData->GetComponent(0, 0));

          // should in some cases use 32700? 32600 is for northern UTM zone, 32700 for southern UTM zone
          gcs = 32600 + utmZone;

          out = gcs;
          if (positionMode == 3) // Absolute lat/lon
          {
            in = gcs; // ...or 32700?
            out = 4326; // lat/lon (espg id code for lat-long-alt coordinates)
            neTol = 1e-8; // about 1mm;
            isLatLon = true;
          }

          northing = northingData->GetComponent(0, 0);
          easting = eastingData->GetComponent(0, 0);
          height = heightData->GetComponent(0, 0);
        }
      }
    }
  }

  std::cout << "origin : [" << northing << ";" << easting << ";" << height << "]" << std::endl;
  std::cout << "gcs : " << gcs << std::endl;

  writer.SetPrecision(neTol, hTol);
  writer.SetGeoConversion(in, out, utmZone, isLatLon);
  writer.SetOrigin(gcs, easting, northing, height);

  QProgressDialog progress("Exporting LAS...", "Abort Export", startFrame,
    startFrame + (endFrame - startFrame) * 2, getMainWindow());
  progress.setWindowModality(Qt::WindowModal);

  reader->Open();
  for (int frame = startFrame; frame <= endFrame; ++frame)
  {
    progress.setValue(frame);

    if (progress.wasCanceled())
    {
      reader->Close();
      return;
    }

    const vtkSmartPointer<vtkPolyData>& data = reader->GetFrame(frame);
    writer.UpdateMetaData(data.GetPointer());
  }

  writer.FlushMetaData();

  for (int frame = startFrame; frame <= endFrame; ++frame)
  {
    progress.setValue(endFrame + (frame - startFrame));

    if (progress.wasCanceled())
    {
      reader->Close();
      return;
    }

    const vtkSmartPointer<vtkPolyData>& data = reader->GetFrame(frame);
    writer.WriteFrame(data.GetPointer());
  }

  reader->Close();
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::setup()
{
  QTimer::singleShot(0, this, SLOT(pythonStartup()));
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::openData(const QString& filename, const QString& positionFilename)
{
  if (!positionFilename.isEmpty())
  {
    this->runPython(QString("vv.openPCAP('%1', '%2')\n").arg(filename, positionFilename));
  }
  else if (QFileInfo(filename).suffix() == "pcap")
  {
    this->runPython(QString("vv.openPCAP('%1')\n").arg(filename));
  }
  else
  {
    this->runPython(QString("vv.openData('%1')\n").arg(filename));
  }
}

//-----------------------------------------------------------------------------
void pqVelodyneManager::onMeasurementGrid(bool gridVisible)
{
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
  this->runPython("vv.openSensor()\n");
}

//-----------------------------------------------------------------------------
pqServer* pqVelodyneManager::getActiveServer()
{
  pqApplicationCore* app = pqApplicationCore::instance();
  pqServerManagerModel* smModel = app->getServerManagerModel();
  pqServer* server = smModel->getItemAtIndex<pqServer*>(0);
  return server;
}

//-----------------------------------------------------------------------------
QWidget* pqVelodyneManager::getMainWindow()
{
  foreach (QWidget* topWidget, QApplication::topLevelWidgets())
  {
    if (qobject_cast<QMainWindow*>(topWidget))
    {
      return topWidget;
    }
  }
  return NULL;
}
