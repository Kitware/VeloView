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

#ifndef __pqLidarViewManager_h
#define __pqLidarViewManager_h

#include <QObject>
#include "applicationui_export.h"

class vtkLidarReader;
class vvAppLogic;

class pqPipelineSource;
class pqServer;
class pqView;

class vtkSMSourceProxy;

class vtkPolyData;

class QAction;
class QLabel;
class QWidget;

class APPLICATIONUI_EXPORT pqLidarViewManager : public QObject
{

  Q_OBJECT

public:
  static pqLidarViewManager* instance();

  virtual ~pqLidarViewManager();

  /// Convenience function for getting the current server.
  static pqServer* getActiveServer();

  /// Convenience function for getting the main window.
  static QWidget* getMainWindow();

  /// Convenience function for destroying a pipeline object and all of its
  /// consumers.
  // static void destroyPipelineSourceAndConsumers(pqPipelineSource *source);

  /// Finds a pipeline source with the given SM XML name.  If there is more than
  /// one, the first is returned.
  // static pqPipelineSource *findPipelineSource(const char *SMName);

  void setSource(pqPipelineSource* source);
  pqPipelineSource* source();

  void setup();

  void openData(const QString& filename, const QString& positionFilename);

  void runPython(const QString& statements);

  static void saveFramesToPCAP(
    vtkSMSourceProxy* proxy, int startFrame, int endFrame, const QString& filename);

  static void saveFramesToLAS(vtkLidarReader* reader, vtkPolyData* position, int startFrame,
    int endFrame, const QString& filename, int positionMode);

public slots:

  void pythonStartup();

  void onMeasurementGrid(bool gridVisible);
  void onEnableCrashAnalysis(bool crashAnalysisEnabled);
  void onResetDefaultSettings();
  void onResetCameraLidar();
  void onResetCenterToLidarCenter();

signals:

  void sourceCreated();
  // this signal can be emited to execute a python script on pqPythonShell
  void pythonCommand(const QString&);

private:
  pqLidarViewManager(QObject* p);

  class pqInternal;
  pqInternal* Internal;

  Q_DISABLE_COPY(pqLidarViewManager);
};

#endif
