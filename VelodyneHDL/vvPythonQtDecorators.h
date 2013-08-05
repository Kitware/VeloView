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

#ifndef __vvPythonQtDecorators_h
#define __vvPythonQtDecorators_h

#include <QObject>
#include "PythonQt.h"

#include "pqVelodyneManager.h"
#include "vvCalibrationDialog.h"
#include "vvConfigure.h"
#include "vvSelectFramesDialog.h"

class VelodyneHDLPython_EXPORT vvPythonQtDecorators : public QObject
{
  Q_OBJECT

public:

  vvPythonQtDecorators(QObject* parent=0) : QObject(parent)
    {
    this->registerClassForPythonQt(&pqVelodyneManager::staticMetaObject);
    this->registerClassForPythonQt(&vvCalibrationDialog::staticMetaObject);
    this->registerClassForPythonQt(&vvSelectFramesDialog::staticMetaObject);
    }

  inline void registerClassForPythonQt(const QMetaObject* metaobject)
    {
    PythonQt::self()->registerClass(metaobject, "paraview");
    }

public slots:


  vvCalibrationDialog* new_vvCalibrationDialog(QWidget* arg0)
    {
    return new vvCalibrationDialog(arg0);
    }


  vvSelectFramesDialog* new_vvSelectFramesDialog(QWidget* arg0)
    {
    return new vvSelectFramesDialog(arg0);
    }



  int frameMode(vvSelectFramesDialog* inst)
    {
    return inst->frameMode();
    }

  void setFrameMode(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameMode(arg0);
    }


  int frameStart(vvSelectFramesDialog* inst)
    {
    return inst->frameStart();
    }

  int frameStop(vvSelectFramesDialog* inst)
    {
    return inst->frameStop();
    }

  int frameStride(vvSelectFramesDialog* inst)
    {
    return inst->frameStride();
    }


  void setFrameStart(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameStart(arg0);
    }

  void setFrameStop(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameStop(arg0);
    }

  void setFrameStride(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameStride(arg0);
    }


  void setFrameMinimum(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameMinimum(arg0);
    }

  void setFrameMaximum(vvSelectFramesDialog* inst, int arg0)
    {
    inst->setFrameMaximum(arg0);
    }


  void setFrameStrideVisibility(vvSelectFramesDialog* inst, bool arg0)
    {
    inst->setFrameStrideVisibility(arg0);
    }


  void saveState(vvSelectFramesDialog* inst)
    {
    inst->saveState();
    }

  void restoreState(vvSelectFramesDialog* inst)
    {
    inst->restoreState();
    }


  void static_pqVelodyneManager_saveFramesToPCAP(vtkSMSourceProxy* arg0, int arg1, int arg2, const QString& arg3)
    {
    pqVelodyneManager::saveFramesToPCAP(arg0, arg1, arg2, arg3);
    }


};

#endif
