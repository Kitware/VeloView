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
#ifndef __vvCalibrationDialog_h
#define __vvCalibrationDialog_h

#include "vvConfigure.h"

#include <QDialog>
#include <QMatrix4x4>

class VelodyneHDLPlugin_EXPORT vvCalibrationDialog : public QDialog
{
  Q_OBJECT

public:
  vvCalibrationDialog(QWidget *p=0);
  virtual ~vvCalibrationDialog();

  Q_INVOKABLE QString selectedCalibrationFile() const;

  Q_INVOKABLE QStringList calibrationFiles() const;

  Q_INVOKABLE QMatrix4x4 sensorTransform() const;

  Q_INVOKABLE double gpsYaw() const;
  Q_INVOKABLE double gpsRoll() const;
  Q_INVOKABLE double gpsPitch() const;

public slots:
  virtual void accept();

protected slots:
  void addFile();
  void removeSelectedFile();
  void onCurrentRowChanged(int row);

private:
  class pqInternal;
  QScopedPointer<pqInternal> Internal;

  Q_DISABLE_COPY(vvCalibrationDialog);
};

#endif
