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
#ifndef __vvSelectFramesDialog_h
#define __vvSelectFramesDialog_h

#include <QDialog>

#include "vvConfigure.h"

class VelodyneHDLPlugin_EXPORT vvSelectFramesDialog : public QDialog
{
  Q_OBJECT
  Q_PROPERTY(int frameMode READ frameMode WRITE setFrameMode)
  Q_PROPERTY(int frameStart READ frameStart WRITE setFrameStart)
  Q_PROPERTY(int frameStop READ frameStop WRITE setFrameStop)
  Q_PROPERTY(int frameStride READ frameStride WRITE setFrameStride)
  Q_PROPERTY(int framePack READ framePack WRITE setFramePack)
  Q_PROPERTY(int frameMinimum READ frameMinimun WRITE setFrameMinimum)
  Q_PROPERTY(int frameMaximum READ frameMaximun WRITE setFrameMaximum)
  Q_PROPERTY(int frameTransform READ frameTransform WRITE setFrameTransform)
  Q_PROPERTY(bool frameStrideVisibility READ frameStrideVisibility WRITE setFrameStrideVisibility)
  Q_PROPERTY(bool framePackVisibility READ framePackVisibility WRITE setFramePackVisibility)
  Q_PROPERTY(bool frameTransformVisibility READ frameTransformVisibility WRITE setFrameTransformVisibility)
  Q_ENUMS(FrameMode FramePack FrameTransform)

public:
  vvSelectFramesDialog(QWidget* p = 0);
  virtual ~vvSelectFramesDialog();

  enum FrameMode
  {
    CURRENT_FRAME = 0,
    ALL_FRAMES,
    FRAME_RANGE
  };
  enum FramePack
  {
    SINGLE_FILE = 0,
    FILE_PER_FRAME
  };
  enum FrameTransform
  {
    SENSOR = 0,
    RELATIVE_GEOPOSITION,
    ABSOLUTE_GEOPOSITION_UTM,
    ABSOLUTE_GEOPOSITION_LATLON,
  };

  int frameMode() const;
  int frameStart() const;
  int frameStop() const;
  int frameStride() const;
  int framePack() const;
  int frameTransform() const;

  int frameMaximun() const;
  int frameMinimun() const;

  bool frameStrideVisibility() const;
  bool framePackVisibility() const;
  bool frameTransformVisibility() const;

public slots:
  virtual void accept();

  void setFrameMode(int frameMode);
  void setFrameStart(int frameStart);
  void setFrameStop(int frameStop);
  void setFrameStride(int frameStride);
  void setFramePack(int framePack);
  void setFrameTransform(int frameTransform);

  void setFrameMinimum(int frameMin);
  void setFrameMaximum(int frameMax);

  void setFrameStrideVisibility(bool visible);
  void setFramePackVisibility(bool visible);
  void setFrameTransformVisibility(bool visible);

  void saveState();
  void restoreState();

protected:
  virtual void showEvent(QShowEvent*);

private:
  class pqInternal;
  pqInternal* Internal;

  Q_DISABLE_COPY(vvSelectFramesDialog)
};

#endif
