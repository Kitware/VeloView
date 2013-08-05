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
public:

  vvSelectFramesDialog(QWidget *p=0);
  virtual ~vvSelectFramesDialog();

  enum
  {
    CURRENT_FRAME = 0,
    ALL_FRAMES,
    FRAME_RANGE
  };

  int frameMode() const;
  void setFrameMode(int frameMode);

  int frameStart() const;
  int frameStop() const;
  int frameStride() const;

  void setFrameStart(int frameStart);
  void setFrameStop(int frameStop);
  void setFrameStride(int frameStride);

  void setFrameMinimum(int frameMin);
  void setFrameMaximum(int frameMax);

  void setFrameStrideVisibility(bool visible);

  void saveState();
  void restoreState();

private:


  class pqInternal;
  pqInternal* Internal;

  Q_DISABLE_COPY(vvSelectFramesDialog);
};

#endif
