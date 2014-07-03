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
#include "vvSelectFramesDialog.h"

#include "ui_vvSelectFramesDialog.h"

#include <pqApplicationCore.h>
#include <pqSettings.h>

//-----------------------------------------------------------------------------
class vvSelectFramesDialog::pqInternal : public Ui::vvSelectFramesDialog
{
};

//-----------------------------------------------------------------------------
vvSelectFramesDialog::vvSelectFramesDialog(QWidget *p) : QDialog(p)
{
  this->Internal = new pqInternal;
  this->Internal->setupUi(this);
  this->Internal->FrameStart->clearFocus();
}

//-----------------------------------------------------------------------------
vvSelectFramesDialog::~vvSelectFramesDialog()
{
  this->saveState();
  delete this->Internal;
}

//-----------------------------------------------------------------------------
int vvSelectFramesDialog::frameMode() const
{
  if (this->Internal->CurrentFrameButton->isChecked())
    {
    return CURRENT_FRAME;
    }
  else if (this->Internal->AllFramesButton->isChecked())
    {
    return ALL_FRAMES;
    }
  else
    {
    return FRAME_RANGE;
    }
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameMode(int frameMode)
{
  if (frameMode == CURRENT_FRAME)
    {
    this->Internal->CurrentFrameButton->setChecked(true);
    }
  else if (frameMode == ALL_FRAMES)
    {
    this->Internal->AllFramesButton->setChecked(true);
    }
  else if (frameMode == FRAME_RANGE)
    {
    this->Internal->FrameRangeButton->setChecked(true);
    }
}

//-----------------------------------------------------------------------------
int vvSelectFramesDialog::frameStart() const
{
  return this->Internal->FrameStart->value();
}

//-----------------------------------------------------------------------------
int vvSelectFramesDialog::frameStop() const
{
  return this->Internal->FrameStop->value();
}

//-----------------------------------------------------------------------------
int vvSelectFramesDialog::frameStride() const
{
  return this->Internal->FrameStride->value();
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameStart(int frameStart)
{
  this->Internal->FrameStart->setValue(frameStart);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameStop(int frameStop)
{
  this->Internal->FrameStop->setValue(frameStop);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameStride(int frameStride)
{
  this->Internal->FrameStride->setValue(frameStride);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameMinimum(int frameMin)
{
  this->Internal->FrameStart->setMinimum(frameMin);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameMaximum(int frameMax)
{
  this->Internal->FrameStop->setMaximum(frameMax);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameStrideVisibility(bool visible)
{
  this->Internal->FrameStride->setVisible(visible);
  this->Internal->FrameStrideLabel->setVisible(visible);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::saveState()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Mode", this->frameMode());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Start", this->frameStart());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Stop", this->frameStop());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Stride", this->frameStride());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Geometry", this->saveGeometry());
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::restoreState()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  this->restoreGeometry(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Geometry").toByteArray());
  this->setFrameMode(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Mode", CURRENT_FRAME).toInt());
  this->setFrameStart(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Start", 0).toInt());
  this->setFrameStop(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Stop", 10).toInt());
  this->setFrameStride(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Stride", 1).toInt());
}
