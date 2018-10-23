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

#include <QMessageBox>



//-----------------------------------------------------------------------------
class vvSelectFramesDialog::pqInternal : public Ui::vvSelectFramesDialog
{
};

//-----------------------------------------------------------------------------
vvSelectFramesDialog::vvSelectFramesDialog(QWidget* p)
  : QDialog(p)
{
  this->Internal = new pqInternal;
  this->Internal->setupUi(this);
  this->Internal->FrameStart->clearFocus();
}

//-----------------------------------------------------------------------------
vvSelectFramesDialog::~vvSelectFramesDialog()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::accept()
{
  if (this->Internal->FrameStop->value() < this->Internal->FrameStart->value())
  {
    QMessageBox::critical(this, "Invalid frame range",
      "The requested frame range is not valid. "
      "The start frame must be less than or equal to the stop frame.");
    return;
  }

  this->saveState();
  QDialog::accept();
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
int vvSelectFramesDialog::framePack() const
{
  if (this->Internal->FilePerFrameButton->isChecked())
  {
    return FILE_PER_FRAME;
  }
  else
  {
    return SINGLE_FILE;
  }
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFramePack(int framePack)
{
  if (framePack == SINGLE_FILE)
  {
    this->Internal->SingleFileButton->setChecked(true);
  }
  else if (framePack == FILE_PER_FRAME)
  {
    this->Internal->FilePerFrameButton->setChecked(true);
  }
}

//-----------------------------------------------------------------------------
int vvSelectFramesDialog::frameTransform() const
{
  if (this->Internal->RelativeButton->isChecked())
  {
    return RELATIVE_GEOPOSITION;
  }
  else if (this->Internal->AbsoluteUtmButton->isChecked())
  {
    return ABSOLUTE_GEOPOSITION_UTM;
  }
  else if (this->Internal->AbsoluteLatLonButton->isChecked())
  {
    return ABSOLUTE_GEOPOSITION_LATLON;
  }
  else
  {
    return SENSOR;
  }
}

//-----------------------------------------------------------------------------
int vvSelectFramesDialog::frameMaximun() const
{
  return std::max( this->Internal->FrameStart->maximum(), this->Internal->FrameStop->maximum());
}

//-----------------------------------------------------------------------------
int vvSelectFramesDialog::frameMinimun() const
{
  return std::max( this->Internal->FrameStart->minimum(), this->Internal->FrameStop->minimum());;
}

//-----------------------------------------------------------------------------
bool vvSelectFramesDialog::frameStrideVisibility() const
{
  return  this->Internal->FrameStrideContainer->isVisible();;
}

//-----------------------------------------------------------------------------
bool vvSelectFramesDialog::framePackVisibility() const
{
  return this->Internal->FramePackContainer->isVisible();;
}

//-----------------------------------------------------------------------------
bool vvSelectFramesDialog::frameTransformVisibility() const
{
  return  this->Internal->FrameTransformContainer->isVisible();;
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameTransform(int frameTransform)
{
  if (frameTransform == SENSOR)
  {
    this->Internal->SensorButton->setChecked(true);
  }
  else if (frameTransform == RELATIVE_GEOPOSITION)
  {
    this->Internal->RelativeButton->setChecked(true);
  }
  else if (frameTransform == ABSOLUTE_GEOPOSITION_UTM)
  {
    this->Internal->AbsoluteUtmButton->setChecked(true);
  }
  else if (frameTransform == ABSOLUTE_GEOPOSITION_LATLON)
  {
    this->Internal->AbsoluteLatLonButton->setChecked(true);
  }
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameMinimum(int frameMin)
{
  this->Internal->FrameStart->setMinimum(frameMin);
  this->Internal->FrameStop->setMinimum(frameMin);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameMaximum(int frameMax)
{
  this->Internal->FrameStart->setMaximum(frameMax);
  this->Internal->FrameStop->setMaximum(frameMax);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameStrideVisibility(bool visible)
{
  this->Internal->FrameStrideContainer->setVisible(visible);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFramePackVisibility(bool visible)
{
  this->Internal->FramePackContainer->setVisible(visible);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::setFrameTransformVisibility(bool visible)
{
  this->Internal->FrameTransformContainer->setVisible(visible);
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::showEvent(QShowEvent* e)
{
  QDialog::showEvent(e);
  this->resize(this->width(), this->minimumSizeHint().height());
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::saveState()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Mode", this->frameMode());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Start", this->frameStart());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Stop", this->frameStop());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Stride", this->frameStride());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Pack", this->framePack());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Transform", this->frameTransform());
  settings->setValue("VelodyneHDLPlugin/SelectFramesDialog/Geometry", this->saveGeometry());
}

//-----------------------------------------------------------------------------
void vvSelectFramesDialog::restoreState()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  this->restoreGeometry(
    settings->value("VelodyneHDLPlugin/SelectFramesDialog/Geometry").toByteArray());
  this->setFrameMode(
    settings->value("VelodyneHDLPlugin/SelectFramesDialog/Mode", CURRENT_FRAME).toInt());
  this->setFrameStart(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Start", 0).toInt());
  this->setFrameStop(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Stop", 10).toInt());
  this->setFrameStride(settings->value("VelodyneHDLPlugin/SelectFramesDialog/Stride", 1).toInt());
  this->setFramePack(
    settings->value("VelodyneHDLPlugin/SelectFramesDialog/Pack", SINGLE_FILE).toInt());
  this->setFrameTransform(
    settings->value("VelodyneHDLPlugin/SelectFramesDialog/Transform", SENSOR).toInt());
}
