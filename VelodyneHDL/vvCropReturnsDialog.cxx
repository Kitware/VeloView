// Copyright 2014 Velodyne Acoustics, Inc.
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
#include "vvCropReturnsDialog.h"

#include "ui_vvCropReturnsDialog.h"

#include <pqApplicationCore.h>
#include <pqSettings.h>

#include <QDialog>
#include <QFileDialog>

//-----------------------------------------------------------------------------
class vvCropReturnsDialog::pqInternal : public Ui::vvCropReturnsDialog
{
public:
  pqInternal() : Settings(pqApplicationCore::instance()->settings()) {}

  void saveSettings();
  void restoreSettings();

  pqSettings* const Settings;
};

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::saveSettings()
{
  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/EnableCropping",
    this->CropGroupBox->isChecked());

  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/CropInside",
    this->CropInsideCheckBox->isChecked());

  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/FirstCornerX",
    this->X1SpinBox->value());
  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/FirstCornerY",
    this->Y1SpinBox->value());
  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/FirstCornerZ",
    this->Z1SpinBox->value());
  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/SecondCornerX",
    this->X2SpinBox->value());
  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/SecondCornerY",
    this->Y2SpinBox->value());
  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/SecondCornerZ",
    this->Z2SpinBox->value());
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::restoreSettings()
{
  this->CropGroupBox->setChecked(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/EnableCropping",
      false).toBool());

  this->CropInsideCheckBox->setChecked(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/CropInside",
      this->CropInsideCheckBox->isChecked()).toBool());

  this->X1SpinBox->setValue(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/FirstCornerX",
      this->X1SpinBox->value()).toDouble());
  this->Y1SpinBox->setValue(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/FirstCornerY",
      this->Y1SpinBox->value()).toDouble());
  this->Z1SpinBox->setValue(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/FirstCornerZ",
      this->Z1SpinBox->value()).toDouble());
  this->X2SpinBox->setValue(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/SecondCornerX",
      this->X2SpinBox->value()).toDouble());
  this->Y2SpinBox->setValue(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/SecondCornerY",
      this->Y2SpinBox->value()).toDouble());
  this->Z2SpinBox->setValue(
    this->Settings->value(
      "VelodyneHDLPlugin/CropReturnsDialog/SecondCornerZ",
      this->Z2SpinBox->value()).toDouble());
}

//-----------------------------------------------------------------------------
vvCropReturnsDialog::vvCropReturnsDialog(QWidget *p)
  : QDialog(p), Internal(new pqInternal)
{
  this->Internal->setupUi(this);
  this->Internal->restoreSettings();
}

//-----------------------------------------------------------------------------
vvCropReturnsDialog::~vvCropReturnsDialog()
{
}

//-----------------------------------------------------------------------------
bool vvCropReturnsDialog::croppingEnabled() const
{
  return this->Internal->CropGroupBox->isChecked();
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::setCroppingEnabled(bool checked)
{
  this->Internal->CropGroupBox->setChecked(checked);
}

//-----------------------------------------------------------------------------
bool vvCropReturnsDialog::cropInside() const
{
  return this->Internal->CropInsideCheckBox->isChecked();
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::setCropInside(bool checked)
{
  this->Internal->CropInsideCheckBox->setChecked(checked);
}

//-----------------------------------------------------------------------------
QVector3D vvCropReturnsDialog::firstCorner() const
{
  const pqInternal* const d = this->Internal.data();
  return QVector3D(qMin(d->X1SpinBox->value(), d->X2SpinBox->value()),
                   qMin(d->Y1SpinBox->value(), d->Y2SpinBox->value()),
                   qMin(d->Z1SpinBox->value(), d->Z2SpinBox->value()));
}

//-----------------------------------------------------------------------------
QVector3D vvCropReturnsDialog::secondCorner() const
{
  const pqInternal* const d = this->Internal.data();
  return QVector3D(qMax(d->X1SpinBox->value(), d->X2SpinBox->value()),
                   qMax(d->Y1SpinBox->value(), d->Y2SpinBox->value()),
                   qMax(d->Z1SpinBox->value(), d->Z2SpinBox->value()));
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::setFirstCorner(QVector3D corner)
{
  pqInternal* const d = this->Internal.data();
  d->X1SpinBox->setValue(corner.x());
  d->Y1SpinBox->setValue(corner.y());
  d->Z1SpinBox->setValue(corner.z());
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::setSecondCorner(QVector3D corner)
{
  pqInternal* const d = this->Internal.data();
  d->X2SpinBox->setValue(corner.x());
  d->Y2SpinBox->setValue(corner.y());
  d->Z2SpinBox->setValue(corner.z());
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::accept()
{
  if(this->Internal->saveCheckBox->isChecked())
    {
    this->Internal->saveSettings();
    }
  QDialog::accept();
}
