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

#include <ctkDoubleRangeSlider.h>

#include <sstream>

//-----------------------------------------------------------------------------
class vvCropReturnsDialog::pqInternal : public Ui::vvCropReturnsDialog
{
public:
  pqInternal()
    : Settings(pqApplicationCore::instance()->settings())
  {
  }

  void saveSettings();
  void restoreSettings();
  void SetSphericalSettings();
  void SetCartesianSettings();
  void ActivateSpinBox();
  void DesactivateSpinBox();
  void InitializeDoubleRangeSlider();
  void SwitchSliderMode(bool isSliderMode);
  void GetCropRegion(double output[6]);
  void onXSliderChanged(double vmin, double vmax);
  void onYSliderChanged(double vmin, double vmax);
  void onZSliderChanged(double vmin, double vmax);
  void updateRangeValues(bool isSliderMode);

  ctkDoubleRangeSlider XDoubleRangeSlider;
  ctkDoubleRangeSlider YDoubleRangeSlider;
  ctkDoubleRangeSlider ZDoubleRangeSlider;

  QLabel XRangeLabel;
  QLabel YRangeLabel;
  QLabel ZRangeLabel;

  double xRange[2];
  double yRange[2];
  double zRange[2];

  pqSettings* const Settings;
};

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::saveSettings()
{
  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/EnableCropping", this->CropGroupBox->isChecked());

  this->Settings->setValue(
    "VelodyneHDLPlugin/CropReturnsDialog/CropOutside", this->CropOutsideCheckBox->isChecked());

  this->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/FirstCornerX", xRange[0]);
  this->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/FirstCornerY", yRange[0]);
  this->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/FirstCornerZ", zRange[0]);
  this->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/SecondCornerX", xRange[1]);
  this->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/SecondCornerY", yRange[1]);
  this->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/SecondCornerZ", zRange[1]);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::restoreSettings()
{
  this->sphericalRadioButton->setChecked(
    this->Settings->value("VelodyneHDLPlugin/CropReturnsDialog/sphericalRadioButton", false)
      .toBool());

  this->cartesianRadioButton->setChecked(
    this->Settings->value("VelodyneHDLPlugin/CropReturnsDialog/cartesianRadioButton", false)
      .toBool());

  if (this->cartesianRadioButton->isChecked())
  {
    this->SetCartesianSettings();
  }

  if (this->sphericalRadioButton->isChecked())
  {
    this->SetSphericalSettings();
  }

  this->CropGroupBox->setChecked(
    this->Settings->value("VelodyneHDLPlugin/CropReturnsDialog/EnableCropping", false).toBool());

  this->CropOutsideCheckBox->setChecked(this->Settings
                                          ->value("VelodyneHDLPlugin/CropReturnsDialog/CropOutside",
                                            this->CropOutsideCheckBox->isChecked())
                                          .toBool());

  this->X1SpinBox->setValue(
    this->Settings
      ->value("VelodyneHDLPlugin/CropReturnsDialog/FirstCornerX", this->X1SpinBox->value())
      .toDouble());
  this->Y1SpinBox->setValue(
    this->Settings
      ->value("VelodyneHDLPlugin/CropReturnsDialog/FirstCornerY", this->Y1SpinBox->value())
      .toDouble());
  this->Z1SpinBox->setValue(
    this->Settings
      ->value("VelodyneHDLPlugin/CropReturnsDialog/FirstCornerZ", this->Z1SpinBox->value())
      .toDouble());
  this->X2SpinBox->setValue(
    this->Settings
      ->value("VelodyneHDLPlugin/CropReturnsDialog/SecondCornerX", this->X2SpinBox->value())
      .toDouble());
  this->Y2SpinBox->setValue(
    this->Settings
      ->value("VelodyneHDLPlugin/CropReturnsDialog/SecondCornerY", this->Y2SpinBox->value())
      .toDouble());
  this->Z2SpinBox->setValue(
    this->Settings
      ->value("VelodyneHDLPlugin/CropReturnsDialog/SecondCornerZ", this->Z2SpinBox->value())
      .toDouble());

  xRange[0] = this->X1SpinBox->value();
  xRange[1] = this->X2SpinBox->value();
  yRange[0] = this->Y1SpinBox->value();
  yRange[1] = this->Y2SpinBox->value();
  zRange[0] = this->Z1SpinBox->value();
  zRange[1] = this->Z2SpinBox->value();

  this->X1SpinBox->setValue(xRange[0]);
  this->X2SpinBox->setValue(xRange[1]);
  this->Y1SpinBox->setValue(yRange[0]);
  this->Y2SpinBox->setValue(yRange[1]);
  this->Z1SpinBox->setValue(zRange[0]);
  this->Z2SpinBox->setValue(zRange[1]);

  this->XDoubleRangeSlider.setMinimumValue(xRange[0]);
  this->XDoubleRangeSlider.setMaximumValue(xRange[1]);
  this->YDoubleRangeSlider.setMinimumValue(yRange[0]);
  this->YDoubleRangeSlider.setMaximumValue(yRange[1]);
  this->ZDoubleRangeSlider.setMinimumValue(zRange[0]);
  this->ZDoubleRangeSlider.setMaximumValue(zRange[1]);
}

//-----------------------------------------------------------------------------
vvCropReturnsDialog::vvCropReturnsDialog(QWidget* p)
  : QDialog(p)
  , Internal(new pqInternal)
{
  this->Internal->setupUi(this);

  this->Internal->InitializeDoubleRangeSlider();

  connect(
    this->Internal->X1SpinBox, SIGNAL(valueChanged(double)), this, SLOT(onSpinBoxChanged(double)));
  connect(
    this->Internal->X2SpinBox, SIGNAL(valueChanged(double)), this, SLOT(onSpinBoxChanged(double)));
  connect(
    this->Internal->Y1SpinBox, SIGNAL(valueChanged(double)), this, SLOT(onSpinBoxChanged(double)));
  connect(
    this->Internal->Y2SpinBox, SIGNAL(valueChanged(double)), this, SLOT(onSpinBoxChanged(double)));
  connect(
    this->Internal->Z1SpinBox, SIGNAL(valueChanged(double)), this, SLOT(onSpinBoxChanged(double)));
  connect(
    this->Internal->Z2SpinBox, SIGNAL(valueChanged(double)), this, SLOT(onSpinBoxChanged(double)));
  connect(&this->Internal->XDoubleRangeSlider, SIGNAL(positionsChanged(double, double)), this,
    SLOT(onXSliderChanged(double, double)));
  connect(&this->Internal->YDoubleRangeSlider, SIGNAL(positionsChanged(double, double)), this,
    SLOT(onYSliderChanged(double, double)));
  connect(&this->Internal->ZDoubleRangeSlider, SIGNAL(positionsChanged(double, double)), this,
    SLOT(onZSliderChanged(double, double)));
  connect(this->Internal->sliderModeCheckBox, SIGNAL(clicked()), this, SLOT(onSliderBoxToggled()));
  connect(this->Internal->noneRadioButton, SIGNAL(clicked()), this, SLOT(onNoneToggled()));
  connect(
    this->Internal->cartesianRadioButton, SIGNAL(clicked()), this, SLOT(onCartesianToggled()));
  connect(
    this->Internal->sphericalRadioButton, SIGNAL(clicked()), this, SLOT(onSphericalToggled()));
  connect(this->Internal->CropGroupBox, SIGNAL(clicked()), this, SLOT(onCropGroupBoxToggled()));

  // Without configuration file, no croping is perform
  this->Internal->noneRadioButton->setChecked(true);
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
bool vvCropReturnsDialog::cropOutside() const
{
  return this->Internal->CropOutsideCheckBox->isChecked();
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::setCropOutside(bool checked)
{
  this->Internal->CropOutsideCheckBox->setChecked(checked);
}

//-----------------------------------------------------------------------------
QVector3D vvCropReturnsDialog::firstCorner() const
{
  double cropRegion[6];
  this->Internal->GetCropRegion(cropRegion);

  if (this->Internal->sphericalRadioButton->isChecked())
  {
    return QVector3D(cropRegion[0], cropRegion[2], qMin(cropRegion[4], cropRegion[5]));
  }
  else
  {
    return QVector3D(qMin(cropRegion[0], cropRegion[1]), qMin(cropRegion[2], cropRegion[3]),
      qMin(cropRegion[4], cropRegion[5]));
  }
}

//-----------------------------------------------------------------------------
QVector3D vvCropReturnsDialog::secondCorner() const
{
  double cropRegion[6];
  this->Internal->GetCropRegion(cropRegion);

  if (this->Internal->sphericalRadioButton->isChecked())
  {
    return QVector3D(cropRegion[1], cropRegion[3], qMax(cropRegion[4], cropRegion[5]));
  }
  else
  {
    return QVector3D(qMax(cropRegion[0], cropRegion[1]), qMax(cropRegion[2], cropRegion[3]),
      qMax(cropRegion[4], cropRegion[5]));
  }
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
  if (this->Internal->saveCheckBox->isChecked())
  {
    this->Internal->saveSettings();
  }

  this->Internal->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/cartesianRadioButton",
    this->Internal->cartesianRadioButton->isChecked());
  this->Internal->Settings->setValue("VelodyneHDLPlugin/CropReturnsDialog/sphericalRadioButton",
    this->Internal->sphericalRadioButton->isChecked());

  QDialog::accept();
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onCartesianToggled()
{
  this->Internal->SetCartesianSettings();
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onSphericalToggled()
{
  this->Internal->SetSphericalSettings();
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onSliderBoxToggled()
{
  this->Internal->SwitchSliderMode(this->Internal->sliderModeCheckBox->isChecked());
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::SetSphericalSettings()
{
  this->ActivateSpinBox();
  // change the labels
  // list of unicode symbol : http://sites.psu.edu/symbolcodes/languages/ancient/greek/greekchart/
  this->XLabel->setText("Rotational angle");
  this->YLabel->setText("Vertical angle");
  this->ZLabel->setText("Distance");

  // Here we take the spherical coordinates used in mathematics (and not physic)
  // (r,theta,phi)
  double minR = 0, maxR = 240;
  double minTheta = 0, maxTheta = 360; // Rotational Angle
  double minPhi = -90, maxPhi = 90;    // Vertical Angle
  // theta is between [minTheta,maxTheta] - Rotational Angle
  this->X1SpinBox->setMinimum(minTheta);
  this->X2SpinBox->setMinimum(minTheta);
  this->XDoubleRangeSlider.setMinimum(minTheta);
  this->X1SpinBox->setMaximum(maxTheta);
  this->X2SpinBox->setMaximum(maxTheta);
  this->XDoubleRangeSlider.setMaximum(maxTheta);
  // phi is between [minPhi,maxPhi] - Vertical Angle
  this->Y1SpinBox->setMinimum(minPhi);
  this->Y2SpinBox->setMinimum(minPhi);
  this->YDoubleRangeSlider.setMinimum(minPhi);
  this->Y1SpinBox->setMaximum(maxPhi);
  this->Y2SpinBox->setMaximum(maxPhi);
  this->YDoubleRangeSlider.setMaximum(maxPhi);
  // R is positive
  this->Z1SpinBox->setMinimum(minR);
  this->Z2SpinBox->setMinimum(minR);
  this->ZDoubleRangeSlider.setMinimum(minR);
  this->Z1SpinBox->setMaximum(maxR);
  this->Z2SpinBox->setMaximum(maxR);
  this->ZDoubleRangeSlider.setMaximum(maxR);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::SetCartesianSettings()
{
  double maxV = 300;
  double minV = -maxV;
  this->ActivateSpinBox();
  // change the labels
  this->XLabel->setText("X");
  this->YLabel->setText("Y");
  this->ZLabel->setText("Z");
  // change the bounds
  // X [-10000,10000]
  this->X1SpinBox->setMinimum(minV);
  this->X2SpinBox->setMinimum(minV);
  this->XDoubleRangeSlider.setMinimum(minV);
  this->X1SpinBox->setMaximum(maxV);
  this->X2SpinBox->setMaximum(maxV);
  this->XDoubleRangeSlider.setMaximum(maxV);
  // Y [-10000,10000]
  this->Y1SpinBox->setMinimum(minV);
  this->Y2SpinBox->setMinimum(minV);
  this->YDoubleRangeSlider.setMinimum(minV);
  this->Y1SpinBox->setMaximum(maxV);
  this->Y2SpinBox->setMaximum(maxV);
  this->YDoubleRangeSlider.setMaximum(maxV);
  // Z [-10000,10000]
  this->Z1SpinBox->setMinimum(minV);
  this->Z2SpinBox->setMinimum(minV);
  this->ZDoubleRangeSlider.setMinimum(minV);
  this->Z1SpinBox->setMaximum(maxV);
  this->Z2SpinBox->setMaximum(maxV);
  this->ZDoubleRangeSlider.setMaximum(maxV);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onNoneToggled()
{
  this->Internal->DesactivateSpinBox();
  this->Internal->CropOutsideCheckBox->setChecked(false);
}

//-----------------------------------------------------------------------------
int vvCropReturnsDialog::GetCropMode() const
{
  // Crop mode :
  // 0 -> cartesian
  // 1 -> Spherical
  // 2 -> Cylindric
  // 3 -> None
  if (this->Internal->cartesianRadioButton->isChecked())
  {
    return 1;
  }
  else if (this->Internal->sphericalRadioButton->isChecked())
  {
    return 2;
  }
  else if (this->Internal->noneRadioButton->isChecked())
  {
    return 0;
  }
  else
  {
    return 3;
  }
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::ActivateSpinBox()
{
  this->XLabel->setDisabled(false);
  this->YLabel->setDisabled(false);
  this->ZLabel->setDisabled(false);

  this->X1SpinBox->setDisabled(false);
  this->X2SpinBox->setDisabled(false);

  this->Y1SpinBox->setDisabled(false);
  this->Y2SpinBox->setDisabled(false);

  this->Z1SpinBox->setDisabled(false);
  this->Z2SpinBox->setDisabled(false);

  this->XDoubleRangeSlider.setDisabled(false);
  this->YDoubleRangeSlider.setDisabled(false);
  this->ZDoubleRangeSlider.setDisabled(false);

  this->CropOutsideCheckBox->setDisabled(false);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::InitializeDoubleRangeSlider()
{
  this->XLayout->addWidget(&this->XDoubleRangeSlider);
  this->XLayout->addWidget(&this->XRangeLabel);
  this->XDoubleRangeSlider.setOrientation(Qt::Horizontal);
  this->XDoubleRangeSlider.setVisible(false);
  this->XRangeLabel.setVisible(false);

  this->YLayout->addWidget(&this->YDoubleRangeSlider);
  this->YLayout->addWidget(&this->YRangeLabel);
  this->YDoubleRangeSlider.setOrientation(Qt::Horizontal);
  this->YDoubleRangeSlider.setVisible(false);
  this->YRangeLabel.setVisible(false);

  this->ZLayout->addWidget(&this->ZDoubleRangeSlider);
  this->ZLayout->addWidget(&this->ZRangeLabel);
  this->ZDoubleRangeSlider.setOrientation(Qt::Horizontal);
  this->ZDoubleRangeSlider.setVisible(false);
  this->ZRangeLabel.setVisible(false);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::SwitchSliderMode(bool isSliderMode)
{
  this->XDoubleRangeSlider.setVisible(isSliderMode);
  this->YDoubleRangeSlider.setVisible(isSliderMode);
  this->ZDoubleRangeSlider.setVisible(isSliderMode);
  this->XRangeLabel.setVisible(isSliderMode);
  this->YRangeLabel.setVisible(isSliderMode);
  this->ZRangeLabel.setVisible(isSliderMode);

  this->X1SpinBox->setVisible(!isSliderMode);
  this->X2SpinBox->setVisible(!isSliderMode);
  this->Y1SpinBox->setVisible(!isSliderMode);
  this->Y2SpinBox->setVisible(!isSliderMode);
  this->Z1SpinBox->setVisible(!isSliderMode);
  this->Z2SpinBox->setVisible(!isSliderMode);

  if (isSliderMode)
  {
    this->onXSliderChanged(this->xRange[0], this->xRange[1]);
    this->onYSliderChanged(this->yRange[0], this->yRange[1]);
    this->onZSliderChanged(this->zRange[0], this->zRange[1]);
  }

  this->XDoubleRangeSlider.setMinimumValue(this->xRange[0]);
  this->YDoubleRangeSlider.setMinimumValue(this->yRange[0]);
  this->ZDoubleRangeSlider.setMinimumValue(this->zRange[0]);
  this->XDoubleRangeSlider.setMaximumValue(this->xRange[1]);
  this->YDoubleRangeSlider.setMaximumValue(this->yRange[1]);
  this->ZDoubleRangeSlider.setMaximumValue(this->zRange[1]);

  this->X1SpinBox->setValue(this->xRange[0]);
  this->X2SpinBox->setValue(this->xRange[1]);
  this->Y1SpinBox->setValue(this->yRange[0]);
  this->Y2SpinBox->setValue(this->yRange[1]);
  this->Z1SpinBox->setValue(this->zRange[0]);
  this->Z2SpinBox->setValue(this->zRange[1]);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::GetCropRegion(double output[6])
{
  if (this->sliderModeCheckBox->isChecked())
  {
    output[0] = this->XDoubleRangeSlider.minimumValue();
    output[1] = this->XDoubleRangeSlider.maximumValue();

    output[2] = this->YDoubleRangeSlider.minimumValue();
    output[3] = this->YDoubleRangeSlider.maximumValue();

    output[4] = this->ZDoubleRangeSlider.minimumValue();
    output[5] = this->ZDoubleRangeSlider.maximumValue();
  }
  else
  {
    output[0] = this->X1SpinBox->value();
    output[1] = this->X2SpinBox->value();

    output[2] = this->Y1SpinBox->value();
    output[3] = this->Y2SpinBox->value();

    output[4] = this->Z1SpinBox->value();
    output[5] = this->Z2SpinBox->value();
  }
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::DesactivateSpinBox()
{
  this->XLabel->setDisabled(true);
  this->YLabel->setDisabled(true);
  this->ZLabel->setDisabled(true);

  this->X1SpinBox->setDisabled(true);
  this->X2SpinBox->setDisabled(true);

  this->Y1SpinBox->setDisabled(true);
  this->Y2SpinBox->setDisabled(true);

  this->Z1SpinBox->setDisabled(true);
  this->Z2SpinBox->setDisabled(true);

  this->XDoubleRangeSlider.setDisabled(true);
  this->YDoubleRangeSlider.setDisabled(true);
  this->ZDoubleRangeSlider.setDisabled(true);

  this->CropOutsideCheckBox->setDisabled(true);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onXSliderChanged(double vmin, double vmax)
{
  this->Internal->onXSliderChanged(vmin, vmax);
  this->Internal->updateRangeValues(true);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onYSliderChanged(double vmin, double vmax)
{
  this->Internal->onYSliderChanged(vmin, vmax);
  this->Internal->updateRangeValues(true);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onZSliderChanged(double vmin, double vmax)
{
  this->Internal->onZSliderChanged(vmin, vmax);
  this->Internal->updateRangeValues(true);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::onXSliderChanged(double vmin, double vmax)
{
  std::stringstream label;
  label << "[" << vmin << ";" << vmax << "]";
  this->XRangeLabel.setText(QString(label.str().c_str()));
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::onYSliderChanged(double vmin, double vmax)
{
  std::stringstream label;
  label << "[" << vmin << ";" << vmax << "]";
  this->YRangeLabel.setText(QString(label.str().c_str()));
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::pqInternal::onZSliderChanged(double vmin, double vmax)
{
  std::stringstream label;
  label << "[" << vmin << ";" << vmax << "]";
  this->ZRangeLabel.setText(QString(label.str().c_str()));
}

void vvCropReturnsDialog::pqInternal::updateRangeValues(bool isSliderMode)
{
  if (isSliderMode)
  {
    this->xRange[0] = this->XDoubleRangeSlider.minimumValue();
    this->xRange[1] = this->XDoubleRangeSlider.maximumValue();
    this->yRange[0] = this->YDoubleRangeSlider.minimumValue();
    this->yRange[1] = this->YDoubleRangeSlider.maximumValue();
    this->zRange[0] = this->ZDoubleRangeSlider.minimumValue();
    this->zRange[1] = this->ZDoubleRangeSlider.maximumValue();
  }
  else
  {
    this->xRange[0] = this->X1SpinBox->value();
    this->xRange[1] = this->X2SpinBox->value();
    this->yRange[0] = this->Y1SpinBox->value();
    this->yRange[1] = this->Y2SpinBox->value();
    this->zRange[0] = this->Z1SpinBox->value();
    this->zRange[1] = this->Z2SpinBox->value();
  }
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onSpinBoxChanged(double vtkNotUsed(value))
{
  this->Internal->updateRangeValues(false);
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::onCropGroupBoxToggled()
{
  this->Internal->X1SpinBox->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->X2SpinBox->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->Y1SpinBox->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->Y2SpinBox->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->Z1SpinBox->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->Z2SpinBox->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->XDoubleRangeSlider.setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->YDoubleRangeSlider.setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->ZDoubleRangeSlider.setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->noneRadioButton->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->cartesianRadioButton->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->sphericalRadioButton->setDisabled(!this->Internal->CropGroupBox->isChecked());
  this->Internal->CropOutsideCheckBox->setDisabled(!this->Internal->CropGroupBox->isChecked());
}

//-----------------------------------------------------------------------------
void vvCropReturnsDialog::UpdateDialogWithCurrentSetting()
{
  this->onCropGroupBoxToggled();
  if (this->Internal->noneRadioButton->isChecked())
  {
    this->onNoneToggled();
  }
}
