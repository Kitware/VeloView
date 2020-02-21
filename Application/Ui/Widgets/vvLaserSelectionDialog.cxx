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
#include "vvLaserSelectionDialog.h"

#include "ui_vvLaserSelectionDialog.h"

#include <pqApplicationCore.h>
#include <pqSettings.h>

#include <QCheckBox>

#include <iostream>

#include <cassert>
#include <cmath>

#define NUM_LASER_MAX 128

//-----------------------------------------------------------------------------
class vvLaserSelectionDialog::pqInternal : public Ui::vvLaserSelectionDialog
{
public:
  pqInternal(QDialog *external)
    : Settings(pqApplicationCore::instance()->settings())
  {
    this->setupUi(external);
    this->CancelButton = new QPushButton("Cancel");
    this->CancelButton->setIcon(external->style()->standardIcon(QStyle::SP_DialogCancelButton));

    this->ApplyButton = new QPushButton("Apply");
    this->ApplyButton->setIcon(external->style()->standardIcon(QStyle::SP_DialogOkButton));

    this->ApplyAndSaveSessionButton = new QPushButton("Apply && save until exit");
    this->ApplyAndSaveSessionButton->setIcon(external->style()->standardIcon(QStyle::SP_DialogSaveButton));

    this->ApplyAndSavePermanentButton = new QPushButton("Apply && save permanently");
    this->ApplyAndSavePermanentButton->setIcon(external->style()->standardIcon(QStyle::SP_DialogSaveButton));

    this->CancelButton->setToolTip(
      "Do not apply this selection and do not save these settings.");
    this->ApplyButton->setToolTip(
      "Apply this selection to current PCAP/Stream only.");
    this->ApplyAndSaveSessionButton->setToolTip(
      "Apply this selection to current and future PCAPs/Streams with same"
      " channel count until " SOFTWARE_NAME " is closed.");
    this->ApplyAndSavePermanentButton->setToolTip(
      "Apply this selection to current and future PCAPs/Streams with same"
      " channel count.");

    this->buttonBox->addButton(this->CancelButton, QDialogButtonBox::ActionRole);
    this->buttonBox->addButton(this->ApplyButton, QDialogButtonBox::ActionRole);
    this->buttonBox->addButton(this->ApplyAndSaveSessionButton, QDialogButtonBox::ActionRole);
    this->buttonBox->addButton(this->ApplyAndSavePermanentButton, QDialogButtonBox::ActionRole);
  }

  void setup();
  void saveSettings();
  void restoreSettings();

  QPushButton *CancelButton;
  QPushButton *ApplyButton;
  QPushButton *ApplyAndSaveSessionButton;
  QPushButton *ApplyAndSavePermanentButton;

  pqSettings* const Settings;

public:
  QTableWidget* Table;

  // The actual number of rows to be displayed.
  // This number is equal to the number of channels.
  int numVisibleRows;

  // Store the current sorting column id and order. This way, we're
  // able to restore it when the calibration file or the opened file changes
  // or when the application is restarted.
  int sortingColumnId;
  Qt::SortOrder sortingColumnOrder;
};

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::pqInternal::saveSettings()
{
  this->sortingColumnId =
    this->Table->horizontalHeader()->sortIndicatorSection();
  this->Settings->setValue(
    QString("LidarPlugin/LaserSelectionDialogSortingColumnId"), this->sortingColumnId);
  this->sortingColumnOrder =
    this->Table->horizontalHeader()->sortIndicatorOrder();
  this->Settings->setValue(
    QString("LidarPlugin/LaserSelectionDialogSortingColumnOrder"), this->sortingColumnOrder);
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::saveSettings()
{
  this->Internal->saveSettings();
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::pqInternal::restoreSettings()
{
  int sortingColumnId =
    this->Settings->value(QString("LidarPlugin/LaserSelectionDialogSortingColumnId")).toInt();
  Qt::SortOrder sortingColumnOrder = Qt::SortOrder(
    this->Settings->value(QString("LidarPlugin/LaserSelectionDialogSortingColumnOrder"))
      .toInt());

  if (sortingColumnId > 0)
  {
    this->sortingColumnId = sortingColumnId;
    this->sortingColumnOrder = sortingColumnOrder;
  }
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::pqInternal::setup()
{
  QTableWidget* table = this->LaserTable;

  table->setColumnWidth(0, 32);
  table->setColumnWidth(1, 64);

  for (int i = 2; i < 13; i++)
  {
    table->setColumnWidth(i, 128);
  }

  // Set checkable header
  QTableWidgetItem* hcheckbox = new QTableWidgetItem();
  hcheckbox->setCheckState(Qt::Checked);
  hcheckbox->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);

  table->setHorizontalHeaderItem(0, hcheckbox);

  numVisibleRows = NUM_LASER_MAX;

  for (size_t i = 0; i < NUM_LASER_MAX; ++i)
  {
    table->insertRow(i);

    QTableWidgetItem* checkbox = new QTableWidgetItem();
    checkbox->setCheckState(Qt::Checked);
    checkbox->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    checkbox->setTextAlignment(Qt::AlignHCenter);

    for (int j = 2; j < 13; j++)
    {
      QTableWidgetItem* values = new QTableWidgetItem;
      values->setData(Qt::EditRole, 0.0);
      values->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

      table->setItem(i, j, values);
    }

    QTableWidgetItem* channel = new QTableWidgetItem;
    channel->setData(Qt::EditRole, QVariant::fromValue(i));
    channel->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    table->setItem(i, 0, checkbox);

    table->setItem(i, 1, channel);
  }

  QTableWidgetItem* leadBox = table->horizontalHeaderItem(0);
  leadBox->setFlags(Qt::ItemIsUserCheckable);

  this->EnableDisableAll->setTristate(false);

  this->Table = table;

  for (int i = 3; i < 13; i++)
  {
    this->Table->setColumnHidden(i, true);
  }
  this->Table->horizontalHeader()->setStretchLastSection(false);
  this->Table->resizeColumnsToContents();

  // Default sort order set to vertical correction
  this->sortingColumnId = 2;
  this->sortingColumnOrder = Qt::AscendingOrder;
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onToggleSelected()
{
  // Update other selected items
  QList<QTableWidgetItem*> list = this->Internal->Table->selectedItems();
  foreach (QTableWidgetItem* sitem, list)
  {
    if (sitem->column() == 0)
    {
      sitem->setCheckState(sitem->checkState() == Qt::Checked ? Qt::Unchecked : Qt::Checked);
    }
  }
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onItemChanged(QTableWidgetItem* vtkNotUsed(item))
{
  // Set enable/disable all based on all checked
  bool allChecked = true;
  bool noneChecked = true;
  // Iterate over visible rows to choose in which state the enable/disable
  // all checkbox should pass

  // Store the current sorting state
  int currentSortingColumn = this->Internal->Table->horizontalHeader()->sortIndicatorSection();
  Qt::SortOrder currentSortOrder = this->Internal->Table->horizontalHeader()->sortIndicatorOrder();

  // Sort the table by channel
  this->Internal->Table->sortItems(1);
  for (int i = 0; i < this->Internal->numVisibleRows; ++i)
  {
    // Check the current row state
    QTableWidgetItem* item = this->Internal->Table->item(i, 0);
    allChecked = allChecked && item->checkState() == Qt::Checked;
    noneChecked = noneChecked && item->checkState() == Qt::Unchecked;
  }
  // Restore the current sorting state
  this->Internal->Table->sortItems(currentSortingColumn, currentSortOrder);
  Qt::CheckState state;
  if (allChecked)
  {
    state = Qt::Checked;
  }
  else if (noneChecked)
  {
    state = Qt::Unchecked;
  }
  else
  {
    state = Qt::PartiallyChecked;
  }
  this->Internal->EnableDisableAll->setCheckState(state);
  if (state != Qt::PartiallyChecked)
  {
    this->Internal->EnableDisableAll->setTristate(false);
  }
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onEnableDisableAll(int state)
{
  if (state != Qt::PartiallyChecked)
  {
    // enable all
    // Store current sorting state
    int currentSortingColumn = this->Internal->Table->horizontalHeader()->sortIndicatorSection();
    Qt::SortOrder currentSortOrder =
      this->Internal->Table->horizontalHeader()->sortIndicatorOrder();

    // Sort the table by channel
    this->Internal->Table->sortItems(1);
    for (int i = 0; i < this->Internal->numVisibleRows; ++i)
    {
      // Enable/Disable the checkboxes
      QTableWidgetItem* item = this->Internal->Table->item(i, 0);
      item->setCheckState(Qt::CheckState(state));
    }
    // Restore the current sorting state
    this->Internal->Table->sortItems(currentSortingColumn, currentSortOrder);
  }
}

//-----------------------------------------------------------------------------
vvLaserSelectionDialog::vvLaserSelectionDialog(QWidget* p)
  : QDialog(p)
{
  this->Internal = new pqInternal(this);
  this->Internal->setup();
  this->Internal->restoreSettings();

  QObject::connect(this->Internal->Table, SIGNAL(itemChanged(QTableWidgetItem*)), this,
    SLOT(onItemChanged(QTableWidgetItem*)));

  QObject::connect(this->Internal->Toggle, SIGNAL(clicked()), this, SLOT(onToggleSelected()));

  QObject::connect(this->Internal->EnableDisableAll, SIGNAL(stateChanged(int)), this,
    SLOT(onEnableDisableAll(int)));

  QObject::connect(this->Internal->DisplayMoreCorrections, SIGNAL(toggled(bool)), this,
    SLOT(onDisplayMoreCorrectionsChanged()));

  QObject::connect(this, SIGNAL(accepted()), this, SLOT(saveSettings()));
  QObject::connect(this, SIGNAL(rejected()), this, SLOT(saveSettings()));

  connect(this->Internal->CancelButton, SIGNAL(clicked()), this, SLOT(saveSettings()));
  connect(this->Internal->ApplyButton, SIGNAL(clicked()), this, SLOT(saveSettings()));
  connect(this->Internal->ApplyAndSaveSessionButton, SIGNAL(clicked()), this, SLOT(saveSettings()));
  connect(this->Internal->ApplyAndSavePermanentButton, SIGNAL(clicked()), this, SLOT(saveSettings()));

  connect(this->Internal->CancelButton, SIGNAL(clicked()), this, SLOT(onCancel()));
  connect(this->Internal->ApplyButton, SIGNAL(clicked()), this, SLOT(onApply()));
  connect(this->Internal->ApplyAndSaveSessionButton, SIGNAL(clicked()), this, SLOT(onApplyAndSaveSession()));
  connect(this->Internal->ApplyAndSavePermanentButton, SIGNAL(clicked()), this, SLOT(onApplyAndSavePermanent()));

  this->Internal->Table->setSortingEnabled(true);
}

//-----------------------------------------------------------------------------
QVector<int> vvLaserSelectionDialog::getLaserSelectionSelector()
{
  QVector<int> result(NUM_LASER_MAX, 1);
  for (int i = 0; i < this->Internal->Table->rowCount(); ++i)
  {
    QTableWidgetItem* value = this->Internal->Table->item(i, 1);
    int channel = value->data(Qt::EditRole).toInt();
    assert(channel < NUM_LASER_MAX && channel >= 0);
    QTableWidgetItem* item = this->Internal->Table->item(i, 0);
    result[channel] = (item->checkState() == Qt::Checked) && !this->Internal->Table->isRowHidden(i);
  }
  return result;
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::setLasersCorrections(const QVector<double>& verticalCorrection,
  const QVector<double>& rotationalCorrection, const QVector<double>& distanceCorrection,
  const QVector<double>& distanceCorrectionX, const QVector<double>& distanceCorrectionY,
  const QVector<double>& verticalOffsetCorrection,
  const QVector<double>& horizontalOffsetCorrection, const QVector<double>& focalDistance,
  const QVector<double>& focalSlope, const QVector<double>& minIntensity,
  const QVector<double>& maxIntensity, int nchannels)
{

  for (int i = 0; i < this->Internal->Table->rowCount(); ++i)
  {
    QTableWidgetItem* value = this->Internal->Table->item(i, 1);
    int channel = value->data(Qt::EditRole).toInt();

    assert(channel < NUM_LASER_MAX && channel >= 0);
    int col = 2;
    QTableWidgetItem* item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, verticalCorrection[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, rotationalCorrection[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, distanceCorrection[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, distanceCorrectionX[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, distanceCorrectionY[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, verticalOffsetCorrection[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, horizontalOffsetCorrection[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, focalDistance[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, focalSlope[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, minIntensity[channel]);

    item = this->Internal->Table->item(i, col++);
    item->setData(Qt::EditRole, maxIntensity[channel]);
  }

  // Display the number of rows according to the number of channels
  if (nchannels != this->Internal->numVisibleRows)
  {
    this->Internal->numVisibleRows = nchannels;

    // Sort the table by channel. It's important for hiding the right rows later
    this->Internal->Table->sortItems(1);

    for (int i = 0; i < this->Internal->Table->rowCount(); ++i)
    {
      // Display the number of rows according to the number of channels
      if (i < this->Internal->numVisibleRows)
      {
        this->Internal->Table->showRow(i);
      }
      // Hide the remaining rows
      else
      {
        this->Internal->Table->hideRow(i);
      }
    }
  }
  // Sort the table by prefered sort order, or by vertical correction by default
  this->Internal->Table->sortItems(
    this->Internal->sortingColumnId, this->Internal->sortingColumnOrder);
  this->Internal->Table->resizeColumnsToContents();
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::setLaserSelectionSelector(const QVector<int>& mask)
{
  for (int i = 0; i < this->Internal->Table->rowCount(); ++i)
  {
    QTableWidgetItem* item = this->Internal->Table->item(i, 0);
    QTableWidgetItem* value = this->Internal->Table->item(i, 1);
    int channel = value->data(Qt::EditRole).toInt();
    assert(channel < NUM_LASER_MAX && channel >= 0);
    item->setCheckState(
      mask[channel] && !this->Internal->Table->isRowHidden(i) ? Qt::Checked : Qt::Unchecked);
  }
}

//-----------------------------------------------------------------------------
vvLaserSelectionDialog::~vvLaserSelectionDialog()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onDisplayMoreCorrectionsChanged()
{
  for (int i = 3; i < 13; i++)
  {
    this->Internal->Table->setColumnHidden(i, !this->Internal->DisplayMoreCorrections->isChecked());
  }
  this->Internal->Table->resizeColumnsToContents();
}

//-----------------------------------------------------------------------------
bool vvLaserSelectionDialog::isDisplayMoreSelectionsChecked()
{
  return this->Internal->DisplayMoreCorrections->isChecked();
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::setDisplayMoreSelectionsChecked(bool state)
{
  this->Internal->DisplayMoreCorrections->setChecked(state);
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onCancel()
{
  this->applyOrder_ = 0;
  QDialog::accept();
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onApply()
{
  this->applyOrder_ = 1;
  QDialog::accept();
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onApplyAndSaveSession()
{
  this->applyOrder_ = 2;
  QDialog::accept();
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onApplyAndSavePermanent()
{
  this->applyOrder_ = 3;
  QDialog::accept();
}

//-----------------------------------------------------------------------------
int vvLaserSelectionDialog::applyOrder()
{
  return this->applyOrder_;
}
