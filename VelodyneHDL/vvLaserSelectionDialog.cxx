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

//-----------------------------------------------------------------------------
class vvLaserSelectionDialog::pqInternal : public Ui::vvLaserSelectionDialog
{
public:
  void setup()
  {
    QTableWidget* table = this->LaserTable;
    QAbstractItemModel* model = table->model();

    table->setColumnWidth(0, 32);
    table->setColumnWidth(1, 128);
    table->setColumnWidth(3, 128);

    // Set checkable header
    QTableWidgetItem* hcheckbox = new QTableWidgetItem();
    hcheckbox->setCheckState(Qt::Checked);
    hcheckbox->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);

    table->setHorizontalHeaderItem(0, hcheckbox);

    for(size_t i = 0; i < 32; ++i)
       {
       table->insertRow(i);

       QTableWidgetItem* checkbox = new QTableWidgetItem();
       checkbox->setCheckState(Qt::Checked);
       checkbox->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
       checkbox->setTextAlignment(Qt::AlignHCenter);

       QTableWidgetItem* verticalAngle = new QTableWidgetItem;
       verticalAngle->setData(Qt::EditRole, 0.0);
       verticalAngle->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

       QTableWidgetItem* channel = new QTableWidgetItem;
       channel->setData(Qt::EditRole, QVariant::fromValue(i));
       channel->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

       table->setItem(i, 0, checkbox);
       table->setItem(i, 1, verticalAngle);
       table->setItem(i, 2, channel);
       }

    QTableWidgetItem* leadBox = table->horizontalHeaderItem(0);
    leadBox->setFlags(Qt::ItemIsUserCheckable);

    this->EnableDisableAll->setTristate(false);

    this->Table = table;
  }

public:

  QTableWidget* Table;
};

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onToggleSelected()
{
  // Update other selected items
  QList<QTableWidgetItem*> list = this->Internal->Table->selectedItems();
  foreach(QTableWidgetItem* sitem, list)
    {
    if(sitem->column() == 0)
      {
      sitem->setCheckState(sitem->checkState() == Qt::Checked ? Qt::Unchecked : Qt::Checked);
      }
    }
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onItemChanged(QTableWidgetItem* item)
{
  // Set enable/disable all based on all checked
  bool allChecked = true;
  bool noneChecked = true;
  //
  for(int i = 0; i < this->Internal->Table->rowCount(); ++i)
    {
    QTableWidgetItem* item = this->Internal->Table->item(i, 0);
    allChecked = allChecked && item->checkState() == Qt::Checked;
    noneChecked = noneChecked && item->checkState() == Qt::Unchecked;
    }
  Qt::CheckState state;
  if(allChecked)
    {
    state = Qt::Checked;
    }
  else if(noneChecked)
    {
    state = Qt::Unchecked;
    }
  else
    {
    state = Qt::PartiallyChecked;
    }
  this->Internal->EnableDisableAll->setCheckState(state);
  if(state != Qt::PartiallyChecked)
    {
    this->Internal->EnableDisableAll->setTristate(false);
    }
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::onEnableDisableAll(int state)
{
  if(state != Qt::PartiallyChecked)
    {
    // enable all
    for(int i = 0; i < this->Internal->Table->rowCount(); ++i)
      {
      QTableWidgetItem* item = this->Internal->Table->item(i, 0);
      item->setCheckState(Qt::CheckState(state));
      }
    }
}

//-----------------------------------------------------------------------------
vvLaserSelectionDialog::vvLaserSelectionDialog(QWidget *p) : QDialog(p)
{
  this->Internal = new pqInternal();
  this->Internal->setupUi(this);

  this->Internal->setup();
  QObject::connect(this->Internal->Table, SIGNAL(itemChanged(QTableWidgetItem*)),
                   this, SLOT(onItemChanged(QTableWidgetItem*)));

  QObject::connect(this->Internal->Toggle, SIGNAL(clicked()),
                   this, SLOT(onToggleSelected()));

  QObject::connect(this->Internal->EnableDisableAll, SIGNAL(stateChanged(int)),
                   this, SLOT(onEnableDisableAll(int)));

}

//-----------------------------------------------------------------------------
QVector<int> vvLaserSelectionDialog::getLaserSelectionMask()
{
  QVector<int> result(64, 1);
  for(int i = 0; i < this->Internal->Table->rowCount(); ++i)
    {
    QTableWidgetItem* item = this->Internal->Table->item(i, 0);
    result[i] = (item->checkState() == Qt::Checked);
    }
  return result;
}

//-----------------------------------------------------------------------------
void vvLaserSelectionDialog::setLaserSelectionMask(const QVector<int>& mask)
{
  for(int i = 0; i < this->Internal->Table->rowCount(); ++i)
    {
    QTableWidgetItem* item = this->Internal->Table->item(i, 0);
    item->setCheckState(mask[i] ? Qt::Checked : Qt::Unchecked);
    }
}

//-----------------------------------------------------------------------------
vvLaserSelectionDialog::~vvLaserSelectionDialog()
{
  delete this->Internal;
}
