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
#include "vvCalibrationDialog.h"

#include "ui_vvCalibrationDialog.h"

#include <pqApplicationCore.h>
#include <pqSettings.h>

#include <QDialog>
#include <QFileDialog>

//-----------------------------------------------------------------------------
class vvCalibrationDialog::pqInternal : public Ui::vvCalibrationDialog
{
public:

};

//-----------------------------------------------------------------------------
vvCalibrationDialog::vvCalibrationDialog(QWidget *p) : QDialog(p)
{
  this->Internal = new pqInternal;
  this->Internal->setupUi(this);

  this->Internal->ListWidget->addItem("None");
  this->Internal->ListWidget->addItems(this->calibrationFiles());

  this->connect(this->Internal->ListWidget, SIGNAL(currentRowChanged(int)), SLOT(onCurrentRowChanged(int)));
  this->connect(this->Internal->AddButton, SIGNAL(clicked()), SLOT(addFile()));
  this->connect(this->Internal->RemoveButton, SIGNAL(clicked()), SLOT(removeSelectedFile()));

  this->restoreSelectedRow();

  pqSettings* settings = pqApplicationCore::instance()->settings();
  this->restoreGeometry(settings->value("VelodyneHDLPlugin/CalibrationFileDialog/Geometry").toByteArray());
}

//-----------------------------------------------------------------------------
vvCalibrationDialog::~vvCalibrationDialog()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  settings->setValue("VelodyneHDLPlugin/CalibrationFileDialog/Geometry", this->saveGeometry());
  delete this->Internal;
}

//-----------------------------------------------------------------------------
QStringList vvCalibrationDialog::calibrationFiles()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  return settings->value("VelodyneHDLPlugin/CalibrationFileDialog/Files").toStringList();
}

//-----------------------------------------------------------------------------
QString vvCalibrationDialog::selectedCalibrationFile()
{
  int row = this->Internal->ListWidget->currentRow();
  if (row > 0)
    {
    return this->Internal->ListWidget->item(row)->text();
    }
  return QString();
}

//-----------------------------------------------------------------------------
void vvCalibrationDialog::onCurrentRowChanged(int row)
{
  this->Internal->RemoveButton->setEnabled(row != 0);
  this->saveSelectedRow();
}

//-----------------------------------------------------------------------------
void vvCalibrationDialog::addFile()
{

  pqSettings* settings = pqApplicationCore::instance()->settings();
  QString defaultDir = settings->value("VelodyneHDLPlugin/OpenData/DefaultDir", QDir::homePath()).toString();


  QString selectedFiler("*.xml");
  QString fileName = QFileDialog::getOpenFileName(this, tr("Choose Calibration File"),
                          defaultDir,
                          tr("xml (*.xml)"), &selectedFiler);

  if (fileName.isEmpty())
    {
    return;
    }

  this->Internal->ListWidget->addItem(fileName);
  this->Internal->ListWidget->setCurrentRow(this->Internal->ListWidget->count()-1);
  this->saveFileList();

  settings->setValue("VelodyneHDLPlugin/OpenData/DefaultDir", QFileInfo(fileName).absoluteDir().absolutePath());
}

//-----------------------------------------------------------------------------
void vvCalibrationDialog::removeSelectedFile()
{
  int row = this->Internal->ListWidget->currentRow();
  if (row >= 0)
    {
    delete this->Internal->ListWidget->takeItem(row);
    this->saveFileList();
    }
}

//-----------------------------------------------------------------------------
void vvCalibrationDialog::saveFileList()
{
  QStringList files;
  for (int i = 1; i < this->Internal->ListWidget->count(); ++i)
    {
    files << this->Internal->ListWidget->item(i)->text();
    }

  pqSettings* settings = pqApplicationCore::instance()->settings();
  settings->setValue("VelodyneHDLPlugin/CalibrationFileDialog/Files", files);
}

//-----------------------------------------------------------------------------
void vvCalibrationDialog::saveSelectedRow()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  settings->setValue("VelodyneHDLPlugin/CalibrationFileDialog/CurrentRow", this->Internal->ListWidget->currentRow());
}

//-----------------------------------------------------------------------------
void vvCalibrationDialog::restoreSelectedRow()
{
  pqSettings* settings = pqApplicationCore::instance()->settings();
  int row = settings->value("VelodyneHDLPlugin/CalibrationFileDialog/CurrentRow").toInt();
  this->Internal->ListWidget->setCurrentRow(row);
  this->onCurrentRowChanged(row);
}
