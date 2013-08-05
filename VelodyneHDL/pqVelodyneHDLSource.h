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
/*=========================================================================

   Program: ParaView
   Module:    pqVelodyneHDLSource.h

   Copyright (c) 2005-2008 Sandia Corporation, Kitware Inc.
   All rights reserved.

   ParaView is a free software; you can redistribute it and/or modify it
   under the terms of the ParaView license version 1.2. 

   See License_v1.2.txt for the full ParaView license.
   A copy of this license can be obtained by contacting
   Kitware Inc.
   28 Corporate Drive
   Clifton Park, NY 12065
   USA

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=========================================================================*/
#include "pqObjectPanel.h"

#include <pqView.h>
#include <pqPVApplicationCore.h>
#include <pqAnimationManager.h>
#include <pqAnimationScene.h>

#include <vtkSMSourceProxy.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMIntVectorProperty.h>

#include <QFileDialog>
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QSlider>
#include <QLabel>
#include <QTimer>

class pqVelodyneHDLSource : public pqObjectPanel
{

  Q_OBJECT

public:

  pqVelodyneHDLSource(pqProxy* proxy, QWidget* p) : pqObjectPanel(proxy, p)
  {
    this->LastTime = 0.0;

    QVBoxLayout* layout = new QVBoxLayout(this);

    this->Timer = new QTimer(this);
    this->Timer->setInterval(33);
    this->connect(this->Timer, SIGNAL(timeout()), SLOT(onPollSource()));


    QHBoxLayout* buttonLayout = new QHBoxLayout;
    this->PlayButton = new QPushButton("Play");
    this->StopButton = new QPushButton("Stop");
    this->connect(this->PlayButton, SIGNAL(clicked()), SLOT(onPlay()));
    this->connect(this->StopButton, SIGNAL(clicked()), SLOT(onStop()));
    buttonLayout->addWidget(this->PlayButton);
    buttonLayout->addWidget(this->StopButton);
    layout->addLayout(buttonLayout);

    this->FileNameLabel = new QLabel("Packet file: <none>");
    this->ChoosePacketFileButton = new QPushButton("Choose packet file");
    layout->addWidget(this->FileNameLabel);
    layout->addWidget(this->ChoosePacketFileButton);
    this->connect(this->ChoosePacketFileButton, SIGNAL(clicked()), SLOT(onChoosePacketFile()));


    this->CalibrationFileNameLabel = new QLabel("Calibration file: <none>");
    this->ChooseCalibrationFileButton = new QPushButton("Choose calibration file");
    layout->addWidget(this->CalibrationFileNameLabel);
    layout->addWidget(this->ChooseCalibrationFileButton);
    this->connect(this->ChooseCalibrationFileButton, SIGNAL(clicked()), SLOT(onChooseCalibrationFile()));


    this->OutputFileNameLabel = new QLabel("Output file: <none>");
    this->ChooseOutputFileButton = new QPushButton("Choose output file");
    layout->addWidget(this->OutputFileNameLabel);
    layout->addWidget(this->ChooseOutputFileButton);
    this->connect(this->ChooseOutputFileButton, SIGNAL(clicked()), SLOT(onChooseOutputFile()));



    /*
    layout->addWidget(new QLabel);

    QPushButton* refreshButton = new QPushButton("Refresh");
    this->connect(refreshButton, SIGNAL(clicked()), SLOT(onRefreshClicked()));
    layout->addWidget(refreshButton);

    QCheckBox* autoRefreshCheck = new QCheckBox("Auto refresh");
    this->connect(autoRefreshCheck, SIGNAL(toggled(bool)), SLOT(onAutoRefreshChecked(bool)));
    layout->addWidget(autoRefreshCheck);

    QSlider* slider = new QSlider(Qt::Horizontal);
    this->connect(slider, SIGNAL(valueChanged(int)), SLOT(onSliderValueChanged(int)));
    layout->addWidget(slider);
    this->AutoRefreshLabel = new QLabel();
    layout->addWidget(this->AutoRefreshLabel);

    layout->addStretch();

    slider->setMinimum(1);
    slider->setMaximum(200);
    slider->setValue(1);

    autoRefreshCheck->setChecked(true);
    */

    this->onAutoRefreshChecked(true);
  }

public slots:

  void onPlay()
  {
    this->setStreaming(true);
  }

  void onStop()
  {
    this->setStreaming(false);
  }

  void setStreaming(bool checked)
  {
    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
    if (!sourceProxy)
      {
      return;
      }

    if (checked)
      {
      sourceProxy->InvokeCommand("Start");
      }
    else
      {
      sourceProxy->InvokeCommand("Stop");
      }

    this->PlayButton->setEnabled(!checked);
    this->StopButton->setEnabled(checked);
    this->ChoosePacketFileButton->setEnabled(!checked);
    this->ChooseCalibrationFileButton->setEnabled(!checked);
    this->ChooseOutputFileButton->setEnabled(!checked);
  }

  void onAutoRefreshChecked(bool checked)
  {
    if (checked)
      {
      this->Timer->start();
      }
    else
      {
      this->Timer->stop();
      }
  }

  void onSliderValueChanged(int sliderValue)
  {
    int timeoutMax = 5000;
    int timeout = timeoutMax * sliderValue / 200.0;
    this->Timer->setInterval(timeout);
    this->AutoRefreshLabel->setText(QString("Auto refresh timeout: %0 s").arg(timeout/1000.0,  0, 'f', 2));
  }

  void onRefreshClicked()
  {
    this->onPollSource();
  }

  void onChoosePacketFile()
  {
    QString selectedFiler("*.pcap");
    QString fileName = QFileDialog::getOpenFileName(this, tr("Choose Packet File"),
                            QString(""),
                            tr("pcap (*.pcap)"), &selectedFiler);
    this->setFileName(fileName);
  }

  void onChooseCalibrationFile()
  {
    QString selectedFiler("*.xml");
    QString fileName = QFileDialog::getOpenFileName(this, tr("Choose Calibration File"),
                            QString(""),
                            tr("xml (*.xml)"), &selectedFiler);
    this->setCalibrationFileName(fileName);
  }

  void onChooseOutputFile()
  {
    QString selectedFiler("*.pcap");
    QString fileName = QFileDialog::getSaveFileName(this, tr("Choose Output File"),
                            QString(""),
                            tr("pcap (*.pcap)"), &selectedFiler);
    this->setOutputFileName(fileName);
  }

  void setFileName(QString fileName)
  {
    if (!QFileInfo(fileName).isFile())
      {
      return;
      }

    this->FileNameLabel->setText(QString("Packet file: %1").arg(QFileInfo(fileName).fileName()));

    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
    vtkSMPropertyHelper(sourceProxy, "PacketFile").Set(fileName.toAscii().data());
    sourceProxy->UpdateProperty("PacketFile");
  }

  void setCalibrationFileName(QString fileName)
  {
    if (!QFileInfo(fileName).isFile())
      {
      return;
      }

    this->CalibrationFileNameLabel->setText(QString("Calibration file: %1").arg(QFileInfo(fileName).fileName()));

    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
    vtkSMPropertyHelper(sourceProxy, "CorrectionsFile").Set(fileName.toAscii().data());
    sourceProxy->UpdateProperty("CorrectionsFile");
  }

  void setOutputFileName(QString fileName)
  {
    this->OutputFileNameLabel->setText(QString("Output file: %1").arg(QFileInfo(fileName).fileName()));

    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
    vtkSMPropertyHelper(sourceProxy, "OutputFile").Set(fileName.toAscii().data());
    sourceProxy->UpdateProperty("OutputFile");
  }

  void onPollSource()
  {
    vtkSMSourceProxy* sourceProxy = vtkSMSourceProxy::SafeDownCast(this->proxy());
    if (!sourceProxy)
      {
      return;
      }

    sourceProxy->InvokeCommand("Poll");
    sourceProxy->UpdatePipelineInformation();

    double lastTime = 0;
    int nElements = vtkSMPropertyHelper(sourceProxy, "TimestepValues").GetNumberOfElements();
    if (nElements)
      {
      lastTime = vtkSMPropertyHelper(sourceProxy, "TimestepValues").GetAsDouble(nElements-1);
      }

    bool latestTimestepChanged = (lastTime != this->LastTime);
    this->LastTime = lastTime;

    if (latestTimestepChanged && this->snapToLatestTimeStep())
      {
      pqPVApplicationCore::instance()->animationManager()->
        getActiveScene()->getProxy()->InvokeCommand("GoToLast");
      }
  }



  bool snapToLatestTimeStep()
  {
    return true;
  }

protected:

  QLabel* FileNameLabel;
  QLabel* CalibrationFileNameLabel;
  QLabel* OutputFileNameLabel;
  QLabel* AutoRefreshLabel;

  QPushButton* PlayButton;
  QPushButton* StopButton;
  QPushButton* ChoosePacketFileButton;
  QPushButton* ChooseCalibrationFileButton;
  QPushButton* ChooseOutputFileButton;
  QTimer* Timer;

  double LastTime;
};
