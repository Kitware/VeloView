/*=========================================================================

   Program: ParaView
   Module:    pqVCRToolbar.cxx

   Copyright (c) 2005,2006 Sandia Corporation, Kitware Inc.
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

========================================================================*/
#include "vvPlayerControlsToolbar.h"

#include "ui_vvPlayerControlsToolbar.h"

#include <limits>

#include <QLabel>
#include <QComboBox>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QList>
#include <QPair>
#include <QString>

#include "pqActiveObjects.h"
#include "pqPVApplicationCore.h"
#include "pqUndoStack.h"
#include "pqAnimationManager.h"
#include <pqAnimationTimeWidget.h>

#include "vvPlayerControlsController.h"
#include "pqAnimationScene.h"
#include <vtkSMProxy.h>
#include <vtkSMProperty.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMTimeKeeperProxy.h>
#include <pqPropertyLinks.h>

class vvPlayerControlsToolbar::pqInternals : public Ui::vvPlayerControlsToolbar
{
public:
  pqPropertyLinks Links;
  QList<QPair<double, QString> > speedFactor;
  QComboBox* speedComboBox;
  QSlider* frameSlider;
  QDoubleSpinBox* timeSpinBox;
  QSpinBox* frameQSpinBox;
  QLabel* frameLabel;
  bool isPlaying;
  bool ContinuePlaying;
};

namespace
{
  /// Used to link the number of elements in a sm-property to the qt widget.
  class vvPlayerControlsToolbarLinks : public pqPropertyLinksConnection
  {
  typedef pqPropertyLinksConnection Superclass;
public:
  vvPlayerControlsToolbarLinks(
    QObject* qobject, const char* qproperty, const char* qsignal,
    vtkSMProxy* smproxy, vtkSMProperty* smproperty, int smindex,
    bool use_unchecked_modified_event,
    QObject* parentObject=0)
    : Superclass(qobject, qproperty, qsignal,
      smproxy, smproperty, smindex,
      use_unchecked_modified_event, parentObject)
    {
    }
  virtual ~vvPlayerControlsToolbarLinks()
    {
    }

protected:
  virtual QVariant currentServerManagerValue(bool use_unchecked) const
    {
    Q_ASSERT(use_unchecked == false);
    Q_UNUSED(use_unchecked);
    unsigned int count = vtkSMPropertyHelper(this->propertySM()).GetNumberOfElements();
    return QVariant(static_cast<int>(count));
    }

private:
  Q_DISABLE_COPY(vvPlayerControlsToolbarLinks);
  };
}

//-----------------------------------------------------------------------------
vvPlayerControlsToolbar::vvPlayerControlsToolbar(QWidget* parentObject)
  : QToolBar(parentObject)
{
  this->UI = new pqInternals();
  Ui::vvPlayerControlsToolbar &ui = *this->UI;
  ui.setupUi(this);

  vvPlayerControlsController* controller = new vvPlayerControlsController(this);
  this->Controller = controller;

  this->connect(pqPVApplicationCore::instance()->animationManager(),
    SIGNAL(activeSceneChanged(pqAnimationScene*)),
    this, SLOT(setAnimationScene(pqAnimationScene*)));

  this->connect(pqPVApplicationCore::instance()->animationManager(),
    SIGNAL(activeSceneChanged(pqAnimationScene*)),
    SLOT(setAnimationScene(pqAnimationScene*)));

  //------------------------//
  // Add the speed control
  //------------------------//
  // create the speed factor that will be display to the user
  // we need to keep an order to display so we can't use hash table or set
  // however the developper is reponsable for for the uniqueness of each element
  this->UI->speedFactor.append(qMakePair(0.,   QString("All frames")));
  this->UI->speedFactor.append(qMakePair(0.1, QString("x 0.1")));
  this->UI->speedFactor.append(qMakePair(0.25, QString("x 0.25")));
  this->UI->speedFactor.append(qMakePair(0.5,  QString("x 0.5")));
  this->UI->speedFactor.append(qMakePair(1.,   QString("x 1"))) ;
  this->UI->speedFactor.append(qMakePair(2.,   QString("x 2")));
  this->UI->speedFactor.append(qMakePair(3.,   QString("x 3")));
  this->UI->speedFactor.append(qMakePair(5.,   QString("x 5")));
  this->UI->speedFactor.append(qMakePair(10.,  QString("x 10")));
  this->UI->speedFactor.append(qMakePair(20.,  QString("x 20")));
  this->UI->speedFactor.append(qMakePair(100., QString("x100")));

  // add the widget to the toolbar
  this->addWidget(new QLabel("Speed:", this));
  this->UI->speedComboBox = new QComboBox(this);
  for (int i = 0; i < this->UI->speedFactor.size(); ++i)
  {
    this->UI->speedComboBox->addItem(this->UI->speedFactor[i].second);
    if (this->UI->speedFactor[i].first == 1)
    {
      this->UI->speedComboBox->setCurrentIndex(i);
    }
  }
  this->addWidget(this->UI->speedComboBox);

  // create all connection
  QObject::connect(this->UI->speedComboBox, SIGNAL(currentIndexChanged(int)),
    this, SLOT(onSpeedChanged()));
  QObject::connect(this, SIGNAL(speedChange(double)),
    controller, SLOT(onSpeedChange(double)));

  // add a separator to visualy group the element together
  this->addSeparator();

  //------------------------//
  // Add the Slider
  //------------------------//
  this->UI->frameSlider = new QSlider(Qt::Horizontal, this);
  this->addWidget(this->UI->frameSlider);

  // create connection
  this->connect(this->UI->frameSlider, SIGNAL(sliderPressed()),
    this, SLOT(PressSlider()));
  this->connect(this->UI->frameSlider, SIGNAL(sliderReleased()),
    this, SLOT(ReleaseSlider()));
  this->connect(this->UI->frameSlider, SIGNAL(valueChanged(int)),
    this, SLOT(setTimeStep(int)));

  //------------------------//
  // Add the Time DoubleSpinBox
  //------------------------//
  this->addWidget(new QLabel("Time", this));
  this->UI->timeSpinBox = new QDoubleSpinBox(this);
  this->addWidget(this->UI->timeSpinBox);

    // create connection
  this->connect(this->UI->timeSpinBox, SIGNAL(valueChanged(double)),
    this, SLOT(setTimeValue(double)));

  //------------------------//
  // Add the Frame SpinBox
  //------------------------//
  this->addWidget(new QLabel("Frame", this));
  this->UI->frameQSpinBox = new QSpinBox(this);
  this->addWidget(this->UI->frameQSpinBox);
  this->UI->frameLabel = new QLabel();
  this->addWidget(this->UI->frameLabel);

  // create connection
  this->connect(this->UI->frameQSpinBox, SIGNAL(valueChanged(int)),
    this, SLOT(setTimeStep(int)));

  // Ideally pqVCRController needs to be deprecated in lieu of a more
  // action-reaction friendly implementation. But for now, I am simply reusing
  // the old code.
  QObject::connect(ui.actionPlay, SIGNAL(triggered()),
    controller, SLOT(onPlay()));
  QObject::connect(ui.actionFirstFrame, SIGNAL(triggered()),
    controller, SLOT(onFirstFrame()));
  QObject::connect(ui.actionPreviousFrame, SIGNAL(triggered()),
    controller, SLOT(onPreviousFrame()));
  QObject::connect(ui.actionNextFrame, SIGNAL(triggered()),
   controller, SLOT(onNextFrame()));
  QObject::connect(ui.actionLastFrame, SIGNAL(triggered()),
    controller, SLOT(onLastFrame()));
  QObject::connect(ui.actionLoop, SIGNAL(toggled(bool)),
    controller, SLOT(onLoop(bool)));

  QObject::connect(controller, SIGNAL(enabled(bool)),
    ui.actionPlay, SLOT(setEnabled(bool)));
  QObject::connect(controller, SIGNAL(enabled(bool)),
    ui.actionFirstFrame, SLOT(setEnabled(bool)));
  QObject::connect(controller, SIGNAL(enabled(bool)),
    ui.actionPreviousFrame, SLOT(setEnabled(bool)));
  QObject::connect(controller, SIGNAL(enabled(bool)),
    ui.actionNextFrame, SLOT(setEnabled(bool)));
  QObject::connect(controller, SIGNAL(enabled(bool)),
    ui.actionLastFrame, SLOT(setEnabled(bool)));
  QObject::connect(controller, SIGNAL(enabled(bool)),
    ui.actionLoop, SLOT(setEnabled(bool)));
  QObject::connect(controller, SIGNAL(loop(bool)),
    ui.actionLoop, SLOT(setChecked(bool)));
  QObject::connect(controller, SIGNAL(playing(bool)),
    this, SLOT(onPlaying(bool)));
}

//-----------------------------------------------------------------------------
vvPlayerControlsToolbar::~vvPlayerControlsToolbar()
{
  delete this->UI;
  this->UI = 0;
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::onPlaying(bool playing)
{
  this->UI->isPlaying = playing;
  if(playing)
    {
    disconnect(this->UI->actionPlay, SIGNAL(triggered()),
      this->Controller, SLOT(onPlay()));
    connect(this->UI->actionPlay, SIGNAL(triggered()),
      this->Controller, SLOT(onPause()));
    this->UI->actionPlay->setIcon(
      QIcon(":/vvResources/Icons/media-playback-pause.png"));
    this->UI->actionPlay->setText("Pa&use");
    }
  else
    {
    connect(this->UI->actionPlay, SIGNAL(triggered()),
      this->Controller, SLOT(onPlay()));
    disconnect(this->UI->actionPlay, SIGNAL(triggered()),
      this->Controller, SLOT(onPause()));
    this->UI->actionPlay->setIcon(
      QIcon(":/vvResources/Icons/media-playback-start.png"));
    this->UI->actionPlay->setText("&Play");
    }

  // this becomes a behavior.
  // this->Implementation->Core->setSelectiveEn abledState(!playing);
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::onSpeedChanged()
{
  for (int i = 0; i < this->UI->speedFactor.size(); ++i)
  {
    if (this->UI->speedFactor[i].second == this->UI->speedComboBox->currentText())
    {
      emit speedChange(this->UI->speedFactor[i].first);
      return;
    }
  }
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::setAnimationScene(pqAnimationScene* scene)
{
  this->UI->Links.clear();
  this->Controller->setAnimationScene(scene);

  this->UI->Links.addPropertyLink<vvPlayerControlsToolbarLinks>(
    this, "timeStepCount", SIGNAL(dummySignal()),
    this->timeKeeper(), this->timeKeeper()->GetProperty("TimestepValues"));
  this->UI->Links.addPropertyLink(
    this, "timeValue", SIGNAL(timeValueChanged()),
    scene->getProxy(), scene->getProxy()->GetProperty("AnimationTime"));
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::setTimeValue(double value)
{
  this->UI->timeSpinBox->blockSignals(true);
  this->UI->timeSpinBox->setValue(value);
  this->UI->timeSpinBox->blockSignals(false);

  int index = vtkSMTimeKeeperProxy::GetLowerBoundTimeStepIndex(this->timeKeeper(), value);

  this->UI->frameQSpinBox->blockSignals(true);
  this->UI->frameQSpinBox->setValue(index);
  this->UI->frameQSpinBox->blockSignals(false);

  this->UI->frameSlider->blockSignals(true);
  this->UI->frameSlider->setValue(index);
  this->UI->frameSlider->blockSignals(false);

  emit timeValueChanged();
}

//-----------------------------------------------------------------------------
double vvPlayerControlsToolbar::timeValue() const
{
  return this->UI->timeSpinBox->value();
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::setTimeStepCount(int value)
{
  this->setTimeValue(0);
  this->UI->frameQSpinBox->setMaximum(value > 0? value -1 : 0);
  this->UI->frameSlider->setMaximum(value > 0? value -1 : 0);
  this->UI->frameLabel->setText(QString("of %1").arg(value));
  this->UI->actionFirstFrame->setToolTip(
    QString("First Frame (%1)").arg(0));
  this->UI->actionLastFrame->setToolTip(
    QString("Last Frame (%1)").arg(value));

  double time = vtkSMTimeKeeperProxy::GetLowerBoundTimeStep(this->timeKeeper(), std::numeric_limits<double>::max());
  this->UI->timeSpinBox->setMaximum(time);
}

//-----------------------------------------------------------------------------
int vvPlayerControlsToolbar::timeStepCount() const
{
  return this->UI->frameQSpinBox->maximum();
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::PressSlider()
{
  if (this->UI->isPlaying)
  {
    this->UI->ContinuePlaying = true;
    this->Controller->getAnimationScene()->pause();
  }
  else
  {
    this->UI->ContinuePlaying = false;
  }
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::ReleaseSlider()
{
  if (this->UI->ContinuePlaying)
  {
    this->Controller->getAnimationScene()->play();
  }
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::setTimeStep(int value)
{
  double time = this->Controller->getAnimationScene()->getTimeSteps()[value];
  this->Controller->getAnimationScene()->setAnimationTime(time);
}

//-----------------------------------------------------------------------------
vtkSMProxy* vvPlayerControlsToolbar::timeKeeper() const
{
  return vtkSMPropertyHelper(this->Controller->getAnimationScene()->getProxy(), "TimeKeeper").GetAsProxy();
}

