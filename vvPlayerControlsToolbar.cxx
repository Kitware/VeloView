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

#include "pqActiveObjects.h"
#include "pqPVApplicationCore.h"
#include "pqUndoStack.h"
#include "pqAnimationManager.h"

#include "vvPlayerControlsController.h"

class vvPlayerControlsToolbar::pqInternals : public Ui::vvPlayerControlsToolbar
{
};

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::constructor()
{
  this->UI = new pqInternals();
  Ui::vvPlayerControlsToolbar &ui = *this->UI;
  ui.setupUi(this);

  vvPlayerControlsController* controller = new vvPlayerControlsController(this);
  this->Controller = controller;
  QObject::connect(pqPVApplicationCore::instance()->animationManager(),
    SIGNAL(activeSceneChanged(pqAnimationScene*)),
    controller, SLOT(setAnimationScene(pqAnimationScene*)));

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
  QObject::connect(controller, SIGNAL(timeRanges(double, double)),
    this, SLOT(setTimeRanges(double, double)));
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
void vvPlayerControlsToolbar::setTimeRanges(double start, double end)
{
  this->UI->actionFirstFrame->setToolTip(
    QString("First Frame (%1)").arg(start, 0, 'g'));
  this->UI->actionLastFrame->setToolTip(
    QString("Last Frame (%1)").arg(end, 0, 'g'));
}

//-----------------------------------------------------------------------------
void vvPlayerControlsToolbar::onPlaying(bool playing)
{
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
  // this->Implementation->Core->setSelectiveEnabledState(!playing);
}

