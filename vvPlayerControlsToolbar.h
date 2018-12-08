/*=========================================================================

   Program: ParaView
   Module:    pqVCRToolbar.h

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
#ifndef VVPLAYERCONTROLSTOOLBAR_H
#define VVPLAYERCONTROLSTOOLBAR_H

#include <QToolBar>

class vvPlayerControlsController;
class pqAnimationScene;
class vtkSMProxy;

/// This class is a mixed of pqVCRToolbar and pqTimAnimationWidget
///
class vvPlayerControlsToolbar : public QToolBar
{
  Q_OBJECT
  Q_PROPERTY(double timeValue READ timeValue WRITE setTimeValue NOTIFY timeValueChanged)
  Q_PROPERTY(int timeStepCount READ timeStepCount WRITE setTimeStepCount)
public:
  vvPlayerControlsToolbar(QWidget* parentObject=0);
  ~vvPlayerControlsToolbar();

public slots:
  /// Get/set the current time value.
  void setTimeValue(double time);
  double timeValue() const;

  /// Get/set the number of timesteps.
  void setTimeStepCount(int count);
  int timeStepCount() const;

protected slots:
  void onPlaying(bool);
  void onSpeedChanged();
  void setAnimationScene(pqAnimationScene*);
  void PressSlider();
  void ReleaseSlider();
  void setTimeStep(int);

signals:
  void speedChange(double);
  void dummySignal();
  void timeValueChanged();

private:
  Q_DISABLE_COPY(vvPlayerControlsToolbar)

  void constructor();
  vtkSMProxy* timeKeeper() const;
  class pqInternals;
  pqInternals* UI;

  vvPlayerControlsController* Controller;
};

#endif // VVPLAYERCONTROLSTOOLBAR_H
