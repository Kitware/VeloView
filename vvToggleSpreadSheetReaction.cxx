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
#include "vvToggleSpreadSheetReaction.h"

#include <pqApplicationCore.h>
#include <pqSettings.h>
#include <pqSpreadSheetView.h>
#include <pqSpreadSheetViewModel.h>
#include <pqView.h>

#include <iostream>

#include <QDockWidget>

//-----------------------------------------------------------------------------
vvToggleSpreadSheetReaction::vvToggleSpreadSheetReaction(
  QAction* action, pqView* view, QDockWidget* docker)
  : Superclass(action)
  , Action(action)
  , View(view)
  , Docker(docker)
{
  // Hidding the XYZ grouped coordinates column by default
  if (this->View->inherits("pqSpreadSheetView"))
  {
    pqSpreadSheetView* ssview = qobject_cast<pqSpreadSheetView*>(this->View);

    // XYZ column
    ssview->getViewModel()->setVisible(1, false);
  }

  QObject::connect(this->Action, SIGNAL(triggered()), this, SLOT(onToggleSpreadsheet()));
  pqSettings* const settings(pqApplicationCore::instance()->settings());
  action->setChecked(settings->value(action->objectName()).toBool());
}

//-----------------------------------------------------------------------------
vvToggleSpreadSheetReaction::~vvToggleSpreadSheetReaction()
{
  pqSettings* const settings(pqApplicationCore::instance()->settings());
  settings->setValue(this->Action->objectName(), this->Action->isChecked());
}

//-----------------------------------------------------------------------------
void vvToggleSpreadSheetReaction::onToggleSpreadsheet()
{
  this->Docker->setVisible(this->Action->isChecked());
  this->View->widget()->setVisible(this->Action->isChecked());
  this->View->widget()->parentWidget()->setVisible(this->Action->isChecked());
}
