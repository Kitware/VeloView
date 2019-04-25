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

#include <pqSpreadSheetView.h>
#include <pqSpreadSheetViewModel.h>
#include <pqView.h>

#include <iostream>

//-----------------------------------------------------------------------------
vvToggleSpreadSheetReaction::vvToggleSpreadSheetReaction(QAction* action, pqView* view)
  : Superclass(action)
  , Action(action)
  , View(view)
{
  QObject::connect(this->Action, SIGNAL(triggered()), this, SLOT(onToggleSpreadsheet()));
  this->onToggleSpreadsheet();
}

//-----------------------------------------------------------------------------
void vvToggleSpreadSheetReaction::onToggleSpreadsheet()
{
  // Hide / Show Dock
  this->View->widget()->parentWidget()->parentWidget()->parentWidget()->parentWidget()->setVisible(this->Action->isChecked());

  pqSpreadSheetView* ssview = qobject_cast<pqSpreadSheetView*>(this->View);

  // Hide Block ID and XYZ column
  ssview->getViewModel()->setVisible(0, false);
  ssview->getViewModel()->setVisible(2, false);
  // Display parameters
  ssview->getViewModel()->setDecimalPrecision(3);
  ssview->getViewModel()->setFixedRepresentation(true);
}
