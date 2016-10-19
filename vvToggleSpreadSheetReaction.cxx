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

#include <pqView.h>
#include <pqSpreadSheetView.h>
#include <pqSpreadSheetViewModel.h>

#include <iostream>

//-----------------------------------------------------------------------------
vvToggleSpreadSheetReaction::vvToggleSpreadSheetReaction(QAction* action, pqView* view)
  : Superclass(action),
  Action(action),
  View(view)
{
  //Hidding X, Y & Z columns by default
  if(this->View->inherits("pqSpreadSheetView"))
  {
  pqSpreadSheetView* ssview = qobject_cast<pqSpreadSheetView*>(this->View);

  //X
  ssview->getViewModel()->setVisible(2,false);
  //Y
  ssview->getViewModel()->setVisible(3,false);
  //Z
  ssview->getViewModel()->setVisible(4,false);
  }

  QObject::connect(
    this->Action, SIGNAL(triggered()),
    this, SLOT(onToggleSpreadsheet()));

  this->onToggleSpreadsheet();
}

//-----------------------------------------------------------------------------
vvToggleSpreadSheetReaction::~vvToggleSpreadSheetReaction()
{
}

//-----------------------------------------------------------------------------
void vvToggleSpreadSheetReaction::onToggleSpreadsheet()
{
  this->View->widget()->setVisible(this->Action->isChecked());
}
