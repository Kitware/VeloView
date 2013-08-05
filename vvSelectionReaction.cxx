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
#include "vvSelectionReaction.h"

#include "pqRubberBandHelper.h"
#include "pqActiveObjects.h"

//-----------------------------------------------------------------------------
vvSelectionReaction::vvSelectionReaction(Modes mode, QAction* parentObject)
  : Superclass(parentObject),
  Mode(mode),
  Helper(new pqRubberBandHelper(this))
{
  switch (mode)
    {
  case SURFACE_POINTS:
    QObject::connect(
      this->Helper, SIGNAL(enableSurfacePointsSelection(bool)),
      this->parentAction(), SLOT(setEnabled(bool)));

    QObject::connect(
      this->parentAction(), SIGNAL(triggered()),
      this->Helper, SLOT(beginSurfacePointsSelection()));

    break;

  case ALL_POINTS:
    QObject::connect(
      this->Helper, SIGNAL(enableFrustumPointSelection(bool)),
      this->parentAction(), SLOT(setEnabled(bool)));

    QObject::connect(
      this->parentAction(), SIGNAL(triggered()),
      this->Helper, SLOT(beginFrustumPointsSelection()));

    break;
    }
  QObject::connect(
    this->Helper, SIGNAL(selectionModeChanged(int)),
    this, SLOT(onSelectionModeChanged(int)));

  // When a selection is marked, we revert to interaction mode.
  QObject::connect(
    this->Helper, SIGNAL(selectionFinished(int, int, int, int)),
    this->Helper, SLOT(endSelection()));

  pqActiveObjects &activeObjects = pqActiveObjects::instance();
  QObject::connect(
    &activeObjects, SIGNAL(viewChanged(pqView*)),
    this->Helper, SLOT(setView(pqView*)));
  this->Helper->setView(activeObjects.activeView());
}

//-----------------------------------------------------------------------------
vvSelectionReaction::~vvSelectionReaction()
{
}

//-----------------------------------------------------------------------------
void vvSelectionReaction::onSelectionModeChanged(int mode)
{
  switch (this->Mode)
    {
  case SURFACE_POINTS:
    this->parentAction()->setChecked(mode == pqRubberBandHelper::SELECT_POINTS);
    break;

  case ALL_POINTS:
    this->parentAction()->setChecked(mode == pqRubberBandHelper::FRUSTUM_POINTS);
    break;
    }
}
