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
