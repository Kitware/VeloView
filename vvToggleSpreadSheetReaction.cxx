#include "vvToggleSpreadSheetReaction.h"

#include <pqView.h>

//-----------------------------------------------------------------------------
vvToggleSpreadSheetReaction::vvToggleSpreadSheetReaction(QAction* action, pqView* view)
  : Superclass(action),
  Action(action),
  View(view)
{

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
  this->View->getWidget()->setVisible(this->Action->isChecked());
}
