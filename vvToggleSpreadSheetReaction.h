#ifndef __vvToggleSpreadSheetReaction_h
#define __vvToggleSpreadSheetReaction_h

#include "pqReaction.h"

class pqView;

class vvToggleSpreadSheetReaction : public pqReaction
{
  Q_OBJECT
  typedef pqReaction Superclass;
public:

  vvToggleSpreadSheetReaction(QAction* action, pqView* view);
  virtual ~vvToggleSpreadSheetReaction();

private slots:
  void onToggleSpreadsheet();

private:
  Q_DISABLE_COPY(vvToggleSpreadSheetReaction);

  QAction* Action;
  pqView* View;
};

#endif
