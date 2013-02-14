#ifndef __vvSelectionReaction_h
#define __vvSelectionReaction_h

#include "pqReaction.h"

class pqRubberBandHelper;

class vvSelectionReaction : public pqReaction
{
  Q_OBJECT
  typedef pqReaction Superclass;
public:
  enum Modes
    {
    SURFACE_POINTS,
    ALL_POINTS
    };

  vvSelectionReaction(Modes mode, QAction* parentAction);
  virtual ~vvSelectionReaction();

private slots:
  void onSelectionModeChanged(int);

private:
  Q_DISABLE_COPY(vvSelectionReaction);
  Modes Mode;
  pqRubberBandHelper* Helper;
};

#endif
