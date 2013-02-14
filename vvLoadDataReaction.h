#ifndef __vvLoadDataReaction_h
#define __vvLoadDataReaction_h

#include "pqLoadDataReaction.h"
#include <QPointer>

/// vvLoadDataReaction extends pqLoadDataReaction to ensure the following:
/// \li as soon as the data is loaded, we show it in the active view.
/// \li any previous data opened is closed, we only show 1 data at a time.
class vvLoadDataReaction : public pqLoadDataReaction
{
  Q_OBJECT
  typedef pqLoadDataReaction Superclass;
public:
  vvLoadDataReaction(QAction* parent);
  virtual ~vvLoadDataReaction();

private slots:
  void onDataLoaded(pqPipelineSource*);

private:
  Q_DISABLE_COPY(vvLoadDataReaction);
  QPointer<pqPipelineSource> PreviousSource;
};

#endif
