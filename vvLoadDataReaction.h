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

  // This method uses the default FileOpen dialog (as against the ParaView
  // specific one used by pqLoadDataReaction).
  pqPipelineSource* loadData();

protected:
  /// Called when the action is triggered.
  virtual void onTriggered()
    {
    pqPipelineSource *source = vvLoadDataReaction::loadData();
    if (source)
      {
      emit this->loadedData(source);
      }
    }
private slots:
  void onDataLoaded(pqPipelineSource*);

private:
  Q_DISABLE_COPY(vvLoadDataReaction);
  QPointer<pqPipelineSource> PreviousSource;
};

#endif
