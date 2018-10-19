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
#ifndef __vvLoadDataReaction_h
#define __vvLoadDataReaction_h

#include "pqLoadDataReaction.h"
#include <QPointer>

#include "vvConfigure.h"

/// vvLoadDataReaction extends pqLoadDataReaction to ensure the following:
/// \li as soon as the data is loaded, we show it in the active view.
/// \li any previous data opened is closed, we only show 1 data at a time.
class vvLoadDataReaction : public pqLoadDataReaction
{
  Q_OBJECT
  typedef pqLoadDataReaction Superclass;

public:
  vvLoadDataReaction(QAction* parent, bool separatePositionFile);
  virtual ~vvLoadDataReaction();

  // This method uses the default FileOpen dialog (as against the ParaView
  // specific one used by pqLoadDataReaction).
  pqPipelineSource* loadData();

protected:
  /// Called when the action is triggered.
  virtual void onTriggered()
  {
    pqPipelineSource* source = vvLoadDataReaction::loadData();
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
  const bool SeparatePositionFile;
};

#endif
