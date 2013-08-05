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
