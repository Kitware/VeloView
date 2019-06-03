//=========================================================================
//
// Copyright 2018 Kitware, Inc.
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
//=========================================================================
#ifndef VTKSLAMMANAGER_H
#define VTKSLAMMANAGER_H

#include <vtkSetGet.h>
#include "vtkSlam.h"

// This custom macro is needed to make the SlamManager time agnostic
// The SlamManager needs to know when RequestData is called and if it's due
// to a new timestep being requested or due to Slam parameters being changed.
// By keeping track of the last time the parameters were modified there is
// no ambiguty anymore. This mecanimsm is similar to the one used by the ParaView filter
// PlotDataOverTime
#undef vtkCustomSetMacro/*(name,type)*/
#define vtkCustomSetMacro(name,type) \
virtual void Set##name (type _arg) \
{ \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg); \
  if (this->name != _arg) \
  { \
    this->name = _arg; \
    this->Modified(); \
    this->ParametersModificationTime.Modified(); \
  } \
}

class VTK_EXPORT vtkSlamManager : public vtkSlam
{
public:
  static vtkSlamManager *New();
  vtkTypeMacro(vtkSlamManager, vtkSlam)
  void PrintSelf(ostream& os, vtkIndent indent) override;

  //! @{ @copydoc FirstFrame
  vtkGetMacro(FirstFrame, int)
  vtkCustomSetMacro(FirstFrame, int)
  //! @}

  //! @{ @copydoc LastFrame
  vtkGetMacro(LastFrame, int)
  vtkCustomSetMacro(LastFrame, int)
  //! @}

  //! @{ @copydoc StepSize
  vtkGetMacro(StepSize, int)
  vtkCustomSetMacro(StepSize, int)
  //! @}

  //! @{ @copydoc AllFrame
  vtkGetMacro(AllFrame, bool)
  vtkCustomSetMacro(AllFrame, bool)
  //! @}

protected:
  vtkSlamManager();
  int RequestUpdateExtent(vtkInformation*,
                          vtkInformationVector**,
                          vtkInformationVector*) override;
  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector) override;

  //! Overwrite FirstFrame and LastFrame to process all the frame
  bool AllFrame = true;

  //! First frame to be process
  int FirstFrame = 0;

  //! No frame after this frame will be processed. In case StepSize is not 1 it corresponds to
  //! the last frame processed
  int LastFrame = 0;

  //! Process one frame every StepSize frames (ex: every frame, every 2 frame, 3 frame, ...)
  int StepSize = 1;

private:
  vtkSlamManager(const vtkSlamManager&) = delete;
  void operator = (const vtkSlamManager&) = delete;

  bool FirstIteration = true;
  int CurrentFrame = 0;
  vtkMTimeType LastModifyTime = 0;
  std::vector<vtkSmartPointer<vtkPolyData>> Cache;
};

#endif // VTKSLAMMANAGER_H
