//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Data: 03-27-2019
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

#ifndef VTK_MLS_POSES_SMOOTHING_H
#define VTK_MLS_POSES_SMOOTHING_H

// VTK
#include <vtkPolyDataAlgorithm.h>

class VTK_EXPORT vtkMLSPosesSmoothing : public vtkPolyDataAlgorithm
{
public:
  static vtkMLSPosesSmoothing *New();
  vtkTypeMacro(vtkMLSPosesSmoothing, vtkPolyDataAlgorithm)

  vtkGetMacro(PolyDeg, int)
  vtkSetMacro(PolyDeg, int)

  vtkGetMacro(KernelSize, int)
  vtkSetMacro(KernelSize, int)

protected:
  vtkMLSPosesSmoothing() = default;

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

  //! Degree of the polynomial parametric function
  //! used to fit while performing Moving Least Square
  int PolyDeg = 3;

  //! Size of the kernel rectangular function used
  int KernelSize = 10;
private:
  vtkMLSPosesSmoothing(const vtkMLSPosesSmoothing&) = delete;
  void operator=(const vtkMLSPosesSmoothing&) = delete;
};

#endif // VTK_MLS_POSES_SMOOTHING_H
