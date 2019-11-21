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

// LOCAL
#include "vtkMLSPosesSmoothing.h"
#include "vtkTemporalTransforms.h"

// VTK
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>

// STD
#include <iostream>

// Implementation of the New function
vtkStandardNewMacro(vtkMLSPosesSmoothing)

//-----------------------------------------------------------------------------
int vtkMLSPosesSmoothing::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get input data
  vtkPolyData* input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  // Smooth the poses data using a MLS (moving least square)
  vtkSmartPointer<vtkTemporalTransforms> toSmoothed = vtkTemporalTransforms::CreateFromPolyData(input);
  vtkSmartPointer<vtkTemporalTransforms> smoothed = toSmoothed->MLSSmoothing(this->PolyDeg, this->KernelSize);

  // Get the output
  vtkPolyData *output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(smoothed);

  return 1;
}
