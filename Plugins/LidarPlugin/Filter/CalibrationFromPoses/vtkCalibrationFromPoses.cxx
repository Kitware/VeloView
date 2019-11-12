//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Data: 04-01-2019
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
#include "vtkCalibrationFromPoses.h"
#include "vtkGeometricCalibration.h"
#include "vtkTemporalTransforms.h"

// VTK
#include <vtkPolyData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

// Implementation of the New function
vtkStandardNewMacro(vtkCalibrationFromPoses)

//-----------------------------------------------------------------------------
vtkCalibrationFromPoses::vtkCalibrationFromPoses()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
int vtkCalibrationFromPoses::FillInputPortInformation(int port, vtkInformation *info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    return 1;
  }
  if (port == 1)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
int vtkCalibrationFromPoses::RequestData(vtkInformation *vtkNotUsed(request),
                                                   vtkInformationVector **inputVector,
                                                   vtkInformationVector *outputVector)
{
  // Get the inputs
  vtkPolyData* input1 = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));
  vtkPolyData* input2 = vtkPolyData::GetData(inputVector[1]->GetInformationObject(0));
  auto reference = vtkTemporalTransforms::CreateFromPolyData(input1);
  auto toCalibrate = vtkTemporalTransforms::CreateFromPolyData(input2);

  // Calibrate the trajectories
  auto calibrated = EstimateCalibrationFromPosesAndApply(reference, toCalibrate,
                                                         this->TimeScaleAnalysisBound,
                                                         this->TimeScaleAnalysisStep,
                                                         this->TimeStep);

  // Get the output
  vtkPolyData* output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(calibrated);

  return 1;
}
