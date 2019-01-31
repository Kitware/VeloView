// Copyright 2019 Kitware SAS.
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

#include "vtkTemporalTransformsApplier.h"

#include <vtkCellData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTransform.h>

#include "vtkTemporalTransforms.h"

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkTemporalTransformsApplier)

//-----------------------------------------------------------------------------
vtkTemporalTransformsApplier::vtkTemporalTransformsApplier()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
  this->Interpolator = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->Interpolator->SetInterpolationTypeToLinear();
  this->InterpolateEachPoint = true;
}

//----------------------------------------------------------------------------
vtkMTimeType vtkTemporalTransformsApplier::GetMTime()
{
  return std::max(this->Superclass::GetMTime(), this->Interpolator->GetMTime());
}

//-----------------------------------------------------------------------------
int vtkTemporalTransformsApplier::RequestData(vtkInformation* vtkNotUsed(request),
                        vtkInformationVector** inputVector,
                        vtkInformationVector* outputVector)
{
  // Get the input
  vtkPolyData* pointcloud = vtkPolyData::GetData(inputVector[1]->GetInformationObject(0));
  vtkPolyData* input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  auto trajectory = vtkTemporalTransforms::CreateFromPolyData(input);

  // Fill the interpolator
  if (this->Interpolator->GetNumberOfTransforms() == 0)
  {
    auto type =  this->Interpolator->GetInterpolationType();
    this->Interpolator = trajectory->CreateInterpolator();
    this->Interpolator->SetInterpolationType(type);
  }


  // Copy the input and create some new points
  vtkPolyData* output = vtkPolyData::GetData(outputVector);
  output->ShallowCopy(pointcloud);
  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(pointcloud->GetNumberOfPoints());
  output->SetPoints(points);

  // Apply the same transform to all points. The transform is determined by the
  // pipeline time
  if (!this->InterpolateEachPoint)
  {
    // get timestamp
    vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
    double currentTimestamp = inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());

    // get the right transform
    auto transform = vtkSmartPointer<vtkTransform>::New();
    this->Interpolator->InterpolateTransform(currentTimestamp,transform);
    transform->Update();

    // apply the transform
    for (vtkIdType i = 0; i < pointcloud->GetNumberOfPoints(); i++)
    {
      float* inputPoint = reinterpret_cast<float*>
          (pointcloud->GetPoints()->GetData()->WriteVoidPointer(3*i,1));
      float* outputPoint = reinterpret_cast<float*>
          (output->GetPoints()->GetData()->WriteVoidPointer(3*i,1));
      transform->InternalTransformPoint(inputPoint, outputPoint);
    }
  }
  // Apply an individual transform to each points. The transform is determined by
  // a time array.
  else
  {
    auto timestamp = this->GetInputArrayToProcess(0, inputVector);
    if (!timestamp)
    {
      vtkErrorMacro(<<"No TimeStamp Array Selected")
      return 1;
    }
    for (vtkIdType i = 0; i < pointcloud->GetNumberOfPoints(); i++)
    {
      // get timestamp in seconds
      double currentTimestamp = timestamp->GetTuple1(i) * 1e-6;

      // get the right transform
      auto transform = vtkSmartPointer<vtkTransform>::New();
      this->Interpolator->InterpolateTransform(currentTimestamp,transform);
      transform->Update();

      // apply the transform
      float* inputPoint = reinterpret_cast<float*>
          (pointcloud->GetPoints()->GetData()->WriteVoidPointer(3*i,1));
      float* outputPoint = reinterpret_cast<float*>
          (output->GetPoints()->GetData()->WriteVoidPointer(3*i,1));
      transform->InternalTransformPoint(inputPoint, outputPoint);
    }
  }

  return 1;
}

//-----------------------------------------------------------------------------
int vtkTemporalTransformsApplier::RequestInformation(vtkInformation* vtkNotUsed(request),
                       vtkInformationVector** vtkNotUsed(inputVector),
                       vtkInformationVector* outputVector)
{
  // indicate that this filter produce continuous timstep in the interpolator range
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  double timeRange[2] = {this->Interpolator->GetMinimumT(), this->Interpolator->GetMaximumT()};
  outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);

  return 1;
}
