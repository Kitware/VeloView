//=========================================================================
//
// Copyright 2012,2013,2014 Kitware, Inc.
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
#include "vtkProcessingSample.h"

#include "vtkFloatArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkStreamingDemandDrivenPipeline.h"

#include <Eigen/Dense>

#include <limits>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkProcessingSample)

//----------------------------------------------------------------------------
vtkProcessingSample::vtkProcessingSample()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkProcessingSample::~vtkProcessingSample()
{
}

//----------------------------------------------------------------------------
int vtkProcessingSample::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkPoints* points = input->GetPoints();
  if (!points || !points->GetNumberOfPoints())
  {
    return 1;
  }

  Eigen::Vector3d meanpoints;
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
  {
    Eigen::Vector3d point;
    points->GetPoint(i, point.data());
    meanpoints += point;
  }
  meanpoints /= points->GetNumberOfPoints();

  // Create a sphere at the center location
  vtkNew<vtkSphereSource> spheresource;
  spheresource->SetCenter(meanpoints.data());
  spheresource->SetRadius(0.5);
  spheresource->Update();

  // Get the intensities and laser ids for points
  vtkUnsignedCharArray* intensities =
    vtkUnsignedCharArray::SafeDownCast(input->GetPointData()->GetArray("intensity"));
  vtkUnsignedCharArray* laserid =
    vtkUnsignedCharArray::SafeDownCast(input->GetPointData()->GetArray("laser_id"));

  // Compute the max, min intensity for laser 3
  int minimum_intensity = std::numeric_limits<int>::max();
  int maximum_intensity = 0;
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
  {
    int laser = laserid->GetValue(i);
    if (laser == 3)
    {
      int point_intensity = intensities->GetValue(i);
      if (point_intensity < minimum_intensity)
      {
        minimum_intensity = point_intensity;
      }
      if (point_intensity > maximum_intensity)
      {
        maximum_intensity = point_intensity;
      }
    }
  }

  // Add field data with some numbers
  vtkSmartPointer<vtkFloatArray> fielddata = vtkSmartPointer<vtkFloatArray>::New();
  fielddata->SetName("Values");
  fielddata->SetNumberOfComponents(1);
  fielddata->InsertNextValue(minimum_intensity);
  fielddata->InsertNextValue(maximum_intensity);

  vtkSmartPointer<vtkPolyData> outputPolyData = vtkSmartPointer<vtkPolyData>::New();
  outputPolyData->DeepCopy(spheresource->GetOutput());
  outputPolyData->GetFieldData()->AddArray(fielddata);

  output->ShallowCopy(outputPolyData);

  return 1;
}

//----------------------------------------------------------------------------
void vtkProcessingSample::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
