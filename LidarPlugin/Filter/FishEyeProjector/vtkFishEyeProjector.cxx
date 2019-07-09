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
#include "vtkFishEyeProjector.h"
#include "CameraProjection.h"

// VTK
#include <vtkImageData.h>
#include <vtkDataArray.h>
#include <vtkIntArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

// Implementation of the New function
vtkStandardNewMacro(vtkFishEyeProjector)

//-----------------------------------------------------------------------------
vtkFishEyeProjector::vtkFishEyeProjector()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(3);
}

//-----------------------------------------------------------------------------
int vtkFishEyeProjector::FillInputPortInformation(int port, vtkInformation *info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkImageData" );
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
int vtkFishEyeProjector::FillOutputPortInformation(int port, vtkInformation *info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkImageData" );
    return 1;
  }
  if (port == 1)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    return 1;
  }
  if (port == 2)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
int vtkFishEyeProjector::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get inputs
  vtkImageData *inImg = vtkImageData::GetData(inputVector[0]->GetInformationObject(0));
  vtkPolyData *pointcloud = vtkPolyData::GetData(inputVector[1]->GetInformationObject(0));

  // Get the output
  vtkImageData* outImg = vtkImageData::GetData(outputVector->GetInformationObject(0));
  vtkPolyData* outCloud = vtkPolyData::GetData(outputVector->GetInformationObject(1));
  vtkPolyData* projectedCloud = vtkPolyData::GetData(outputVector->GetInformationObject(2));
  if (!inImg || !pointcloud)
  {
    vtkGenericWarningMacro("Null pointer entry, can not launch the filter");
    return 1;
  }

  outImg->DeepCopy(inImg);
  outCloud->DeepCopy(pointcloud);
  projectedCloud->DeepCopy(pointcloud);
  projectedCloud->GetPoints()->Resize(0);
  for (int i = 0; i < projectedCloud->GetPointData()->GetNumberOfArrays(); ++i)
  {
    projectedCloud->GetPointData()->GetArray(i)->Resize(0);
  }


  vtkDataArray* intensity = outCloud->GetPointData()->GetArray("intensity");

  vtkSmartPointer<vtkIntArray> rgbArray = vtkSmartPointer<vtkIntArray>::New();
  rgbArray->SetNumberOfComponents(3);
  rgbArray->SetNumberOfTuples(outCloud->GetNumberOfPoints());
  rgbArray->SetName("RGB");
  outCloud->GetPointData()->AddArray(rgbArray);


  // Project the points in the image
  for (int pointIndex = 0; pointIndex < outCloud->GetNumberOfPoints(); ++pointIndex)
  {
    double* pos = outCloud->GetPoint(pointIndex);
    Eigen::Vector3d X(pos[0], pos[1], pos[2]);
    Eigen::Vector2d y = FisheyeProjection(W, X, true);

    // y represents the pixel coordinates using opencv convention, we need to
    // go back to vtkImageData pixel convention
    int vtkRaw = y(1);
    int vtkCol = y(0);

    if ((vtkRaw < 0) || (vtkRaw >= inImg->GetDimensions()[1]) ||
        (vtkCol < 0) || (vtkCol >= inImg->GetDimensions()[0]))
    {
      rgbArray->SetTuple3(pointIndex, 255, 255, 255);
      continue;
    }
    vtkRaw = std::min(std::max(0, vtkRaw), inImg->GetDimensions()[1] - 1);
    vtkCol = std::min(std::max(0, vtkCol), inImg->GetDimensions()[0] - 1);

    // register the point if it is valid
    double pt[3] = {y(0), y(1), 0};
    projectedCloud->GetPoints()->InsertNextPoint(pt);
    for (int i = 0; i < projectedCloud->GetPointData()->GetNumberOfArrays(); ++i)
    {
      projectedCloud->GetPointData()->GetArray(i)->InsertNextTuple(
            pointcloud->GetPointData()->GetArray(i)->GetTuple(pointIndex));
    }

    // Get its color
    double intensityValue = intensity->GetTuple1(pointIndex);
    Eigen::Vector3d color = GetRGBColourFromReflectivity(intensityValue, 0, 255);

    double rgb[3];
    for (int k = 0; k < 3; ++k)
    {
      outImg->SetScalarComponentFromDouble(vtkCol, vtkRaw, 0, k, color(2 - k));
      rgb[k] = inImg->GetScalarComponentAsDouble(vtkCol, vtkRaw, 0, k);
    }
    rgbArray->SetTuple3(pointIndex, rgb[0], rgb[1], rgb[2]);
  }

  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(projectedCloud->GetNumberOfPoints() * 2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < projectedCloud->GetNumberOfPoints(); ++i)
  {
    ids[i * 2] = 1;
    ids[i * 2 + 1] = i;
  }
  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(projectedCloud->GetNumberOfPoints(), cells.GetPointer());
  projectedCloud->SetVerts(cellArray);

  return 1;
}
