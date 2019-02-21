//=========================================================================
//
// Copyright 2012,2013,2014,2019 Kitware, Inc.
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

#include "vtkConversions.h"

// LOCAL
#include "vtkEigenTools.h"

// VTK
#include <vtkMatrix4x4.h>


//----------------------------------------------------------------------------
std::vector<Eigen::Vector3d> vtkPointsToEigenVector(vtkPoints* points)
{
  std::vector<Eigen::Vector3d> eigenVector(points->GetNumberOfPoints());
  for (unsigned int k = 0; k < points->GetNumberOfPoints(); ++k)
  {
    points->GetPoint(k, eigenVector[k].data());
  }
  return eigenVector;
}


//----------------------------------------------------------------------------
vtkSmartPointer<vtkPoints> eigenVectorToVTKPoints(std::vector<Eigen::Vector3d> const& points)
{
  auto newPoints = vtkSmartPointer<vtkPoints>::New();
  newPoints->SetNumberOfPoints(points.size());
  float temp[3];
  for (size_t i = 0; i < points.size(); ++i)
  {
    temp[0] = points[i].x();
    temp[1] = points[i].y();
    temp[2] = points[i].z();
    newPoints->SetPoint(i, temp);
  }
  return newPoints;
}

//----------------------------------------------------------------------------
std::pair<Eigen::Vector3d, Eigen::Vector3d> GetPoseParamsFromTransform(vtkSmartPointer<vtkTransform> transform)
{
  vtkSmartPointer<vtkMatrix4x4> H = transform->GetMatrix();

  // Position
  Eigen::Vector3d position;
  position(0) = H->Element[0][3];
  position(1) = H->Element[1][3];
  position(2) = H->Element[2][3];
  // orientation
  Eigen::Vector3d angles;
  angles(0) = std::atan2(H->Element[2][1], H->Element[2][2]);
  angles(1) = -std::asin(H->Element[2][0]);
  angles(2) = std::atan2(H->Element[1][0], H->Element[0][0]);

  return std::pair<Eigen::Vector3d, Eigen::Vector3d>(angles, position);
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkTransform> GetTransformFromPosesParams(std::pair<Eigen::Vector3d, Eigen::Vector3d> dof6)
{
  Eigen::Matrix3d R = RollPitchYawToMatrix(dof6.first);
  Eigen::Vector3d T = dof6.second;

  vtkSmartPointer<vtkMatrix4x4> H = vtkSmartPointer<vtkMatrix4x4>::New();
  H->Zero();
  for (unsigned int i = 0; i < 3; ++i)
  {
    for (unsigned int j = 0; j < 3; ++j)
    {
      H->SetElement(i, j, R(i, j));
    }
    H->SetElement(i, 3, T(i));
  }
  H->SetElement(3, 3, 1.0);

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->SetMatrix(H.Get());
  return transform;
}

//------------------------------------------------------------------------------
Eigen::Matrix3d RotationMatrixFromTransform(vtkTransform* transform)
{
  vtkSmartPointer<vtkMatrix4x4> H = vtkSmartPointer<vtkMatrix4x4>::New();
  transform->GetMatrix(H);
  Eigen::Matrix3d out = Eigen::Matrix3d(3, 3);
  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 3; j++)
    {
      out(i, j) = H->GetElement(i, j);
    }
  }
  return out;
}

//------------------------------------------------------------------------------
Eigen::Vector3d PositionVectorFromTransform(vtkTransform* transform)
{
  double pos[3];
  transform->GetPosition(pos);
  return Eigen::Vector3d(pos[0], pos[1], pos[2]);
}
