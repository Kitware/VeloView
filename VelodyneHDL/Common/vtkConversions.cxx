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