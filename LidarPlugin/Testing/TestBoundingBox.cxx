//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (pierre.guilbert@kitware.com)
// Date: 06-25-2019
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

// STD
#include <iostream>
#include <stdlib.h>

// VTK
#include <vtkMath.h>

// LOCAL
#include "BoundingBox.h"
#include "vtkEigenTools.h"

//-----------------------------------------------------------------------------
int TestBoundingBox()
{
  double epsilon = 1e-3;
  const unsigned int N = 4;

  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  OrientedBoundingBox<3> obb;
  for (int i = 0; i < 3; ++i)
  {
    if ((std::abs(obb.Center(i))) > epsilon || (std::abs(obb.Width(i)) > epsilon))
    {
      return 1;
    }
  }
  for (int i = 0; i < I.size(); ++i)
  {
    if (std::abs(obb.Orientation(i) - I(i)) > epsilon)
    {
      return 1;
    }
  }

  obb.Width = Eigen::Vector3d(3.0, 2.0, 4.0);
  if (std::abs(obb.GetVolume() - 24) > epsilon)
  {
    std::cout << "Got: " << obb.GetVolume() << " expected: " << 3.0 * 2.0 * 4.0 << std::endl;
    return 1;
  }

  obb.Width = Eigen::Vector3d(0.2824, 0.2847, 0.3153);
  obb.Center = Eigen::Vector3d(0.5, 0.5, 0.5);
  obb.Orientation = RollPitchYawToMatrix(Eigen::Vector3d(1.14, -1.078, 0.78));

  // check the IsInside using monte-carlo method
  unsigned int M = 50000000;
  unsigned int fracM = 0;
  for (int i = 0; i < M; ++i)
  {
    double xr = std::rand() / static_cast<double>(RAND_MAX);
    double yr = std::rand() / static_cast<double>(RAND_MAX);
    double zr = std::rand() / static_cast<double>(RAND_MAX);
    Eigen::Vector3d query(xr, yr, zr);

    if (obb.IsPointInside(query))
    {
      fracM++;
    }
  }

  double v0 = obb.GetVolume();
  double v1 = static_cast<double>(fracM) / static_cast<double>(M);

  if (std::abs(v0 - v1) / v1 > epsilon)
  {
    return 1;
  }

  return 0;
}

//-----------------------------------------------------------------------------
int main()
{
  // initialize the random generator to a fixed seed
  // for test repetability
  std::srand(1992);

  int nbrErrors = 0;
  nbrErrors += TestBoundingBox();
  return nbrErrors;
}
