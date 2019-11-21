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

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

// Eigen
#include <Eigen/Dense>

/**
 * @class OrientedBoundingBox
 * @brief Oriented N-dimensional bounding box
 */
template <unsigned int N>
class OrientedBoundingBox
{
public:
  OrientedBoundingBox()
  {
    this->Center = Eigen::Matrix<double, N, 1>::Zero();
    this->Width = Eigen::Matrix<double, N, 1>::Zero();
    this->Orientation = Eigen::Matrix<double, N, N>::Identity();
  }

  OrientedBoundingBox(Eigen::Matrix<double, N, 1> argCenter,
                      Eigen::Matrix<double, N, 1> argWidth,
                      Eigen::Matrix<double, N, N> argOrientation)
  {
    this->Center = argCenter;
    this->Width = argWidth;
    this->Orientation = argOrientation;
  }

  OrientedBoundingBox(Eigen::Matrix<double, N, 1> minCorner, Eigen::Matrix<double, N, 1> maxCorner)
  {
    this->Center = (minCorner + maxCorner) / 2.0;
    this->Width = maxCorner - minCorner;
    this->Orientation = Eigen::Matrix<double, N, N>::Identity();
  }

  /// Return the volume of the bounding box
  double GetVolume();

  /// Return true if the query point is inside the bb
  bool IsPointInside(Eigen::Matrix<double, N, 1> query);

  // oriented bounding box is represented by its
  // -Center position
  // -Width along dimensions
  // -Orientation to the non-oriented bb
  Eigen::Matrix<double, N, 1> Center;
  Eigen::Matrix<double, N, 1> Width;
  Eigen::Matrix<double, N, N> Orientation;

  // Semantic of the bounding box
  std::string Type = "N.A.";
};

// Implementation
#include "BoundingBox.txx"

#endif // BOUNDING_BOX_H
