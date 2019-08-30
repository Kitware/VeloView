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

// LOCAL
#include "BoundingBox.h"

//-----------------------------------------------------------------------------
template <unsigned int N>
double OrientedBoundingBox<N>::GetVolume()
{
  double volume = 1.0;
  for (int i = 0; i < N; ++i)
  {
    volume *= this->Width(i);
  }
  return volume;
}

//-----------------------------------------------------------------------------
template <unsigned int N>
bool OrientedBoundingBox<N>::IsPointInside(Eigen::Matrix<double, N, 1> query)
{
  Eigen::Matrix<double, N, 1> recentered = this->Orientation.transpose() * (query - this->Center);

  bool isInside = true;
  for (size_t k = 0; k < N; ++k)
  {
    isInside &= std::abs(recentered(k)) <= this->Width(k) / 2.0;
  }
  return isInside;
}
