//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Date: 04-08-2019
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
#include "MotionModel.h"

//-----------------------------------------------------------------------------
AffineIsometry::AffineIsometry(const Eigen::Matrix3d& argR, const Eigen::Vector3d& argT, double argTime):
  R(argR), T(argT), time(argTime)
{
  this->R = argR;
  this->T = argT;
  this->time = argTime;
}

//-----------------------------------------------------------------------------
AffineIsometry SampledSensorPath::operator()(double time)
{
  Eigen::Matrix4d H = LinearTransformInterpolation<double>(this->Samples[0].R, this->Samples[0].T,
                                                           this->Samples[1].R, this->Samples[1].T,
                                                           time);
  return AffineIsometry(H.block(0, 0, 3, 3), H.block(0, 3, 3, 1), time);
}
