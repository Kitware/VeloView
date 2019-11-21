//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Date: 20-06-2019
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

#ifndef CERES_TOOLS_H
#define CERES_TOOLS_H

// EIGEN
#include <Eigen/Dense>

// CERES
#include <ceres/ceres.h>

namespace ceres
{
//-----------------------------------------------------------------------------
template <typename T>
Eigen::Matrix<T, 3, 1> MatrixToRollPitchYaw(const Eigen::Matrix<T, 3, 3>& rotation)
{
  Eigen::Matrix<T, 3, 1> eulerAngles;
  eulerAngles(0) = ceres::atan2(rotation(2, 1), rotation(2, 2));
  eulerAngles(1) = -ceres::asin(rotation(2, 0));
  eulerAngles(2) = ceres::atan2(rotation(1, 0), rotation(0, 0));

  return eulerAngles;
}

//-----------------------------------------------------------------------------
template <typename T>
Eigen::Matrix<T, 3, 3> RollPitchYawToMatrix(T roll, T pitch, T yaw)
{
  Eigen::Matrix<T, 3, 3> R;
  T crx = ceres::cos(roll); T srx = ceres::sin(roll);
  T cry = ceres::cos(pitch); T sry = ceres::sin(pitch);
  T crz = ceres::cos(yaw); T srz = ceres::sin(yaw);

  // Compute final rotation value
  R << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
       cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
          -sry,               srx*cry,               crx*cry;
  return R;
}

//-----------------------------------------------------------------------------
std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6> >
Express6DoFPoseInOtherCoordinateFrame(const Eigen::Matrix<double, 6, 1>& dof6,
                                      const Eigen::Matrix<double, 6, 1>& newReferenceFrameDof6)
{
  // Initial differential values
  Eigen::Matrix<double, 6, 1> j0, j1, j2, j3, j4, j5;
  j0 << 1, 0, 0, 0, 0, 0;
  j1 << 0, 1, 0, 0, 0, 0;
  j2 << 0, 0, 1, 0, 0, 0;
  j3 << 0, 0, 0, 1, 0, 0;
  j4 << 0, 0, 0, 0, 1, 0;
  j5 << 0, 0, 0, 0, 0, 1;

  // convert the dof6 pose to jet structure
  Eigen::Matrix<ceres::Jet<double, 6>, 6, 1> dof6jet;
  dof6jet(0) = ceres::Jet<double, 6>(dof6(0), j0);
  dof6jet(1) = ceres::Jet<double, 6>(dof6(1), j1);
  dof6jet(2) = ceres::Jet<double, 6>(dof6(2), j2);
  dof6jet(3) = ceres::Jet<double, 6>(dof6(3), j3);
  dof6jet(4) = ceres::Jet<double, 6>(dof6(4), j4);
  dof6jet(5) = ceres::Jet<double, 6>(dof6(5), j5);

  // Convert dof6 parameters to rotation matrix and
  // position vector
  Eigen::Matrix<ceres::Jet<double, 6>, 3, 3> oldR = ceres::RollPitchYawToMatrix(dof6jet(0), dof6jet(1), dof6jet(2));
  Eigen::Matrix<ceres::Jet<double, 6>, 3, 1> oldT(dof6jet(3), dof6jet(4), dof6jet(5));

  // convert dof6 parameters of the new reference frame
  // to rotation matrix and position vector
  Eigen::Matrix<double, 3, 3> refR = RollPitchYawToMatrix(newReferenceFrameDof6(0),
                                                          newReferenceFrameDof6(1),
                                                          newReferenceFrameDof6(2));
  Eigen::Matrix<ceres::Jet<double, 6>, 3, 3> refRj;
  Eigen::Matrix<ceres::Jet<double, 6>, 3, 1> refTj;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      refRj(i, j) = ceres::Jet<double, 6>(refR(i, j));
    }
    refTj(i) = ceres::Jet<double, 6>(newReferenceFrameDof6(i + 3));
  }

  // Compute the new rotation matrix and position vector
  // according to the new reference frame
  Eigen::Matrix<ceres::Jet<double, 6>, 3, 3> newR = refRj.transpose() * oldR;
  Eigen::Matrix<ceres::Jet<double, 6>, 3, 1> newT = refRj.transpose() * (oldT - refTj);

  // new dof6 parameters of the pose expressed in the new
  // reference frame
  Eigen::Matrix<ceres::Jet<double, 6>, 6, 1> newDof6jet;
  newDof6jet.block(0, 0, 3, 1) = ceres::MatrixToRollPitchYaw(newR);
  newDof6jet.block(3, 0, 3, 1) = newT;

  // extract and store the new dof6 values and the jacobian
  // of the mapping between the dof6 expressed into the world
  // reference frame and dof6 expressed in the new reference frame
  Eigen::Matrix<double, 6, 1> newDof6;
  Eigen::Matrix<double, 6, 6> newDof6Jacobian;
  for (int i = 0; i < 6; ++i)
  {
    newDof6(i) = newDof6jet(i).a;
    for (int j = 0; j < 6; ++j)
    {
      newDof6Jacobian(i, j) = newDof6jet(i).v(j);
    }
  }

  return std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6> >(newDof6, newDof6Jacobian);
}
};

#endif // CERES_TOOLS_H
