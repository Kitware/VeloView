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

#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

// EIGEN
#include <Eigen/Dense>

// CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>

/**
* \class AffineIsometry
* \brief represents a bijective transformation from a 3D
*        euclidean affine space into another whose associated
*        linear application is an orthogonal morphism
*/
class AffineIsometry
{
public:
  AffineIsometry() {};
  AffineIsometry(const Eigen::Matrix3d& argR, const Eigen::Vector3d& argT, double argTime);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  double time = 0;
};

/**
* \class SampledSensorPath
* \brief represents the sampled sensor path estimated
*        using SLAM or measured using GPS / IMU data.
*        The orientation and position of the sensor for
*        a given time t that does not correspond to a
*        sampled time will be interpolated using linear
*        or spline interpolation (in R^3 and SO(3))
*/
class SampledSensorPath
{
public:
  std::vector<AffineIsometry> Samples = std::vector<AffineIsometry>(2);

  // return the affine isometry corresponding
  // to the requested time using a spline or
  // linear interpolation
  AffineIsometry operator()(double t);
};

/**
* \class LinearTransformInterpolation
* \brief Perform the linear interpolation between
*        two isometric affine transforms. The rotational
*        part is interpolated using a SLERP and the translation
*        part is linearly interpolated. The function is templated
*        to be usable with dual number and autodifferentiation
*/
//-----------------------------------------------------------------------------
template <typename T>
Eigen::Matrix<T, 4, 4> LinearTransformInterpolation(const Eigen::Matrix<T, 3, 3>& R0, const Eigen::Matrix<T, 3, 1>& T0,
                                                    const Eigen::Matrix<T, 3, 3>& R1, const Eigen::Matrix<T, 3, 1>& T1,
                                                    T s)
{
  // Linearly interpolate the translation part
  Eigen::Matrix<T, 3, 1> Tc = (T(1.0) - s) * T0 + s * T1;

  // SLERP interpolation of the rotational part
  T r0[9], r1[9];
  T angle_axis_r0[3], angle_axis_r1[3];

  // column major
  for (unsigned int j = 0; j < 3; ++j)
  {
    for (unsigned int i = 0; i < 3; ++i)
    {
      r0[j + 3 * i] = R0(i, j);
      r1[j + 3 * i] = R1(i, j);
    }
  }

  T q0[4], q1[4], q[4];
  // Rotation matrix to quaternions
  ceres::RotationMatrixToAngleAxis(r0, angle_axis_r0);
  ceres::RotationMatrixToAngleAxis(r1, angle_axis_r1);
  ceres::AngleAxisToQuaternion(angle_axis_r0, q0);
  ceres::AngleAxisToQuaternion(angle_axis_r1, q1);

  // Canonical scalar product on quaternion
  T dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];

  // To prevent the SLERP interpolation to take the long path
  // we first check their relative orientation. If the angle
  // is superior to 90 we take the opposite quaternion which
  // is closer and represents the same rotation
  if(dot < T(0.0))
  {
    dot = -dot;
    for (unsigned int k = 0; k < 4; ++k)
    {
      q1[k] = T(-1.0) * q1[k];
    }
  }

  // To avoid division by zero, perform a linear interpolation (LERP), if our
  // quarternions are nearly in the same direction, otherwise resort
  // to spherical linear interpolation. In the limiting case (for small
  // angles), SLERP is equivalent to LERP.
  T t1, t2;
  if ((T(1.0) - ceres::abs(dot)) < T(1e-6))
  {
    t1 = T(1.0) - s;
    t2 = s;
  }
  else
  {
    // Angle (defined by the canonical scalar product for quaternions)
    // between the two quaternions
    const T theta = ceres::acos(dot);
    t1 = ceres::sin((T(1.0) - s) * theta) / ceres::sin(theta);
    t2 = ceres::sin(s * theta) / ceres::sin(theta);
  }
  for (unsigned int k = 0; k < 4; ++k)
  {
    q[k] = t1 * q0[k] + t2 * q1[k];
  }
  T r[9];
  ceres::QuaternionToRotation(q, r);

  Eigen::Matrix<T, 4, 4> H = Eigen::Matrix<T, 4, 4>::Zero();
  // column major
  for (unsigned int j = 0; j < 3; ++j)
  {
    for (unsigned int i = 0; i < 3; ++i)
    {
      H(i, j) = r[i + 3 * j];
    }
    H(j, 3) = Tc(j);
  }
  H(3, 3) = T(1);
  return H;
}

#endif // MOTION_MODEL_H
