//=========================================================================
//
// Copyright 2018 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Data: 03-27-2018
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

#ifndef VTK_COST_FUNCTION_H
#define VTK_COST_FUNCTION_H

// EIGEN
#include <Eigen/Dense>
// CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace CostFunctions
{

//-----------------------------------------------------------------------------
struct AffineIsometryResidual
{
public:
  AffineIsometryResidual(Eigen::Matrix3d argA,
                         Eigen::Vector3d argC,
                         Eigen::Vector3d argX,
                         double argLambda)
  {
    this->A = argA;
    this->C = argC;
    this->X = argX;
    this->lambda = argLambda;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> Ac;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        Ac(i, j) = T(this->A(i, j));

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Compute Y = R(theta) * X + T - C
    T Yx = cry*crz*T(X(0)) + (srx*sry*crz-crx*srz)*T(X(1)) + (crx*sry*crz+srx*srz)*T(X(2)) + w[3] - T(C(0));
    T Yy = cry*srz*T(X(0)) + (srx*sry*srz+crx*crz)*T(X(1)) + (crx*sry*srz-srx*crz)*T(X(2)) + w[4] - T(C(1));
    T Yz = -sry*T(X(0)) + srx*cry*T(X(1)) + crx*cry*T(X(2)) + w[5] - T(C(2));

    // Compute final residual value which is:
    // Ht * A * H with H = R(theta)X + T
    Eigen::Matrix<T, 3, 1> Y;
    Y << Yx, Yy, Yz;
    T squaredResidual = T(lambda) * (Y.transpose() * Ac * Y)(0);

    // since t -> sqrt(t) is not differentiable
    // in 0, we check the value of the distance
    // infenitesimale part. If it is not finite
    // it means that the first order derivative
    // has been evaluated in 0
    if (squaredResidual < T(1e-6))
    {
      residual[0] = T(0);
    }
    else
    {
      residual[0] = ceres::sqrt(squaredResidual);
    }

    return true;
  }

private:
  Eigen::Matrix3d A;
  Eigen::Vector3d C;
  Eigen::Vector3d X;
  double lambda;
};

//-----------------------------------------------------------------------------
struct LinearDistortionResidual
{
public:
  LinearDistortionResidual(Eigen::Matrix3d argA,
                                   Eigen::Vector3d argC,
                                   Eigen::Vector3d argX,
                                   Eigen::Vector3d argT0,
                                   Eigen::Matrix3d argR0,
                                   double argTime,
                                   double argLambda)
  {
    this->A = argA;
    this->R0 = argR0;
    this->C = argC;
    this->X = argX;
    this->T0 = argT0;
    this->time = argTime;
    this->lambda = argLambda;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> Ac, R0c, R1c, Rc;
    Eigen::Matrix<T, 3, 1> Xc, Cc, T0c;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        Ac(i, j) = T(this->A(i, j));
        R0c(i, j) = T(this->R0(i, j));
      }
      Xc(i) = T(this->X(i));
      Cc(i) = T(this->C(i));
      T0c(i) = T(this->T0(i));
    }
    Eigen::Matrix<T, 3, 1> T1c; T1c << T(w[3]), T(w[4]), T(w[5]);

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Compute final rotation value
    R1c << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
           cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
              -sry,               srx*cry,               crx*cry;

    // Now, compute the rotation and translation to
    // apply to X depending on (R0, T0) and (R1, T1)
    // The applied isometry will be the linear
    // interpolation between these two transforms:
    // (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
    T s = T(this->time);
    Eigen::Matrix<T, 3, 1> Tc = (T(1.0) - s) * T0c + s * T1c;

    T r0[9], r1[9], dr[9];
    T angle_axis_r0[3], angle_axis_r1[3];
    // column major
    for (unsigned int j = 0; j < 3; ++j)
    {
      for (unsigned int i = 0; i < 3; ++i)
      {
        r0[i + 3 * j] = R0c(i, j);
        r1[i + 3 * j] = R1c(i, j);
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

    // column major
    for (unsigned int j = 0; j < 3; ++j)
    {
      for (unsigned int i = 0; i < 3; ++i)
      {
        Rc(i, j) = r[i + 3 * j];
      }
    }

    Rc = R1c;
    //Tc = T1c;

    // Compute final residual value which is:
    // Yt * A * Y with Y = R(theta) * X + T - C
    Eigen::Matrix<T, 3, 1> Y = Rc * Xc + Tc  - Cc;
    T squaredResidual = T(lambda) * (Y.transpose() * Ac * Y)(0);

    // since t -> sqrt(t) is not differentiable
    // in 0, we check the value of the distance
    // infenitesimale part. If it is not finite
    // it means that the first order derivative
    // has been evaluated in 0
    if (squaredResidual < T(1e-6))
    {
      residual[0] = T(0);
    }
    else
    {
      residual[0] = ceres::sqrt(squaredResidual);
    }

    return true;
  }

private:
  Eigen::Matrix3d A;
  Eigen::Matrix3d R0;
  Eigen::Vector3d T0;
  Eigen::Vector3d C;
  Eigen::Vector3d X;
  double time;
  double lambda;
};

}

#endif // VTK_COST_FUNCTION_H