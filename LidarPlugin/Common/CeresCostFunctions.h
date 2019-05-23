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

#ifndef CERES_COST_FUNCTIONS_H
#define CERES_COST_FUNCTIONS_H

// EIGEN
#include <Eigen/Dense>

// CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// LOCAL
#include "MotionModel.h"

namespace CostFunctions
{
/**
* \class MahalanobisDistanceAffineIsometryResidual
* \brief Cost function to minimize to estimate the affine isometry transformation
*        (rotation and translation) that minimizes the mahalanobis distance
*        between a point X and its neighborhood encoded by the mean point C
*        and the variance covariance matrix A
*/
//-----------------------------------------------------------------------------
struct MahalanobisDistanceAffineIsometryResidual
{
public:
  MahalanobisDistanceAffineIsometryResidual(const Eigen::Matrix3d& argA,
                                            const Eigen::Vector3d& argC,
                                            const Eigen::Vector3d& argX,
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
    // Create sin / cos evaluation variables in static way.
    // The idea is that all residual function will need to
    // evaluate those sin / cos so we will only compute then
    // once each time the parameters values change
    static T crx, cry, crz, srx, sry, srz;
    static T lastWValues[6] = {T(-1.0), T(-1.0), T(-1.0), T(-1.0), T(-1.0), T(-1.0)};
    if ((w[0] != lastWValues[0]) || (w[1] != lastWValues[1]) || (w[2] != lastWValues[2]) ||
        (w[3] != lastWValues[3]) || (w[4] != lastWValues[4]) || (w[5] != lastWValues[5]))
    {
      // store sin / cos values for this angle
      crx = ceres::cos(w[0]); srx = ceres::sin(w[0]);
      cry = ceres::cos(w[1]); sry = ceres::sin(w[1]);
      crz = ceres::cos(w[2]); srz = ceres::sin(w[2]);

      for (int k = 0; k < 6; ++k)
        lastWValues[k] = w[k];
    }

    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> Ac;
    for (int i = 0; i < 9; ++i)
        Ac(i) = T(this->A(i));

    // Compute Y = R(theta) * X + T - C
    Eigen::Matrix<T, 3, 1> Y;
    Y(0) = cry*crz*T(X(0)) + (srx*sry*crz-crx*srz)*T(X(1)) + (crx*sry*crz+srx*srz)*T(X(2)) + w[3] - T(C(0));
    Y(1) = cry*srz*T(X(0)) + (srx*sry*srz+crx*crz)*T(X(1)) + (crx*sry*srz-srx*crz)*T(X(2)) + w[4] - T(C(1));
    Y(2) = -sry*T(X(0)) + srx*cry*T(X(1)) + crx*cry*T(X(2)) + w[5] - T(C(2));

    // Compute final residual value which is:
    // Ht * A * H with H = R(theta)X + T
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

/**
* \class MahalanobisDistanceLinearDistortionResidual
* \brief Cost function to minimize to estimate the rotation R1 and translation T1 so that:
         The linearly interpolated transform:
         (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
         applies to X acquired at time t minimizes the mahalanobis distance.
*/
//-----------------------------------------------------------------------------
struct MahalanobisDistanceLinearDistortionResidual
{
public:
  MahalanobisDistanceLinearDistortionResidual(const Eigen::Matrix3d& argA,
                                              const Eigen::Vector3d& argC,
                                              const Eigen::Vector3d& argX,
                                              const Eigen::Vector3d& argT0,
                                              const Eigen::Matrix3d& argR0,
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
    Eigen::Matrix<T, 3, 3> Ac, R0c, R1c;
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
    Eigen::Matrix<T, 4, 4> H = LinearTransformInterpolation<T>(R0c, T0c, R1c, T1c, T(this->time));
    Eigen::Matrix<T, 3, 3> Rc = H.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> Tc = H.block(0, 3, 3, 1);

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

/**
* \class MahalanobisDistanceIsometryAndLinearDistortionResidual
* \brief Cost function to minimize to estimate the rotation R1 and translation T1 so that:
         The linearly interpolated transform:
         (R, T) = (R1 * R1^t, t * R1 * T + T)
         applies to X acquired at time t minimizes the mahalanobis distance.
*/
//-----------------------------------------------------------------------------
struct MahalanobisDistanceIsometryAndLinearDistortionResidual
{
public:
  MahalanobisDistanceIsometryAndLinearDistortionResidual(const Eigen::Matrix3d& argA,
                                              const Eigen::Vector3d& argC,
                                              const Eigen::Vector3d& argX,
                                              double argTime,
                                              double argLambda)
  {
    this->A = argA;
    this->C = argC;
    this->X = argX;
    this->time = argTime;
    this->lambda = argLambda;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> Ac, R1c;
    Eigen::Matrix<T, 3, 1> Xc, Cc;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        Ac(i, j) = T(this->A(i, j));
      }
      Xc(i) = T(this->X(i));
      Cc(i) = T(this->C(i));
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
    Eigen::Matrix<T, 4, 4> H = LinearTransformInterpolation<T>(R1c, T1c, R1c * R1c, R1c * T1c + T1c, T(this->time));
    Eigen::Matrix<T, 3, 3> Rc = H.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> Tc = H.block(0, 3, 3, 1);

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
  Eigen::Vector3d C;
  Eigen::Vector3d X;
  double time;
  double lambda;
};

/**
* \class MahalanobisDistanceLinearDistortionResidual
* \brief Cost function to minimize to estimate the rotation R1 and translation T1 so that:
         The linearly interpolated transform:
         (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
         applies to X acquired at time t minimizes the mahalanobis distance.
*/
//-----------------------------------------------------------------------------
struct MahalanobisDistanceInterpolatedMotionResidual
{
public:
  MahalanobisDistanceInterpolatedMotionResidual(const Eigen::Matrix3d& argA,
                                                const Eigen::Vector3d& argC,
                                                const Eigen::Vector3d& argX,
                                                double argTime,
                                                double argLambda)
  {
    this->A = argA;
    this->C = argC;
    this->X = argX;
    this->time = argTime;
    this->lambda = argLambda;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> Ac, R0c, R1c;
    Eigen::Matrix<T, 3, 1> Xc, Cc;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        Ac(i, j) = T(this->A(i, j));
      }
      Xc(i) = T(this->X(i));
      Cc(i) = T(this->C(i));
    }

    // Get the current estimation positions
    Eigen::Matrix<T, 3, 1> T0c(w[3], w[4], w[5]);
    Eigen::Matrix<T, 3, 1> T1c(w[9], w[10], w[11]);

    // Get the current estimation orientations

    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);
    R0c << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
           cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
              -sry,               srx*cry,               crx*cry;

    crx = ceres::cos(w[6]); srx = ceres::sin(w[6]);
    cry = ceres::cos(w[7]); sry = ceres::sin(w[7]);
    crz = ceres::cos(w[8]); srz = ceres::sin(w[8]);
    R1c << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
           cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
              -sry,               srx*cry,               crx*cry;

    // Now, compute the rotation and translation to
    // apply to X depending on (R0, T0) and (R1, T1)
    // The applied isometry will be the linear
    // interpolation between these two transforms:
    // (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
    Eigen::Matrix<T, 4, 4> H = LinearTransformInterpolation<T>(R0c, T0c, R1c, T1c, T(this->time));
    Eigen::Matrix<T, 3, 3> Rc = H.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> Tc = H.block(0, 3, 3, 1);

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
  Eigen::Vector3d C;
  Eigen::Vector3d X;
  double time;
  double lambda;
};

/**
* \class FrobeniusDistanceRotationCalibrationResidual
* \brief Cost function to minimize to estimate the calibration rotation between two sensors
*        based on their trajectory. To do that, we exploit the "solid-system"
*        constraint that links the coordinate reference frame of the two sensors.
*
* Using the geometric constraints that come from the solid-assumption, it is
* possible to estimate the 3-DoF rotation calibration (R in SO(3)) between
* two sensors by using the estimation of their poses over the time.
*
* Let's t0 and t1 be two times
* Let's R be the orientation of the sensor 1 according to the sensor 2 reference frame
* Since we have the solid-assumption, R is constant and not time depending
*
* Let's P0 (resp P1) in SO(3) be the orientation of the sensor 1 at time t0 (resp t1)
*
* Let's Q0 (resp Q1) in SO(3) be the orientation of the sensor 2 at time t0 (resp t1)
*
* From these two temporal points on the poses "trajectory", it is possible to express
* the change of orientation between the sensor 1 at time t0 and sensor 1 at time t1
* using two differents way
*
* 1- By using the poses of the sensor 1 at time t0 and t1:
*    dR0 = P0' * P1
*
* 2- By using the solid-assumption and firstly express the point in the
*    other sensor reference frame using the calibration parameters. Then, using
*    method 1 for the second sensor and finally using the calibration again to
*    dR1 = R' * Q0' * Q1 * R
*
* And finally, we are looking for R that satisfy:
* dR1 = dR0
*
* This lead to a non-linear least square problem that can be solved using a
* levenberg-marquardt algorithm. The non-linearity comes from R belonging to
* SO(3) manifold. We estimate R using the Euler-Angle mapping between R^3 and SO(3)
* R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
*/
//-----------------------------------------------------------------------------
struct FrobeniusDistanceRotationCalibrationResidual
{
public:
  FrobeniusDistanceRotationCalibrationResidual(const Eigen::Matrix3d& argP1, const Eigen::Matrix3d& argP2,
                                               const Eigen::Matrix3d& argQ1, const Eigen::Matrix3d& argQ2)
  {
    this->P1 = argP1; this->P2 = argP2;
    this->Q1 = argQ1; this->Q2 = argQ2;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> P1j, P2j, Q1j, Q2j;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        P1j(i, j) = T(this->P1(i, j));
        P2j(i, j) = T(this->P2(i, j));
        Q1j(i, j) = T(this->Q1(i, j));
        Q2j(i, j) = T(this->Q2(i, j));
      }
    }

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R0;
    R0 << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
          cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
             -sry,               srx*cry,               crx*cry;

    // Compute the residual matrix
    Eigen::Matrix<T, 3, 3> ResidualMatrix = R0.transpose() * Q1j.transpose() * Q2j * R0 - P1j.transpose() * P2j;

    // Compute final residual value which is the frobenius norme
    // of the residual matrix
    T squaredResidual = (ResidualMatrix.transpose() * ResidualMatrix).trace();

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
  Eigen::Matrix3d P1, P2, Q1, Q2;
};

/**
* \class FrobeniusDistanceRotationAndTranslationCalibrationResidual
* \brief Cost function to minimize to estimate the calibration rotation and translation
*        between two sensors based on their trajectory. To do that, we exploit the
*        "solid-system" constraint that links the coordinate reference frame of the two sensors.
*
* Using the geometric constraints that come from the solid-assumption, it is
* possible to estimate the 6-DoF calibration (R in SO(3), T in R^3) between
* two sensors by using the estimation of their poses over the time.
*
* Let's t0 and t1 be two times
* Let's R, T be the pose of the sensor 1 according to the sensor 2 reference frame
* Since we have the solid-assumption, R and T are constant and not time depending
*
* Let's P0 (resp P1) in SO(3) be the orientation of the sensor 1 at time t0 (resp t1)
* Let's V0 (resp V1) in R^3 be the position of the sensor 1 at time t0 (resp t1)
*
* Let's Q0 (resp Q1) in SO(3) be the orientation of the sensor 2 at time t0 (resp t1)
* Let's U0 (resp U1) in R^3 be the position of the sensor 2 at time t0 (resp t1)
*
* From these two temporal points on the poses "trajectory", it is possible to express
* the change of reference frame between the sensor 1 at time t0 and sensor 1 at time t1
* using two differents way
*
* 1- By using the poses of the sensor 1 at time t0 and t1:
*    dR0 = P0' * P1
*    dT0 = P0' * (V1 - V0)
*
* 2- By using the solid-assumption and firstly express the point in the
*    other sensor reference frame using the calibration parameters. Then, using
*    method 1 for the second sensor and finally using the calibration again to
*    dR1 = R' * Q0' * Q1 * R
*    dT1 = R0' * (Q0' * (Q1 * T + (U1 - U0)) - T)
*
* And finally, we are looking for R and T that satisfies:
* dR1 = dR0
* dT1 = dT0
*
* This lead to a non-linear least square problem that can be solved using a
* levenberg-marquardt algorithm. The non-linearity comes from R belonging to
* SO(3) manifold. We estimate R using the Euler-Angle mapping between R^3 and SO(3)
* R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
*/
//-----------------------------------------------------------------------------
struct FrobeniusDistanceRotationAndTranslationCalibrationResidual
{
public:
  FrobeniusDistanceRotationAndTranslationCalibrationResidual(const Eigen::Matrix3d& argP1, const Eigen::Matrix3d& argP2,
                                                             const Eigen::Matrix3d& argQ1, const Eigen::Matrix3d& argQ2,
                                                             const Eigen::Vector3d& argV1, const Eigen::Vector3d& argV2,
                                                             const Eigen::Vector3d& argU1, const Eigen::Vector3d& argU2)
  {
    this->P1 = argP1; this->P2 = argP2;
    this->Q1 = argQ1; this->Q2 = argQ2;
    this->V1 = argV1; this->V2 = argV2;
    this->U1 = argU1; this->U2 = argU2;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> P1j, P2j, Q1j, Q2j;
    Eigen::Matrix<T, 3, 1> V1j, V2j, U1j, U2j;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        P1j(i, j) = T(this->P1(i, j));
        P2j(i, j) = T(this->P2(i, j));
        Q1j(i, j) = T(this->Q1(i, j));
        Q2j(i, j) = T(this->Q2(i, j));
      }
      V1j(i) = T(this->V1(i));
      V2j(i) = T(this->V2(i));
      U1j(i) = T(this->U1(i));
      U2j(i) = T(this->U2(i));
    }

    Eigen::Matrix<T, 3, 1> dX;
    dX << T(w[3]), T(w[4]), T(w[5]);

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R0;
    R0 << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
          cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
             -sry,               srx*cry,               crx*cry;

    Eigen::Matrix<T, 3, 1> T1 = R0.transpose() * (Q1j.transpose() * (Q2j * dX + (U2j - U1j)) - dX);
    Eigen::Matrix<T, 3, 1> T2 = P1j.transpose() * (V2j - V1j);
    T squaredResidual = ((T1 - T2).transpose() * (T1 - T2))(0);

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
  Eigen::Matrix3d P1, P2, Q1, Q2;
  Eigen::Vector3d V1, V2, U1, U2;
};

/**
* \class EuclideanDistanceAffineIsometryResidual
* \brief Cost function to minimize to estimate the rotation and translation
*        between two sensors based on their trajectory. To do that we minimize
*        the euclidean distance between matched points from the trajectory
*        according to the rotational and translational parameters. It results
*        an affine transform "best" fit the second trajectory upon the second
*
* Let's t in R be the current time
* Let's X be the position of the sensor 1 at time t
* Met's Y be the position of the sensor 2 at time t
*
* We want to estimate R0 in SO(3) and T0 in R^3 such that
* Y = RX + T
*
* This lead to a non-linear least square problem that can be solved using a
* levenberg-marquardt algorithm. The non-linearity comes from R belonging to
* SO(3) manifold. We estimate R using the Euler-Angle mapping between R^3 and SO(3)
* R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
*/
//-----------------------------------------------------------------------------
struct EuclideanDistanceAffineIsometryResidual
{
public:
  EuclideanDistanceAffineIsometryResidual(const Eigen::Vector3d& argX, const Eigen::Vector3d& argY)
  {
    this->X = argX;
    this->Y = argY;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 1> Xj, Yj;
    for (int i = 0; i < 3; ++i)
    {
      Xj(i) = T(this->X(i));
      Yj(i) = T(this->Y(i));
    }

    Eigen::Matrix<T, 3, 1> dX;
    dX << T(w[3]), T(w[4]), T(w[5]);

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R0;
    R0 << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
          cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
             -sry,               srx*cry,               crx*cry;

    Eigen::Matrix<T, 3, 1> L = R0 * Xj + dX - Yj;
    T squaredResidual = (L.transpose() * L)(0);

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
  Eigen::Vector3d X, Y;
};
}

#endif // CERES_COST_FUNCTIONS_H
