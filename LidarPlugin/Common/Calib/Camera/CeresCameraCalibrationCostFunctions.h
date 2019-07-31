//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Data: 03-27-2019
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

#ifndef CERES_CAMERA_CALIBRATION_COST_FUNCTIONS_H
#define CERES_CAMERA_CALIBRATION_COST_FUNCTIONS_H

// EIGEN
#include <Eigen/Dense>

// CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>

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
struct PinholeModelAlgebraicDistance
{
public:
  PinholeModelAlgebraicDistance(const Eigen::Vector3d& argX,
                                const Eigen::Vector2d& argx)
  {
    this->X = argX;
    this->x = argx;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 4, 1> Xc(T(this->X(0)), T(this->X(1)), T(this->X(2)), T(1));
    Eigen::Matrix<T, 2, 1> xc(T(this->x(0)), T(this->x(1)));

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R;
    R << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
         cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
            -sry,               srx*cry,               crx*cry;

    // Create current position
    Eigen::Matrix<T, 3, 1> C(w[3], w[4], w[5]);

    // Create current intrinsic parameters
    Eigen::Matrix<T, 3, 3> K = Eigen::Matrix<T, 3, 3>::Zero();
    K(0, 0) = T(w[6]);
    K(1, 1) = T(w[7]);
    K(0, 2) = T(w[8]);
    K(1, 2) = T(w[9]);
    K(0, 1) = T(w[10]);
    K(2, 2) = T(1);

    // Create current calibration matrix
    Eigen::Matrix<T, 3, 4> P;
    P.block(0, 0, 3, 3) = K * R.transpose();
    P.col(3) = -K * R.transpose() * C;
    P = P / P(2, 3);

    // Project the 3d point using the current P estimation
    Eigen::Matrix<T, 3, 1> Xproj = P * Xc;
    Eigen::Matrix<T, 2, 1> yc(Xproj(0) / Xproj(2), Xproj(1) / Xproj(2));

    // Compute final residual value which is:
    // d(xc, P*XC)
    Eigen::Matrix<T, 2, 1> e = (yc - xc);
    T squaredResidual = (e.transpose() * e)(0);

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
  Eigen::Vector3d X;
  Eigen::Vector2d x;
};

/**
* \class MahalanobisDistanceAffineIsometryResidual
* \brief Cost function to minimize to estimate the affine isometry transformation
*        (rotation and translation) that minimizes the mahalanobis distance
*        between a point X and its neighborhood encoded by the mean point C
*        and the variance covariance matrix A
*/
//-----------------------------------------------------------------------------
struct FisheyeModelAlgebraicDistance
{
public:
  FisheyeModelAlgebraicDistance(const Eigen::Vector3d& argX,
                                const Eigen::Vector2d& argx)
  {
    this->X = argX;
    this->x = argx;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 1> Xc(T(this->X(0)), T(this->X(1)), T(this->X(2)));
    Eigen::Matrix<T, 2, 1> xc(T(this->x(0)), T(this->x(1)));

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R;
    R << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
         cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
            -sry,               srx*cry,               crx*cry;

    // Create current position
    Eigen::Matrix<T, 3, 1> C(w[3], w[4], w[5]);

    // Create current intrinsic parameters
    Eigen::Matrix<T, 3, 3> K = Eigen::Matrix<T, 3, 3>::Zero();
    K(0, 0) = T(w[6]);
    K(1, 1) = T(w[7]);
    K(0, 2) = T(w[8]);
    K(1, 2) = T(w[9]);
    K(0, 1) = T(w[10]);
    K(2, 2) = T(1);

    // First, express the 3D point in the camera
    // reference frame
    Eigen::Matrix<T, 3, 1> Xcam = R.transpose() * (Xc - C);

    // Then, project the 3D point on the plane
    // that is distant from the camera origin
    // of 1 unit
    Eigen::Matrix<T, 2, 1> Xp1(Xcam(0) / Xcam(2), Xcam(1) / Xcam(2));

    // Compute the incident angle
    T r = Xp1.norm();
    T theta = ceres::atan(r);

    // Compute the undistorded incident angle
    T thetad = theta * (T(1) + w[11] * ceres::pow(theta, 2) + w[12] * ceres::pow(theta, 4) +
                        w[13] * ceres::pow(theta, 6) + w[14] * ceres::pow(theta, 8));

    // Compute the undistorded plan coordinates
    Eigen::Matrix<T, 2, 1> Xp1d = (thetad / r) * Xp1;

    // Express the point in the pixel coordinates
    Eigen::Matrix<T, 3, 1> Xp1dh(Xp1d(0), Xp1d(1), T(1));
    Eigen::Matrix<T, 3, 1> Xpix = K * Xp1dh;

    // Compute final residual value which is:
    // d(xc, P*XC)
    Eigen::Matrix<T, 2, 1> yc(Xpix(0) / Xpix(2), Xpix(1) / Xpix(2));
    Eigen::Matrix<T, 2, 1> e = (yc - xc);
    T squaredResidual = (e.transpose() * e)(0);

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
  Eigen::Vector3d X;
  Eigen::Vector2d x;
};

/**
* \class MahalanobisDistanceAffineIsometryResidual
* \brief Cost function to minimize to estimate the affine isometry transformation
*        (rotation and translation) that minimizes the mahalanobis distance
*        between a point X and its neighborhood encoded by the mean point C
*        and the variance covariance matrix A
*/
//-----------------------------------------------------------------------------
struct BrownConradyAlgebraicDistance
{
public:
  BrownConradyAlgebraicDistance(const Eigen::Vector3d& argX,
                                const Eigen::Vector2d& argx)
  {
    this->X = argX;
    this->x = argx;
  }

  void SetActivatedParams(const std::vector<bool>& argActivatedParams)
  {
    this->ActivatedParams = argActivatedParams;
  }

  void SetW0(Eigen::VectorXd argW0)
  {
    this->W0 = argW0;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Copy w so that we will be able to set to zero the
    // infinitesimal part of a parameter which will have
    // the effect to disable the optimization according
    // to this parameter
    std::vector<T> wcopy(17);
    for (int i = 0; i < 17; ++i)
    {
      wcopy[i] = w[i];
    }

    // Check which parameters should not be optimized
    if (this->ActivatedParams.size() == 17)
    {
      for (int i = 0; i < 17; ++i)
      {
        if (!this->ActivatedParams[i])
        {
          wcopy[i] = T(this->W0(i));
        }
      }
    }

    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 1> Xc(T(this->X(0)), T(this->X(1)), T(this->X(2)));
    Eigen::Matrix<T, 2, 1> xc(T(this->x(0)), T(this->x(1)));

    // store sin / cos values for this angle
    T crx = ceres::cos(wcopy[0]); T srx = ceres::sin(wcopy[0]);
    T cry = ceres::cos(wcopy[1]); T sry = ceres::sin(wcopy[1]);
    T crz = ceres::cos(wcopy[2]); T srz = ceres::sin(wcopy[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R;
    R << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
         cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
            -sry,               srx*cry,               crx*cry;

    // Create current position
    Eigen::Matrix<T, 3, 1> C(wcopy[3], wcopy[4], wcopy[5]);

    // Create current intrinsic parameters
    Eigen::Matrix<T, 3, 3> K = Eigen::Matrix<T, 3, 3>::Zero();
    K(0, 0) = T(wcopy[6]);
    K(1, 1) = T(wcopy[7]);
    K(0, 2) = T(wcopy[8]);
    K(1, 2) = T(wcopy[9]);
    K(0, 1) = T(wcopy[10]);
    K(2, 2) = T(1);

    // First, express the 3D point in the camera
    // reference frame
    Eigen::Matrix<T, 3, 1> Xcam = R.transpose() * (Xc - C);

    // Then, project the 3D point on the plane
    // that is distant from the camera origin
    // of 1 unit
    Eigen::Matrix<T, 2, 1> Xp1(Xcam(0) / Xcam(2), Xcam(1) / Xcam(2));

    // Undistorded the projected image
    T r = Xp1.norm();
    T k1 = wcopy[11]; T k2 = wcopy[12];
    T p1 = wcopy[13]; T p2 = wcopy[14];
    T p3 = wcopy[15]; T p4 = wcopy[16];

    T xdist = Xp1(0) + Xp1(0) * (k1 * ceres::pow(r, 2) + k2 * ceres::pow(r, 4)) +
                   (p1 * (ceres::pow(r, 2) + T(2) * ceres::pow(Xp1(0), 2)) +
                    T(2) * p2 * Xp1(0) * Xp1(1)) * (T(1) + p3 * ceres::pow(r, 2) + p4 * ceres::pow(r, 4));
    T ydist = Xp1(1) + Xp1(1) * (k1 * ceres::pow(r, 2) + k2 * ceres::pow(r, 4)) +
                   (T(2) * p1 * Xp1(0) * Xp1(1) + p2 * (ceres::pow(r, 2) + T(2) * ceres::pow(Xp1(1), 2))) *
                   (T(1) + p3 * ceres::pow(r, 2) + p4 * ceres::pow(r, 4));
    Eigen::Matrix<T, 2, 1> Xp1d(xdist, ydist);

    // Express the point in the pixel coordinates
    Eigen::Matrix<T, 3, 1> Xp1dh(Xp1d(0), Xp1d(1), T(1));
    Eigen::Matrix<T, 3, 1> Xpix = K * Xp1dh;

    // Compute final residual value which is:
    // d(xc, P*XC)
    Eigen::Matrix<T, 2, 1> yc(Xpix(0) / Xpix(2), Xpix(1) / Xpix(2));
    Eigen::Matrix<T, 2, 1> e = (yc - xc);
    T squaredResidual = (e.transpose() * e)(0);

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
  Eigen::Vector3d X;
  Eigen::Vector2d x;
  std::vector<bool> ActivatedParams = std::vector<bool>(0);
  Eigen::VectorXd W0;
};
}

#endif // CERES_CAMERA_CALIBRATION_COST_FUNCTIONS_H
