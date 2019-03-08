//=========================================================================
//
// Copyright 2018 Kitware, Inc.
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
#include <algorithm>
#include <random>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <vtkMath.h>

// LOCAL
#include "vtkEigenTools.h"

//-----------------------------------------------------------------------------
Eigen::Quaterniond AvgUnitQuaternions(const std::vector<Eigen::Quaterniond>& Q)
{
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  for (size_t i = 0; i < Q.size(); i++)
  {
    Eigen::Vector4d q = Eigen::Vector4d(Q[i].w(), Q[i].x(), Q[i].y(), Q[i].z());
    A = A + q * q.transpose();
  }

  A = (1.0 / Q.size()) * A;
  Eigen::EigenSolver<Eigen::Matrix4d> eig(A);

  if ((eig.eigenvectors()(0,0).real() == eig.eigenvectors()(0,0)) &&
      (eig.eigenvectors()(1,0).real() == eig.eigenvectors()(1,0)) &&
      (eig.eigenvectors()(2,0).real() == eig.eigenvectors()(2,0)) &&
      (eig.eigenvectors()(3,0).real() == eig.eigenvectors()(3,0)))
  {
    return Eigen::Quaterniond(eig.eigenvectors()(0,0).real(),
                              eig.eigenvectors()(1,0).real(),
                              eig.eigenvectors()(2,0).real(),
                              eig.eigenvectors()(3,0).real());
  }
  else
  {
    std::cerr << "eigen decomposition has complex eigen values" << std::endl;
    // return the identity matrix
    return Eigen::Quaterniond(0, 0, 0, 0);
  }
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d AvgRotation(const std::vector<Eigen::Matrix3d>& rotations)
{
  std::vector<Eigen::Quaterniond> Q = std::vector<Eigen::Quaterniond>(rotations.size());
  for (size_t i = 0; i < rotations.size(); i++)
  {
    Q[i] = Eigen::Quaterniond(rotations[i]);
  }
  Eigen::Quaterniond avg = AvgUnitQuaternions(Q);
  return avg.normalized().toRotationMatrix();
}

//-----------------------------------------------------------------------------
Eigen::Vector3d MatrixToRollPitchYaw(const Eigen::Matrix3d& rotation)
{
  Eigen::Vector3d eulerAngles;
  eulerAngles(0) = std::atan2(rotation(2, 1), rotation(2, 2));
  eulerAngles(1) = -std::asin(rotation(2, 0));
  eulerAngles(2) = std::atan2(rotation(1, 0), rotation(0, 0));

  return eulerAngles;
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d RollPitchYawToMatrix(double roll, double pitch, double yaw)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d RollPitchYawToMatrix(const Eigen::Vector3d& angles)
{
  return RollPitchYawToMatrix(angles(0), angles(1), angles(2));
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d RollPitchYawInDegreeToMatrix(double roll, double pitch, double yaw)
{
  return RollPitchYawToMatrix((vtkMath::Pi() / 180.0) * roll,
                              (vtkMath::Pi() / 180.0) * pitch,
                              (vtkMath::Pi() / 180.0) * yaw);
}

//-----------------------------------------------------------------------------
double SignedAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  // if one of the vector is the null vector return 0
  if ((v1.norm() < std::numeric_limits<double>::epsilon()) ||
      (v2.norm() < std::numeric_limits<double>::epsilon()))
  {
    return 0.0;
  }

  // Check if the vectors are colinear
  Eigen::Vector3d orthogonal = v1.cross(v2);
  if (orthogonal.norm() < std::numeric_limits<double>::epsilon())
  {
    double lambda = v1.sum() / v2.sum();
    if (lambda > 0)
    {
      return 0.0;
    }
    if (lambda < 0)
    {
      return vtkMath::Pi();
    }
  }
  orthogonal.normalize();

  double scaleCosinusAngle = v1.dot(v2);
  double scaleSinusAngle = (v1.cross(v2)).dot(orthogonal);
  return std::atan2(scaleSinusAngle, scaleCosinusAngle);
}

//-----------------------------------------------------------------------------
bool IsMatrixFinite(const Eigen::Matrix3d& M)
{
  return ((M - M).array() == (M - M).array()).all();
}

//----------------------------------------------------------------------------
Eigen::Vector3d GetSphericalCoordinates(const Eigen::Vector3d& X)
{
  Eigen::Vector3d Y = X;
  // If the norm of the vector is
  // not too small we normalized it
  double r = Y.norm();
  if (r > std::numeric_limits<double>::epsilon())
  {
    Y.normalize();
  }
  else
  {
    return Eigen::Vector3d(0, 0, 0);
  }

  // Project Y onto the (ex, ey) plane and norm it
  Eigen::Vector3d projY(Y(0), Y(1), 0);
  if (projY.norm() < std::numeric_limits<double>::epsilon())
  {
    return Eigen::Vector3d(r, 0, (Y(2) >= 0) ? 0.0 : vtkMath::Pi());
  }

  // Compute Phi angle
  double scaledCosinePhi = Y.dot(Eigen::Vector3d::UnitZ());
  double scaledSinusPhi = Y.dot(projY.normalized());
  double phi = std::atan2(scaledSinusPhi, scaledCosinePhi);

  // Compute theta angle
  double scaledCosineTheta = projY.dot(Eigen::Vector3d::UnitX());
  double scaledSinusTheta = projY.dot(Eigen::Vector3d::UnitY());
  double theta = std::atan2(scaledSinusTheta, scaledCosineTheta);

  Eigen::Vector3d sphericalCoords(r, theta, phi);
  return sphericalCoords;
}

//----------------------------------------------------------------------------
Eigen::Matrix<double, 3, 1> GetSphericalCoordinates(const Eigen::Vector3d& X,
                                                    const Eigen::Matrix3d& Basis,
                                                    const Eigen::Vector3d& Origin)
{
  // Express the vector in the new reference frame
  Eigen::Vector3d Y = Basis.transpose() * (X - Origin);
  return GetSphericalCoordinates(Y);
}

//----------------------------------------------------------------------------
Eigen::Matrix3d ComputeHomography(const std::vector<Eigen::Vector2d>& x,
                                  const std::vector<Eigen::Vector2d>& y)
{
  if (x.size() != y.size())
  {
    vtkGenericWarningMacro("The input and output list of vectors must be the same size");
    return Eigen::Matrix3d::Identity();
  }

  // Compute the direct linear transform matrix
  Eigen::MatrixXd A(2 * x.size(), 9);
  for (unsigned int i = 0; i < x.size(); ++i)
  {
    A.row(2 * i) << x[i](0), x[i](1), 1,
                    0, 0, 0,
                    -y[i](0) * x[i](0), -y[i](0) * x[i](1), -y[i](0);

    A.row(2 * i + 1) << 0, 0, 0,
                    x[i](0), x[i](1), 1,
                    -y[i](1) * x[i](0), -y[i](1) * x[i](1), -y[i](1);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd flattenH = svd.matrixV().col(8);
  Eigen::Matrix3d H;
  H << flattenH(0), flattenH(1), flattenH(2),
       flattenH(3), flattenH(4), flattenH(5),
       flattenH(6), flattenH(7), flattenH(8);
  return H;
}
