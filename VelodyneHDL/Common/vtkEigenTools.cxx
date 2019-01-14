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
  for (int i = 0; i < Q.size(); i++)
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
  for (int i = 0; i < rotations.size(); i++)
  {
    Q[i] = Eigen::Quaterniond(rotations[i]);
  }
  Eigen::Quaterniond avg = AvgUnitQuaternions(Q);
  return avg.normalized().toRotationMatrix();
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d RollPitchYawToMatrix(double roll, double pitch, double yaw)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
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
  Eigen::Vector3d nv1 = v1.normalized();
  Eigen::Vector3d nv2 = v2.normalized();
  double sinus = v1.cross(v2).norm();
  double cosinus = v1.dot(v2);
  return std::atan2(sinus, cosinus);
}

//-----------------------------------------------------------------------------
bool IsMatrixFinite(const Eigen::Matrix3d& M)
{
  return ((M - M).array() == (M - M).array()).all();
}