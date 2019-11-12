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

// LOCAL
#include "CameraProjection.h"
#include "vtkEigenTools.h"

// STD
#include <iostream>
#include <fstream>
#include <sstream>

// BOOST
#include <boost/algorithm/string.hpp>

//-----------------------------------------------------------------------------
Eigen::Vector3d GetRGBColourFromReflectivity(double v, double vmin, double vmax)
{
   Eigen::Vector3d c(1.0, 1.0, 1.0); // white
   double dv;
   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c[2] = 0;
      c[1] = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c[2] = 0;
      c[0] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c[2] = 4 * (v - vmin - 0.5 * dv) / dv;
      c[0] = 0;
   } else {
      c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c[0] = 0;
   }
   return 255.0 * c;
}

//----------------------------------------------------------------------------
void LoadCameraParamsFromCSV(std::string filename, Eigen::VectorXd& W)
{
  // Load file and check that the file is opened
  std::ifstream file(filename.c_str());
  if (!file.is_open())
  {
    std::cout << "Error: could not load file: " << filename << std::endl;
    return;
  }

  std::string line;
  std::getline(file, line);
  std::vector<std::string> values;
  boost::algorithm::split(values, line, boost::is_any_of(","));
  // initialize the parameters
  W = Eigen::VectorXd(values.size(), 1);
  for (int i = 0; i < values.size(); ++i)
  {
    W(i) = std::atof(values[i].c_str());
  }
  return;
}

//----------------------------------------------------------------------------
void WriteCameraParamsCSV(std::string filename, Eigen::VectorXd& W)
{
  // Load file and check that the file is opened
  std::ofstream file(filename.c_str());
  if (!file.is_open())
  {
    std::cout << "Error: could not open file: " << filename << std::endl;
    return;
  }
  for (int i = 0; i < W.size(); ++i)
  {
    file << W(i) << ",";
  }
  file.close();
  return;
}

//----------------------------------------------------------------------------
Eigen::Vector2d FisheyeProjection(const Eigen::Matrix<double, 15, 1>& W,
                                  const Eigen::Vector3d& X,
                                  bool shouldClip)
{
  // Get rotation matrix
  Eigen::Matrix3d R = RollPitchYawToMatrix(W(0), W(1), W(2));
  Eigen::Vector3d T(W(3), W(4), W(5));

  // Express the 3D point in the camera reference frame
  Eigen::Vector3d Xcam = R.transpose() * (X - T);

  // check that the point is not behind the camera plane
  if (shouldClip && (Xcam(2) < 0))
  {
    return Eigen::Vector2d(-1, -1);
  }

  // Project the 3D point in the plan
  Eigen::Vector2d Xp1(Xcam(0) / Xcam(2), Xcam(1) / Xcam(2));

  // Undistorded the projected image
  double r = Xp1.norm();
  double theta = std::atan(r);
  double thetad = theta * (1 + W(11) * std::pow(theta, 2) + W(12) * std::pow(theta, 4) +
                           W(13) * std::pow(theta, 6) + W(14) * std::pow(theta, 8));
   Eigen::Vector2d Xp1d = (thetad / r) * Xp1;

   // Create current intrinsic parameters
   Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
   K(0, 0) = W(6);
   K(1, 1) = W(7);
   K(0, 2) = W(8);
   K(1, 2) = W(9);
   K(0, 1) = W(10);
   K(2, 2) = 1;

   // Express the point in the pixel coordinates
   Eigen::Vector3d Xp1dh(Xp1d(0), Xp1d(1), 1);
   Eigen::Vector3d Xpix = K * Xp1dh;
   return Eigen::Vector2d(Xpix(0) / Xpix(2), Xpix(1) / Xpix(2));
}

//----------------------------------------------------------------------------
Eigen::Vector2d BrownConradyPinholeProjection(const Eigen::Matrix<double, 17, 1>& W,
                                              const Eigen::Vector3d& X,
                                              bool shouldClip)
{
  // Get rotation matrix
  Eigen::Matrix3d R = RollPitchYawToMatrix(W(0), W(1), W(2));
  Eigen::Vector3d T(W(3), W(4), W(5));

  // Express the 3D point in the camera reference frame
  Eigen::Vector3d Xcam = R.transpose() * (X - T);

  // check that the point is not behind the camera plane
  if (shouldClip && (Xcam(2) < 0))
  {
    return Eigen::Vector2d(-1, -1);
  }

  // Project the 3D point in the plan
  Eigen::Vector2d Xp1(Xcam(0) / Xcam(2), Xcam(1) / Xcam(2));

  // Undistorded the projected image
  double r = Xp1.norm();
  double k1 = W(11); double k2 = W(12);
  double p1 = W(13); double p2 = W(14);
  double p3 = W(15); double p4 = W(16);

  double xdist = Xp1(0) + Xp1(0) * (k1 * std::pow(r, 2) + k2 * std::pow(r, 4)) +
                 (p1 * (std::pow(r, 2) + 2 * std::pow(Xp1(0), 2)) +
                  2 * p2 * Xp1(0) * Xp1(1)) * (1 + p3 * std::pow(r, 2) + p4 * std::pow(r, 4));
  double ydist = Xp1(1) + Xp1(1) * (k1 * std::pow(r, 2) + k2 * std::pow(r, 4)) +
                 (2 * p1 * Xp1(0) * Xp1(1) + p2 * (std::pow(r, 2) + 2 * std::pow(Xp1(1), 2))) *
                 (1 + p3 * std::pow(r, 2) + p4 * std::pow(r, 4));
  Eigen::Vector2d Xp1d(xdist, ydist);

   // Create current intrinsic parameters
   Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
   K(0, 0) = W(6);
   K(1, 1) = W(7);
   K(0, 2) = W(8);
   K(1, 2) = W(9);
   K(0, 1) = W(10);
   K(2, 2) = 1;

   // Express the point in the pixel coordinates
   Eigen::Vector3d Xp1dh(Xp1d(0), Xp1d(1), 1);
   Eigen::Vector3d Xpix = K * Xp1dh;
   return Eigen::Vector2d(Xpix(0) / Xpix(2), Xpix(1) / Xpix(2));
}
