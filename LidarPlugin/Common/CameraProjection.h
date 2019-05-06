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

#ifndef CAMERA_PROJECTION_H
#define CAMERA_PROJECTION_H

// STD
#include <string>
#include <vector>

// EIGEN
#include <Eigen/Dense>

/**
   * @brief LoadCameraParamsFromCSV Load parameters from a csv file
   *
   * @param filename filename that contains the parameters
   * @param W loaded parameters
   */
void LoadCameraParamsFromCSV(std::string filename, Eigen::VectorXd& W);

/**
   * @brief WriteCameraParamsCSV Write parameters into a csv file
   *
   * @param filename filename to write the camera parameters
   * @param W to write parameters
   */
void WriteCameraParamsCSV(std::string filename, Eigen::VectorXd& W);

/**
   * @brief FisheyeProjection Project a 3D point using a fisheye camera model
   *        the projected 2D points will be expressed in pixel coordinates
   *
   * @param W fisheye camera model parameters
   * @param X 3D point to project
   * @param T position of the camera (extrinsic parameters)
   * @param W pinhole model parameters
   */
Eigen::Vector2d FisheyeProjection(const Eigen::Matrix<double, 15, 1>& W, const Eigen::Vector3d& X);

/**
   * @brief GetRGBColourFromReflectivity map the reflectivity signal
   *        onto a RGB color map
   *
   * @param v reflectivity signal
   * @param vmin minimal value of the reflectivity signal
   * @param vmax maximal value of the reflectivity signal
   */
Eigen::Vector3d GetRGBColourFromReflectivity(double v, double vmin, double vmax);

#endif // CAMERA_PROJECTION_H
