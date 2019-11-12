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
   * @param shouldClip Clip points that are behind the camera plane
   */
Eigen::Vector2d FisheyeProjection(const Eigen::Matrix<double, 15, 1>& W,
                                  const Eigen::Vector3d& X,
                                  bool shouldClip = false);

/**
   * @brief BrownConradyPinholeProjection Project a 3D point using a pinhole
   *        camera model with Brown-Conrady camera distortion model.
   *        the projected 2D points will be expressed in pixel coordinates
   *
   * @param W pinhole Brown-Conrady camera model parameters
   * @param X 3D point to project
   * @param shouldPlaneClip Clip points that are behind the camera plane
   * @param shouldFoVClip Clip points that are not in the FoV of the camera
   *        this is usefull since high distortion parameters can introduce
   *        non injective behaviour between 3D direction and 2D pixels
   * @param fovAngle angle of the field of view
   */
Eigen::Vector2d BrownConradyPinholeProjection(const Eigen::Matrix<double, 17, 1>& W,
                                              const Eigen::Vector3d& X,
                                              bool shouldPlaneClip = false);

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
