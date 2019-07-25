//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (pierre.guilbert@kitware.com)
// Data: 07-30-2019
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

#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

// STD
#include <string>
#include <vector>

// EIGEN
#include <Eigen/Dense>

enum ProjectionType
{
  Pinhole = 0,
  BrownConradyPinhole = 1,
  FishEye = 2
};

/**
   * @brief CameraModel class that represents a camera model included
   *        by the following modelization:
   *        1- Extrinsic parameters, element of SE(3)
   *        2- Intrinsic parameters, representing the focal and pixel grid
   *        3- Optical parameters, representing the optical system distortions
   */
class CameraModel
{
public:
  CameraModel() = default;

  // Setters
  void SetParams(Eigen::VectorXd argW);
  void SetK(Eigen::Matrix3d argK);
  void SetR(Eigen::Matrix3d argR);
  void SetT(Eigen::Vector3d argT);
  void SetOptics(Eigen::VectorXd argOptics);
  void SetCameraModelType(ProjectionType type);

  // Getters
  Eigen::VectorXd GetParametersVector();
  Eigen::Matrix3d GetK();
  Eigen::Matrix3d GetR();
  Eigen::Vector3d GetT();
  Eigen::VectorXd GetOptics();
  ProjectionType GetType();

  void LoadParamsFromFile(std::string filename);
  static void WriteParamsToFile(std::string outFilename, Eigen::VectorXd Win, ProjectionType typein);

protected:
  //! intrinsic parameters of the camera model
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();

  //! extrinsic parameters of the camera model
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();

  //! optical parameters of the camera
  Eigen::VectorXd Optics;

  //! type of camera model
  ProjectionType Type = ProjectionType::Pinhole;
};

#endif // CAMERA_MODEL_H
