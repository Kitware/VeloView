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

#ifndef VTK_CAMERA_CALIBRATION_H
#define VTK_CAMERA_CALIBRATION_H

// STD
#include <string>
#include <vector>

// EIGEN
#include <Eigen/Dense>

void LoadCameraParamsFromCSV(std::string filename, Eigen::VectorXd& W);
void LoadMatchesFromCSV(std::string filename, std::vector<Eigen::Vector3d>& X, std::vector<Eigen::Vector2d>& x);

/**
   * @brief LinearPinholeCalibration Compute the pinhole camera model parameters
   *        using a linear model by directly computing the coefficients of the
   *        projection matrix. To do that we will estimate the coefficients of the
   *        projection matrix by minimizing the reprojection euclidean distance between
   *        the image keypoints and the reprojected 3D associated keypoints. This leads
   *        to a linear least square cost function that can be solved close form
   *
   *        Note that the set of the camera projection matrix is not a vector space
   *        but a manifold of M3,4(R). This mean that the linear method is not optimal
   *        Use NonLinearPinholeCalibration for an optimal solution using a parametrization
   *        of the camera projection matrix manifold.
   * @param X 3D keypoints associated to the 2D keypoints
   * @param x 2D keypoints associated to the 3D keypoints
   * @param P camera projection matrix estimated
   */
double LinearPinholeCalibration(const std::vector<Eigen::Vector3d>& X, const std::vector<Eigen::Vector2d>& x, Eigen::Matrix<double, 3, 4>& P);

/**
   * @brief NonLinearPinholeCalibration Compute the pinhole camera model parameters
   *        using a non-linear parametrization of the camera projection matrix manifold
   *        To do that we will estimate the parameters by minimizing the reprojection
   *        euclidean distance between the image keypoints and the reprojected 3D associated
   *        keypoints. This leads to a non-linear least square cost function that is solved
   *        using the iterative Levenberg-Marquardt Algorithm
   *
   *        Note that to converge, the method need to start with a first estimation that is not
   *        too far from the solution. One can first use the linear method using LinearPinholeCalibration
   *        to have a good estimation and then use the non linear method
   * @param X 3D keypoints associated to the 2D keypoints
   * @param x 2D keypoints associated to the 3D keypoints
   * @param P W pinhole camera model parameters estimated
   */
double NonLinearPinholeCalibration(const std::vector<Eigen::Vector3d>& X, const std::vector<Eigen::Vector2d>& x, Eigen::Matrix<double, 11, 1>& W);

/**
   * @brief NonLinearFisheyeCalibration Compute the fisheye camera model parameters
   *        using a non-linear parametrization of the camera projection matrix manifold
   *        To do that we will estimate the parameters by minimizing the reprojection
   *        euclidean distance between the image keypoints and the reprojected 3D associated
   *        keypoints. This leads to a non-linear least square cost function that is solved
   *        using the iterative Levenberg-Marquardt Algorithm
   *
   *        Note that to converge, the method need to start with a first estimation that is not
   *        too far from the solution. One can first use the linear method using LinearPinholeCalibration
   *        to have a good estimation and then use the non linear method
   * @param X 3D keypoints associated to the 2D keypoints
   * @param x 2D keypoints associated to the 3D keypoints
   * @param P W fisheye camera model parameters estimated
   */
double NonLinearFisheyeCalibration(const std::vector<Eigen::Vector3d>& X, const std::vector<Eigen::Vector2d>& x, Eigen::Matrix<double, 15, 1>& W);

/**
   * @brief CalibrationMatrixDecomposition Decompose the pinhole camera model
   *        projection matrix to recover the pinhole camera model parameters
   *
   * @param P Camera projection matrix
   * @param K matrix of intrisic parameters
   * @param R orientation of the camera (extrinsic parameters)
   * @param T position of the camera (extrinsic parameters)
   */
void CalibrationMatrixDecomposition(const Eigen::Matrix<double, 3, 4>& P, Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& T);

/**
   * @brief CalibrationMatrixDecomposition Decompose the pinhole camera model
   *        projection matrix to recover the pinhole camera model parameters
   *
   * @param K matrix of intrisic parameters
   * @param R orientation of the camera (extrinsic parameters)
   * @param T position of the camera (extrinsic parameters)
   * @param W pinhole model parameters
   */
void GetParametersFromMatrix(const Eigen::Matrix3d& K, const Eigen::Matrix3d& R, const Eigen::Vector3d& T, Eigen::Matrix<double, 11, 1>& W);
void GetMatrixFromParameters(const Eigen::Matrix<double, 11, 1>& W, Eigen::Matrix<double, 3, 4>& P);
Eigen::Matrix<double, 3, 4> GetMatrixFromParameters(const Eigen::Matrix<double, 11, 1>& W);

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

#endif // VTK_CAMERA_CALIBRATION_H
