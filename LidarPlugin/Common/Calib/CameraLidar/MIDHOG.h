//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (pierre.guilbert@kitware.com)
// Date: 2019-07-03
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

// this algorithm is originally inspired by:
// "Fine Scale Image Registration in large-scale urban LIDAR point sets",
// Maximilien Guislain, Julie Digne, Raphaêlle Chaine and Gilles Monnier.
//
// It is an extension of the 2D-features method to automatically calibrate
// a camera within a large-scale dense point-cloud. It is an iterative algorithm
// that aims to estimate the geometric and optical calibration of a camera using
// a two step method. For each optimization step:
//
// - Synthetic Image rendering: The pointcloud is projected to create an image
//   according to the current estimation of the camera model (pose and optic).
//   The synthetic image also aims to handle occultation and data filling when
//   the point cloud is projected
//
// - Optimization: For the current synthetic image rendered, estimate the best
//   optical and geometric parameters that minimize the objective / cost function
//   The objective function is defined as a linear interpolation between the mutual
//   information between the real image and the synthetic one and a DHOG function.
//
// Note that many basic image operators (median filter, gradient, ...) are reimplemented
// for two reasons:
// - They are specifically designed for the synthetic images created (taking into account missing data, ...)
// - They can be templated using ceres::Jet type and thus, be fully differentiable

#ifndef MIDHOG_CALIBRATION_H
#define MIDHOG_CALIBRATION_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OPENCV
#include <opencv2/core.hpp>

class MIDHOGCalibration
{
public:
  MIDHOGCalibration() = default;
  ~MIDHOGCalibration() = default;

  MIDHOGCalibration(const cv::Mat& image,
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                    const Eigen::Matrix<double, 17, 1>& cameraParams,
                    int neighborRadius);

  // Getter / Setter
  cv::Mat GetSyntheticImage(int idx);

  // Project the pointcloud to create an synthetic image using the
  // current estimation of the camera geometric and optic parameters
  void CreateSyntheticImage();

protected:

  // Real image acquired from the camera we aim to calibrate.
  // If the system LiDAR - Camera is mobile, the estimated pose
  // of the camera will correspond to the pose of the camera at which
  // this image have been acquired according to the reference frame in
  // which the 3D points are expressed
  cv::Mat Image;

  // Synthetic image corresponding of the point cloud projected using
  // the current camera parameters estimation
  std::vector<cv::Mat> SyntheticImage;

  // Radius of the neighbor (in pixel) used to check the visibility
  // of the 3D point once projected on the image. To compute if a point
  // is visible we will estimate its solid angle using the points that
  // have been projected on the 2D-neighborhood
  int NeighborRadius = 7;

  // Radius of the neigborhood (in pixel) used to compute the image pixel
  // value interpolation. The interpolation will be a mixed of linear and
  // quadratic interpolation depending on the data avilable on the neighborhood
  int NeighborRadiusInterpolation = 6;

  // Radius of the neigborhood (in pixel) used to compute the median filter
  int NeighborRadiusMedianFilter = 1;

  // Point-cloud corresponding to the geometry observed by the image. The estimated
  // pose of the camera will be expressed according to the reference frame in which
  // the points are expressed. If one aims to have the relative pose of the camera
  // and the Lidar, the 3D points should be expressed in the reference frame attached
  // to the lidar at the time of acquisition of the image.
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr Cloud;

  // 17-dof parameters of the camera according to the brown conrady camera model
  // 3-dof for the orientation using euler angle parametrization of SO(3)
  // 3-dof for the position of the camera
  // 5-dof corresponding to the intrinsic parameters (focal, skew, pixel size, ...)
  // 6-dof corresponding to the optical distortions (tangential and radial)
  Eigen::Matrix<double, 17, 1> CameraParams = Eigen::Matrix<double, 17, 1>::Zero();

  // Estimate the normals of the points using an PCA
  // approach.
  void ComputeCloudNormals();
};

#endif // MIDHOG_CALIBRATION_H
