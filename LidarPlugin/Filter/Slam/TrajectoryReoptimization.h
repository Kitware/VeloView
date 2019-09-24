//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Date: 05-06-2019
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

#ifndef TRAJECTORY_REOPTIMIZATION_H
#define TRAJECTORY_REOPTIMIZATION_H

// EIGEN
#include <Eigen/Dense>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// VTK
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

/**
* \enum MeasureProvided
* \brief Indicates to the algorithm if the data provided
*        contains: position only, orientation only or
*        position and orientation
*/
enum class MeasureProvided
{
  PositionOnly = 1,
  OrientationOnly = 2,
  OrientationPosition = 3
};

/**
* \class PoseEstimation
* \brief Represent the relative or absolute pose estimation
*        of a sensor for a time of acquisition. The pose estimation
*        is provided with the Variance - Covariance matrix that
*        estimated the estimation error regarding the 6-DoF parameters
*/
class PoseEstimation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> Sigma = Eigen::Matrix<double, 6, 6>::Identity();
  int PlanarNbr = 0;
  int EdgesNbr = 0;
  double MahalanobisDistance = 0;
  double AnglesDistance = 0;
  double PositionDistance = 0;
};
using PoseEstimationVector = std::vector<PoseEstimation, Eigen::aligned_allocator<PoseEstimation>>;

/**
 * @brief TrajectoryReoptimization Reoptimization of a poses-trajectory
 *        (both positions and orientations) estimated by the SLAM algorithm
 *        using an external groundtruth landmark (GPS / IMU, ....)
 *        Here, the estimated map will not be directly used to reoptimize
 *        the trajectory. For each pose estimated, we will compute the local
 *        (i.e relative) transform to apply so that the end pose will
 *        match the groundtruth pose. We will also use the Variance-Covariance
 *        matrix associated to pose parameters estimated during SLAM to add a
 *        data attached term:
 *
 *        - We denote Hg the ground truth end pose
 *        - We denote Hi the local (i.e relatively to referencial of frame i - 1)
 *          pose estimated by the SLAM for the frame i
 *        - We denote dHi(w) the local pose correction associated to the pose Hi
 *        - We denote Hi(w) = dHi(w) * Hi the corrected pose for the estimated parameters w
 *
 *        Finally, we want w in R^(6n) where n is the number of poses so that:
 *        d(product(Hi(w)), Hg)^2 + lambda * w.t * Sigma w is minimal
 *        with:
 *        d being a distance upon the poses set and lambda an hyper-parameter
 *
 * @param absolutePoses the absolute poses estimations we want to reoptimize
 * @param H 4x4 matrix representing the groundtruth pose associated to frame index i1
 * @param gtIndex frame index corresponding to the ground truth data
 * @param maxIteration maximum number of iteration for the optimization
 */
PoseEstimationVector TrajectoryReoptimization(const PoseEstimationVector& absolutePoses,
                                              const Eigen::Matrix4d& H, int gtIndex, int maxIteration = 30,
                                              MeasureProvided dataMode = MeasureProvided::OrientationPosition);
PoseEstimationVector TrajectoryReoptimization(const PoseEstimationVector& absolutePoses,
                                              const std::vector<Eigen::Matrix4d>& H, std::vector<int> gtIndex,
                                              int maxIteration = 30,
                                              MeasureProvided dataMode = MeasureProvided::OrientationPosition);

/**
 * @brief RelativePosesFromAbsolutePoses Compute relative sensor poses estimation
 *        from the absolute poses. The relative pose consist of the pose of the sensor
 *        at the time of acquisition tk according to the reference frame attached to the
 *        sensor at the time tk-1.
 *        Hence, we can compute:
 *        dR = R(tk-1).transpose() * R(tk)
 *        dT = R(tk-1).transpose() * (T(tk) - T(tk-1))
 *        dSigma = J * Sigma * J.transpose()
 *
 *        where J is the jacobian of the mapping between the absolute 6-dof and
 *        the relative 6-dof evaluated at the current value of the absolute 6-dof
 *        parameters
 *
 * @param absolutePoses the absolute poses estimations we want to expressed in a relative way
 */
PoseEstimationVector RelativePosesFromAbsolutePoses(const PoseEstimationVector& absolutePoses);

/// Convert poses stored into PoseEstimation data structure into polydata
vtkSmartPointer<vtkPolyData> ConvertPoseEstimationToPolyData(const PoseEstimationVector& poses);

/// Convert a polydata into a PoseEstimation data structure
void ConvertPolyDataToPoseEstimation(vtkSmartPointer<vtkPolyData> trajectory,
                                     PoseEstimationVector& poses);

#endif // TRAJECTORY_REOPTIMIZATION_H
