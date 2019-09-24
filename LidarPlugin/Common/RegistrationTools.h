//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Date: 05-29-2019
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

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
// EIGEN
#include <Eigen/Dense>

/**
 * @brief ICPPointToPlaneRegistration From a given source pointcloud and a first estimation
 *        pose of this pointcloud; refine the pose estimation by registering the
 *        source point cloud on the reference pointcloud
 *
 *        The registration will be performed using an ICP algorithm with a
 *        point to surface distance
 *
 * @param reference reference point cloud
 * @param toAligned point cloud we want to align on the reference
 * @param H initial guess of the pose of toAligned point cloud
 * @param maxICPIteration maximum number of ICP iteration
 * @param maxLMIteration maximum number of Levenberg-Marquardt iteration
 *        per ICP iteration
 */
Eigen::Matrix4d ICPPointToPlaneRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr reference,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr toAligned,
                                            const Eigen::Matrix4d& H,
                                            std::vector<bool>& pointUsed,
                                            unsigned int maxICPIteration = 4,
                                            unsigned int maxLMIteration = 15);

/**
 * @brief FindBestPlaneParameters Compute parameters of the plane that best
 *        approximate the neighborhood of a point according to the squared
 *        euclidean distance.
 *
 *        The parameters are provided as a symmetric matrix and a point belonging
 *        to the plane so that:
 *        d(X, Plane) = (X - P).t * SemiDist * (X - P)
 *
 * @param query point whose neighborhood must be approximated
 * @param knearest number of point selected in query neighborhood
 * @param maxDist maximum distance allowed for a point to be consider in the neighborhood
 * @param minPlanarity minimum planarity score to consider the neighborhood as a plane
 * @param kdtree kdtree used to extract knearest points
 * @param SemiDist symmetric matrix encoding the point-to-plane distance
 * @param P point belonging to the plane
 * @param planarity of the neighborhood
 */
bool FindBestPlaneParameters(pcl::PointXYZ query, unsigned int knearest, double maxDist,
                             double  minPlanarity, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree,
                             Eigen::Matrix3d& SemiDist, Eigen::Vector3d& P, double& planarity);

/**
 * @brief ComputeSimilitude Compute the similitude that matches Xi to Yi
 *
 * @param X set of input points
 * @param Y set of output points
 * @param W parameters of the similitude [a0, a1, a2, t0, t1, t2, s0, s1, s2]
 *        with ai being euler-angle SO(3) manifold parametrization. We chose the
 *        following Euler-Angle mapping between R^3 and SO(3)
 *        R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
 *        ti being the translation and si the scale factors
 */
void ComputeSimilitude(const std::vector<Eigen::Vector3d>& X,
                       const std::vector<Eigen::Vector3d>& Y,
                       Eigen::Matrix<double, 9, 1>& W);
