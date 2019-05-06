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

// LOCAL
#include "RegistrationTools.h"
#include "vtkEigenTools.h"
#include "CeresCostFunctions.h"
// STD
#include <iostream>
// PCL
#include <pcl/kdtree/kdtree_flann.h>
// CERES
#include <ceres/ceres.h>

//-----------------------------------------------------------------------------
Eigen::Matrix4d ICPPointToPlaneRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr reference,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr toAligned,
                                            const Eigen::Matrix4d& H,
                                            std::vector<bool>& pointUsed,
                                            unsigned int maxICPIteration,
                                            unsigned int maxLMIteration)
{
  int requiredNearest = 9;
  pointUsed = std::vector<bool>(toAligned->size(), false);
  // Build kdtree to perform fast knearest neighbor
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeReference(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeToAligned(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtreeReference->setInputCloud(reference);
  kdtreeToAligned->setInputCloud(toAligned);

  // Current pose estimation
  Eigen::Matrix<double, 6, 1> DoF6Params;
  DoF6Params.block(0, 0, 3, 1) = MatrixToRollPitchYaw(H.block(0, 0, 3, 3));
  DoF6Params.block(3, 0, 3, 1) = H.block(0, 3, 3, 1);

  std::vector<double> maxDist;
  maxDist.push_back(5);
  maxDist.push_back(4.5);
  maxDist.push_back(4);
  maxDist.push_back(3.5);
  maxDist.push_back(3);
  maxDist.push_back(2.5);
  maxDist.push_back(2);
  maxDist.push_back(1.5);
  maxDist.push_back(1);

  // loop over ICP iteration
  for (unsigned int icpIteration = 0; icpIteration < maxICPIteration; ++icpIteration)
  {
    // Current estimated isometry
    Eigen::Matrix3d R = RollPitchYawToMatrix(DoF6Params.block(0, 0, 3, 1));
    Eigen::Vector3d T = DoF6Params.block(3, 0, 3, 1);

    // Matches parameters
    std::vector<Eigen::Matrix3d> SemiDist;
    std::vector<Eigen::Vector3d> PointOnPlane, Points;
    std::vector<double> scores;

    unsigned int tooFarCounter = 0;
    unsigned int notPlanarCounter = 0;

    // loop to create ICP matches
    for (unsigned int pointIndex = 0; pointIndex < toAligned->size(); ++pointIndex)
    {
      // Get point and transform it
      pcl::PointXYZ rawPclPoint =  toAligned->points[pointIndex];
      Eigen::Vector3d rawX(rawPclPoint.x, rawPclPoint.y, rawPclPoint.z);
      Eigen::Vector3d X = R * rawX + T;
      pcl::PointXYZ pclPoint(X(0), X(1), X(2));

      Eigen::Matrix3d A;
      Eigen::Vector3d P;
      double planarity;

      unsigned int maxIndex = std::min((unsigned int)(maxDist.size() - 1), icpIteration);

      // Check that the current point is
      // belonging itself on a plane
      bool isOk = FindBestPlaneParameters(rawPclPoint, requiredNearest, maxDist[maxIndex], 0.35, kdtreeToAligned, A, P, planarity);

      // Get matches params
      isOk = isOk & FindBestPlaneParameters(pclPoint, requiredNearest, maxDist[maxIndex], 0.35, kdtreeReference, A, P, planarity);

      if (!isOk)
      {
        pointUsed[pointIndex] = false;
        continue;
      }

      PointOnPlane.push_back(P);
      Points.push_back(rawX);
      scores.push_back(planarity);
      SemiDist.push_back(A);
      pointUsed[pointIndex] = true;
    }

    //std::cout << "# Match: " << SemiDist.size() << " on " << toAligned->size() << std::endl;
    //std::cout << "# rejection: " << (toAligned->size() - SemiDist.size()) << std::endl;

    // We want to estimate our 6-DOF parameters using a non
    // linear least square minimization. The non linear part
    // comes from the Euler Angle parametrization of the rotation
    // endomorphism SO(3). To minimize it we use CERES to perform
    // the Levenberg-Marquardt algorithm.
    ceres::Problem problem;
    for (unsigned int k = 0; k < Points.size(); ++k)
    {
      ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctions::MahalanobisDistanceAffineIsometryResidual, 1, 6>(
                                           new CostFunctions::MahalanobisDistanceAffineIsometryResidual(SemiDist[k], PointOnPlane[k],
                                                                                                        Points[k], scores[k]));
      problem.AddResidualBlock(cost_function, nullptr, DoF6Params.data());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = maxLMIteration;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
  }

  Eigen::Matrix4d Href;
  Href.block(0, 0, 3, 3) = RollPitchYawToMatrix(DoF6Params.block(0, 0, 3, 1));
  Href.block(0, 3, 3, 1) = DoF6Params.block(3, 0, 3, 1);
  return Href;
}

//-----------------------------------------------------------------------------
bool FindBestPlaneParameters(pcl::PointXYZ query, unsigned int knearest, double maxDist,
                             double  minPlanarity, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree,
                             Eigen::Matrix3d& SemiDist, Eigen::Vector3d& P, double& planarity)
{
  // Get nearest neighbor of the current point transformed
  // using the current isometry estimation within the reference
  // pointcloud
  std::vector<float> neighSquaredDist;
  std::vector<int> neighIndex;
  kdtree->nearestKSearch(query, knearest, neighIndex, neighSquaredDist);

  // Invalid the query if the neighbor is too far
  if (std::sqrt(neighSquaredDist[neighSquaredDist.size() - 1]) > maxDist)
  {
    return false;
  }

  // Compute best plane that approximate the neighborhood
  Eigen::MatrixXd data(knearest, 3);
  for (unsigned int k = 0; k < knearest; k++)
  {
    pcl::PointXYZ pt = kdtree->getInputCloud()->points[neighIndex[k]];
    data.row(k) << pt.x, pt.y, pt.z;
  }
  P = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - P.transpose();
  Eigen::Matrix3d Sigma = centered.transpose() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(Sigma);

  // Eigenvalues
  Eigen::VectorXd D = eig.eigenvalues();
  Eigen::Vector3d n = eig.eigenvectors().col(0);
  SemiDist = n * n.transpose();
  planarity = (D(1) - D(0)) / D(2);

  if (planarity < minPlanarity)
  {
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
void ComputeSimilitude(const std::vector<Eigen::Vector3d>& X,
                       const std::vector<Eigen::Vector3d>& Y,
                       Eigen::Matrix<double, 9, 1>& W)
{
  ceres::Problem problem;
  for (unsigned int k = 0; k < X.size(); ++k)
  {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctions::SimilitudeResidual, 1, 9>(
                                         new CostFunctions::SimilitudeResidual(X[k], Y[k]));
    problem.AddResidualBlock(cost_function, nullptr, W.data());
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 250;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
}
