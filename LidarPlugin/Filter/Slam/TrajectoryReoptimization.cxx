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

#include "TrajectoryReoptimization.h"
// STD
#include <iostream>
// PCL
#include <pcl/kdtree/kdtree_flann.h>
// CERES
#include <ceres/ceres.h>
// VTK
#include <vtkCell.h>
#include <vtkDoubleArray.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPointData.h>
// LOCAL
#include "TrajectoryReoptimizationCostFunctions.h"
#include "vtkEigenTools.h"
#include "CeresCostFunctions.h"
#include "RegistrationTools.h"
#include "CeresTools.h"

//-----------------------------------------------------------------------------
PoseEstimationVector RelativePosesFromAbsolutePoses(const PoseEstimationVector& absolutePoses)
{
  int NPoses = absolutePoses.size();
  PoseEstimationVector relativePoses(NPoses);
  for (int poseIndex = 0; poseIndex < NPoses; ++poseIndex)
  {
    // Compute relative pose estimation
    PoseEstimation currentRelativePose = absolutePoses[poseIndex];
    int prevIndex = std::max(0, poseIndex - 1);
    // Express the rotation and position of the current sensor
    // pose according to the previous pose coordinate system
    currentRelativePose.H = absolutePoses[prevIndex].H.inverse() * absolutePoses[poseIndex].H;
    // Express the variance-covariance matrix of the 6dof
    // estimation in the coordinate frame attached to the
    // previous pose coordinate system
    Eigen::Matrix<double, 6, 1> prevW, currW;

    prevW.block(0, 0, 3, 1) = MatrixToRollPitchYaw(absolutePoses[prevIndex].H.block(0, 0, 3, 3));
    prevW.block(3, 0, 3, 1) = absolutePoses[prevIndex].H.block(0, 3, 3, 1);

    currW.block(0, 0, 3, 1) = MatrixToRollPitchYaw(absolutePoses[poseIndex].H.block(0, 0, 3, 3));
    currW.block(3, 0, 3, 1) = absolutePoses[poseIndex].H.block(0, 3, 3, 1);

    Eigen::Matrix<double, 6, 6> J = ceres::Express6DoFPoseInOtherCoordinateFrame(currW, prevW).second;
    Eigen::Matrix<double, 6, 6> Sigma = absolutePoses[poseIndex].Sigma.inverse();
    currentRelativePose.Sigma = (J * Sigma * J.transpose()).inverse();

    relativePoses[poseIndex] = currentRelativePose;
  }

  return relativePoses;
}

//-----------------------------------------------------------------------------
PoseEstimationVector TrajectoryReoptimization(const PoseEstimationVector& absolutePoses,
                                              const Eigen::Matrix4d& H, int gtIndex, int maxIteration,
                                              MeasureProvided dataMode)
{
  std::vector<Eigen::Matrix4d> HList;
  std::vector<int> gtIndexList;
  HList.push_back(H);
  gtIndexList.push_back(gtIndex);
  return TrajectoryReoptimization(absolutePoses, HList, gtIndexList, maxIteration, dataMode);
}

//-----------------------------------------------------------------------------
PoseEstimationVector TrajectoryReoptimization(const PoseEstimationVector& absolutePoses,
                                              const std::vector<Eigen::Matrix4d>& H, std::vector<int> gtIndex,
                                              int maxIteration, MeasureProvided dataMode)
{
  // Relative poses estimated by the SLAM algorithm
  // It represents the pose of the sensor at the time tk
  // according to the reference frame attached to the sensor
  // at time tk-1. It is the estimated odometry of the sensor
  PoseEstimationVector relativePoses = RelativePosesFromAbsolutePoses(absolutePoses);

  // Non-linear least square problem that consists of 2 residual blocks
  // - Distance to groundtruth minimization
  // - Regularizer that leads the attach to odometry data
  std::vector<Eigen::Matrix<double, 6, 1> > W(relativePoses.size());
  std::vector<double*> parameterBlocks(relativePoses.size());
  for (int k = 0; k < relativePoses.size(); ++k)
  {
    W[k] = Eigen::Matrix<double, 6, 1>::Zero();
    parameterBlocks[k] = W[k].data();
  }

  // Build the problem
  ceres::Problem problem;
  // groundtruth distance minimization
  ceres::DynamicAutoDiffCostFunction<CostFunctions::TrajectoryReoptimizationResidual, Stride>* gtFunction =
      CostFunctions::TrajectoryReoptimizationResidual::Create(relativePoses, H, gtIndex, dataMode);
  // Regularizer function minimization
  ceres::DynamicAutoDiffCostFunction<CostFunctions::TrajectoryParametersRegulizer, Stride>* regFunction =
  CostFunctions::TrajectoryParametersRegulizer::Create(relativePoses.size(), relativePoses);
  problem.AddResidualBlock(gtFunction, new ceres::ScaledLoss(nullptr, 1.0, ceres::TAKE_OWNERSHIP), parameterBlocks);
  problem.AddResidualBlock(regFunction, new ceres::ScaledLoss(nullptr, 1.0 / static_cast<double>(relativePoses.size()), ceres::TAKE_OWNERSHIP), parameterBlocks);

  ceres::Solver::Options options;
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;
  options.minimizer_type = ceres::LINE_SEARCH;
  //options.line_search_direction_type = ceres::STEEPEST_DESCENT;
  //options.line_search_type = ceres::ARMIJO;
  options.max_num_iterations = maxIteration;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << std::endl;

  // absolute poses reoptimized by the algorithm
  // It represents the pose of the sensor at the time tk
  // according to the fixed reference frame. Typically the
  // one attached to the sensor at time t0
  PoseEstimationVector absoluteCorrectedPoses;
  Eigen::Matrix4d Hcorrected = Eigen::Matrix4d::Identity();
  for (int poseIndex = 0; poseIndex < absolutePoses.size(); ++poseIndex)
  {
    // Compute the current pose regarding world reference frame
    Eigen::Matrix4d dH = Eigen::Matrix4d::Identity();
    dH.block(0, 0, 3, 3) = RollPitchYawToMatrix(W[poseIndex].block(0, 0, 3, 1));
    dH.block(0, 3, 3, 1) = W[poseIndex].block(3, 0, 3, 1);
    Hcorrected = Hcorrected * dH * relativePoses[poseIndex].H;
    PoseEstimation currentEstimation = relativePoses[poseIndex];
    currentEstimation.H = Hcorrected;
    currentEstimation.AnglesDistance = std::sqrt(std::abs((W[poseIndex].block(0, 0, 3, 1).transpose() * W[poseIndex].block(0, 0, 3, 1))(0)));
    currentEstimation.PositionDistance = std::sqrt(std::abs((W[poseIndex].block(3, 0, 3, 1).transpose() * W[poseIndex].block(3, 0, 3, 1))(0)));
    currentEstimation.MahalanobisDistance = std::sqrt(std::abs((W[poseIndex].transpose() * currentEstimation.Sigma.inverse() * W[poseIndex])(0)));
    absoluteCorrectedPoses.push_back(currentEstimation);
  }
  return absoluteCorrectedPoses;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> ConvertPoseEstimationToPolyData(const PoseEstimationVector& poses)
{
  vtkSmartPointer<vtkPolyData> trajectory = vtkSmartPointer<vtkPolyData>::New();
  vtkNew<vtkDoubleArray> axisAngleArray;
  axisAngleArray->SetName("Orientation(AxisAngle)");
  axisAngleArray->SetNumberOfComponents(4);
  axisAngleArray->SetNumberOfTuples(poses.size());
  vtkNew<vtkPoints> points;

  for (unsigned int poseIndex = 0; poseIndex < poses.size(); ++poseIndex)
  {
    Eigen::Matrix3d Rot = poses[poseIndex].H.block(0, 0, 3, 3);
    Eigen::AngleAxisd angleAxis(Rot);
    Eigen::Vector3d pos = poses[poseIndex].H.block(0, 3, 3, 1);
    double axisAngle[4];
    axisAngle[0] = angleAxis.axis()(0); axisAngle[1] = angleAxis.axis()(1);
    axisAngle[2] = angleAxis.axis()(2); axisAngle[3] = angleAxis.angle();
    axisAngleArray->SetTuple4(poseIndex, axisAngle[0], axisAngle[1], axisAngle[2], axisAngle[3]);
    points->InsertNextPoint(pos.data());
  }

  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(trajectory->GetNumberOfPoints() * 2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < trajectory->GetNumberOfPoints(); ++i)
  {
    ids[i * 2] = 1;
    ids[i * 2 + 1] = i;
  }

  vtkNew<vtkCellArray> cellArray;
  cellArray->SetCells(trajectory->GetNumberOfPoints(), cells.GetPointer());
  trajectory->SetVerts(cellArray);
  trajectory->GetPointData()->AddArray(axisAngleArray);
  trajectory->SetPoints(points);
}

//-----------------------------------------------------------------------------
void ConvertPolyDataToPoseEstimation(vtkSmartPointer<vtkPolyData> trajectory,
                                     PoseEstimationVector& poses)
{
  poses.clear();
  poses.resize(trajectory->GetNumberOfPoints());

  if (!trajectory)
  {
    vtkGenericWarningMacro("The input trajectory is a null pointer");
    return;
  }

  // Create the new corrected trajectory
  vtkSmartPointer<vtkDataArray> varianceArray = trajectory->GetPointData()->GetArray("Variance Covariance Matrix");
  vtkSmartPointer<vtkDataArray> axisAngleArray = trajectory->GetPointData()->GetArray("Orientation(AxisAngle)");

  // Check if required information are available
  if (!varianceArray || !axisAngleArray)
  {
    vtkGenericWarningMacro("One or many required information arrays are missing from the trajectory");
    return;
  }

  // Get information about the poses estimations
  for (unsigned int poseIndex = 0; poseIndex < trajectory->GetNumberOfPoints(); ++poseIndex)
  {
    // Get information about the pose estimation
    double* variance = varianceArray->GetTuple(poseIndex);
    double* axisAngle = axisAngleArray->GetTuple(poseIndex);
    double* position = trajectory->GetPoint(poseIndex);
    double angle = axisAngle[3];
    Eigen::Vector3d axis(axisAngle[0], axisAngle[1], axisAngle[2]);

    // Populate current absolute pose estimation information
    PoseEstimation currentAbsolutePose;
    currentAbsolutePose.H.block(0, 0, 3, 3) = Eigen::Matrix3d(Eigen::AngleAxisd(axisAngle[3], axis));
    currentAbsolutePose.H.block(0, 3, 3, 1) = Eigen::Vector3d(position[0], position[1], position[2]);
    currentAbsolutePose.H(3, 3) = 1.0;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        currentAbsolutePose.Sigma(i, j) = variance[i + 6 * j];
    Eigen::Matrix<double, 6, 6> invSigma = currentAbsolutePose.Sigma.inverse();
    currentAbsolutePose.Sigma = invSigma;
    poses[poseIndex] = currentAbsolutePose;
  }
  return;
}
