//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Data: 05-07-2019
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

#ifndef TRAJECTORY_REOPTIMIZATION_COST_FUNSTIONS_H
#define TRAJECTORY_REOPTIMIZATION_COST_FUNSTIONS_H

// LOCAL
#include "TrajectoryReoptimization.h"
#include "CeresTools.h"

// EIGEN
#include <Eigen/Dense>

// CERES
#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <ceres/rotation.h>

// Dynamic autodiff stride. The functor will evaluates
// the partial derivatives by batches of size Stride. hence,
// a larger stride will reduce the number of required passes
// to compute the all Jacobian but we result in a decreasing
// cache efficiency.
#define Stride 50

namespace CostFunctions
{
/**
* \class TrajectoryReoptimizationResidual
* \brief Reoptimization of a poses-trajectory
*        (both positions and orientations) estimated by the SLAM algorithm
*        using an external groundtruth landmark (GPS / IMU, ....)
*        Here, the estimated map will not be directly used to reoptimize
*        the trajectory. For each pose estimated, we will compute the local
*        (i.e relative) transform to apply so that the end pose will map
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
* @param argRelativePoses the list of relative poses that have been computed by the slam
*        and that we want to refined using the ground truth landmark
* @param argH ground truth pose data
*/
//-----------------------------------------------------------------------------
struct TrajectoryReoptimizationResidual
{
public:
  TrajectoryReoptimizationResidual(const PoseEstimationVector& argRelativePoses,
                                   const std::vector<Eigen::Matrix4d>& argH,
                                   const std::vector<int>& argIndex,
                                   MeasureProvided argMode)
  {
    this->RelativePoses = argRelativePoses;
    this->Hanchor = argH;
    this->anchorIndex = argIndex;
    this->MeasureMode = argMode;
  }

  template <typename T>
  bool operator()(T const* const* w, T* residual) const
  {
    int NPoses = this->RelativePoses.size();
    Eigen::Matrix<T, 4, 4> Hf = Eigen::Matrix<T, 4, 4>::Identity();

    // Compute the updated trajectory according to the current
    // parameter estimation and store the absolute poses for which
    // we have an anchor ground truth
    std::vector<Eigen::Matrix<T, 4, 4> > absolutePosesCorrected;
    for (int poseIndex = 0; poseIndex < NPoses; ++poseIndex)
    {
      // Compute rotation / position for this pose correction
      Eigen::Matrix<T, 3, 3> dR = ceres::RollPitchYawToMatrix(w[poseIndex][0], w[poseIndex][1], w[poseIndex][2]);
      Eigen::Matrix<T, 3, 1> dT(w[poseIndex][3], w[poseIndex][4], w[poseIndex][5]);
      // Compute the correction
      Eigen::Matrix<T, 4, 4> dHi = Eigen::Matrix<T, 4, 4>::Identity();
      dHi.block(0, 0, 3, 3) = dR;
      dHi.block(0, 3, 3, 1) = dT;
      // compute the corrected relative pose
      Eigen::Matrix<T, 4, 4> relH;
      for (int i = 0; i < this->RelativePoses[poseIndex].H.size(); ++i)
        relH(i) = T(this->RelativePoses[poseIndex].H(i));
      Eigen::Matrix<T, 4, 4> correctedH = dHi * relH;
      // Compute the corrected absolute pose
      Hf = Hf * correctedH;
      absolutePosesCorrected.push_back(Hf);
    }

    // create a residual value per anchor poses available in the data
    // the residual will be the frobenius squared norm between the currently
    // estimated absolute pose and the corresponding anchor.
    for (int anchor = 0; anchor < this->Hanchor.size(); ++anchor)
    {
      Eigen::Matrix<T, 4, 4> targetH;
      for (int i = 0; i < this->Hanchor[anchor].size(); ++i)
        targetH(i) = T(this->Hanchor[anchor](i));

      Eigen::Matrix<T, 4, 4> estimatedH = absolutePosesCorrected[this->anchorIndex[anchor]];

      T squaredResidual;
      if (this->MeasureMode == MeasureProvided::PositionOnly)
      {
        squaredResidual = ((estimatedH.block(0, 3, 3, 1) - targetH.block(0, 3, 3, 1)).transpose() *
                           (estimatedH.block(0, 3, 3, 1) - targetH.block(0, 3, 3, 1)))(0);
      }
      else if (this->MeasureMode == MeasureProvided::OrientationOnly)
      {
        squaredResidual = ((estimatedH.block(0, 0, 3, 3) - targetH.block(0, 0, 3, 3)).transpose()
                         * (estimatedH.block(0, 0, 3, 3) - targetH.block(0, 0, 3, 3))).trace();
      }
      else
      {
        squaredResidual = ((estimatedH - targetH).transpose() * (estimatedH - targetH)).trace();
      }

      // since t -> sqrt(t) is not differentiable
      // in 0, we check the value of the distance
      // infenitesimale part. If it is not finite
      // it means that the first order derivative
      // has been evaluated in 0
      if (squaredResidual < T(1e-6))
      {
        residual[anchor] = T(0);
      }
      else
      {
        residual[anchor] = ceres::sqrt(squaredResidual);
      }
    }

    return true;
  }

  static ceres::DynamicAutoDiffCostFunction<TrajectoryReoptimizationResidual, Stride>* Create(const PoseEstimationVector& argRelativePoses,
                                                                                              const std::vector<Eigen::Matrix4d>& argH,
                                                                                              const std::vector<int>& argIndex,
                                                                                              MeasureProvided argMode)
  {
    TrajectoryReoptimizationResidual* constraint = new TrajectoryReoptimizationResidual(argRelativePoses, argH, argIndex, argMode);
    ceres::DynamicAutoDiffCostFunction<TrajectoryReoptimizationResidual, Stride>* function =
        new ceres::DynamicAutoDiffCostFunction<TrajectoryReoptimizationResidual, Stride>(constraint);

    // Add all the parameter blocks that affect this contraint
    for (int poseIndex = 0; poseIndex < argRelativePoses.size(); ++poseIndex)
    {
      // 6-dof correction associated to this pose
      function->AddParameterBlock(6);
    }

    // We have one residual per anchor pose available
    function->SetNumResiduals(argH.size());
    return (function);
  }

private:
  PoseEstimationVector RelativePoses;
  std::vector<Eigen::Matrix4d> Hanchor;
  std::vector<int> anchorIndex;
  MeasureProvided MeasureMode;
};

//-----------------------------------------------------------------------------
struct TrajectoryParametersRegulizer
{
public:
  TrajectoryParametersRegulizer(int argNPoses, const PoseEstimationVector& argRelativePoses)
  {
    this->NPoses = argNPoses;
    this->RelativePoses = argRelativePoses;
  }

  template <typename T>
  bool operator()(T const* const* w, T* residual) const
  {
    // Regularizer terms
    for (int poseIndex = 0; poseIndex < NPoses; ++poseIndex)
    {
      Eigen::Matrix<T, 6, 1> block;
      block << w[poseIndex][0], w[poseIndex][1], w[poseIndex][2], w[poseIndex][3], w[poseIndex][4], w[poseIndex][5];

      Eigen::Matrix<T, 6, 6> Sigma;
      for (int i = 0; i < this->RelativePoses[poseIndex].Sigma.size(); ++i)
      {
        Sigma(i) = T(this->RelativePoses[poseIndex].Sigma(i));
      }

      T squaredResidual = block.transpose() * Sigma * block;
      if (squaredResidual < T(1e-6))
      {
        residual[poseIndex] = T(0);
      }
      else
      {
        residual[poseIndex] = ceres::sqrt(squaredResidual);
      }
    }
    return true;
  }

  static ceres::DynamicAutoDiffCostFunction<TrajectoryParametersRegulizer, Stride>* Create(int argNPoses,
                                                                                           const PoseEstimationVector& argRelativePoses)
  {
    TrajectoryParametersRegulizer* constraint = new TrajectoryParametersRegulizer(argNPoses, argRelativePoses);
    ceres::DynamicAutoDiffCostFunction<TrajectoryParametersRegulizer, Stride>* function =
        new ceres::DynamicAutoDiffCostFunction<TrajectoryParametersRegulizer, Stride>(constraint);

    // Add all the parameter blocks that affect this contraint
    for (int poseIndex = 0; poseIndex < argNPoses; ++poseIndex)
    {
      // 6-dof correction associated to this pose
      function->AddParameterBlock(6);
    }
    function->SetNumResiduals(argNPoses);
    return (function);
  }
private:
  int NPoses;
  PoseEstimationVector RelativePoses;
};

}

#endif // TRAJECTORY_REOPTIMIZATION_COST_FUNSTIONS_H
