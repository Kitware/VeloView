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

// LOCAL
#include "CameraCalibration.h"
#include "CameraProjection.h"
#include "vtkEigenTools.h"
#include "CeresCameraCalibrationCostFunctions.h"

// STD
#include <iostream>
#include <fstream>
#include <sstream>

// BOOST
#include <boost/algorithm/string.hpp>

// CERES
#include <ceres/ceres.h>

//----------------------------------------------------------------------------
void LoadMatchesFromCSV(std::string filename, std::vector<Eigen::Vector3d>& X, std::vector<Eigen::Vector2d>& x)
{
  // Load file and check that the file is opened
  std::ifstream file(filename.c_str());
  if (!file.is_open())
  {
    std::cout << "Error: could not load file: " << filename << std::endl;
    return;
  }

  // check the file header
  std::string tokenFileHeader("X,Y,Z,u,v");
  std::string line;
  std::getline(file, line);
  if (line != tokenFileHeader)
  {
    std::cout << "Error file header is: " << line << " expected: " << tokenFileHeader << std::endl;
    return;
  }

  // parse the file
  while (std::getline(file, line))
  {
    std::vector<std::string> values;
    boost::algorithm::split(values, line, boost::is_any_of(","));

    // check that each lines has the right number of value
    if (values.size() < 5)
    {
      std::cout << "Error, number of values is: " << values.size() << " expected 5" << std::endl;
      continue;
    }

    Eigen::Vector3d Xc(std::atof(values[0].c_str()), std::atof(values[1].c_str()), std::atof(values[2].c_str()));
    Eigen::Vector2d xc(std::atof(values[3].c_str()), std::atof(values[4].c_str()));
    X.push_back(Xc);
    x.push_back(xc);
  }
  return;
}

//----------------------------------------------------------------------------
double LinearPinholeCalibration(const std::vector<Eigen::Vector3d>& X, const std::vector<Eigen::Vector2d>& x, Eigen::Matrix<double, 3, 4>& P)
{
  P = Eigen::Matrix<double, 3, 4>::Zero();
  if (X.size() != x.size())
  {
    std::cout << "Error: matches have different sizes" << std::endl;
    return -1.0;
  }

  // We will estimate the coefficients of the projection matrix
  // by minimizing the reprojection euclidean distance between
  // the image keypoints and the 3D associated keypoints reprojected

  // Compute the normal equations
  Eigen::MatrixXd A(2 * X.size(), 12);
  for (int k = 0; k < X.size(); ++k)
  {
    A.row(2 * k) << X[k](0), X[k](1), X[k](2), 1,
                    0, 0, 0, 0,
                    -x[k](0) * X[k](0), -x[k](0) * X[k](1), -x[k](0) * X[k](2), -x[k](0);

    A.row(2 * k + 1) << 0, 0, 0, 0,
                        X[k](0), X[k](1), X[k](2), 1,
                       -x[k](1) * X[k](0), -x[k](1) * X[k](1), -x[k](1) * X[k](2), -x[k](1);
  }

  // Solve the normal equations
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd flattenP = svd.matrixV().col(11);
  P << flattenP(0), flattenP(1), flattenP(2), flattenP(3),
       flattenP(4), flattenP(5), flattenP(6), flattenP(7),
       flattenP(8), flattenP(9), flattenP(10), flattenP(11);
  P = P / P(2, 3);

  // Compute the RMSE
  double meanErr = 0;
  for (int k = 0; k < X.size(); ++k)
  {
    Eigen::Vector4d homoX(X[k](0), X[k](1), X[k](2), 1);
    Eigen::Vector3d projX = P * homoX;
    Eigen::Vector2d projXn(projX(0) / projX(2), projX(1) / projX(2));
    //std::cout << "X: " << X[k].transpose() << " x: " << x[k].transpose() << " prjX: " << projXn.transpose() << " dist: " << (x[k] - projXn).norm() << std::endl;
    meanErr += ((x[k] - projXn).transpose() * (x[k] - projXn))(0);
  }
  return std::sqrt(meanErr / (1.0 * X.size()));
}

//----------------------------------------------------------------------------
void CalibrationMatrixDecomposition(const Eigen::Matrix<double, 3, 4>& P, Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& T)
{
  // M = K*R
  Eigen::Matrix3d M = P.block(0, 0, 3, 3);

  // M is the product of an upper triangulate matrix
  // and an orthogonal matrix. We will use a RQ decomposition
  // to recover K and R. Since eigen only provides QR
  // decomposition we will inverse the column order of M to
  // get its RQ decomposition from the QR decomposition
  Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
  D(2, 0) = 1; D(1, 1) = 1; D(0, 2) = 1;
  Eigen::Matrix3d Mtilde = D * M;
  Eigen::HouseholderQR<Eigen::Matrix3d> QRdec(Mtilde.transpose());
  Eigen::Matrix3d Qtilde = QRdec.householderQ();
  Eigen::Matrix3d Rtilde = Qtilde.transpose() * (Mtilde.transpose());
  K = D * Rtilde.transpose() * D;
  R = D * Qtilde.transpose();
  T = P.col(3);
  T = - R.transpose() * K.inverse() * T;

  // rescale the matrix
  K = K / K(2, 2);
  R = R * std::cbrt(1.0 / R.determinant());
  R = R.transpose().eval();

  Eigen::Matrix<double, 3, 4> H;
  H.block(0, 0, 3, 3) = K * R.transpose();
  H.col(3) = -K * R.transpose() * T;
  H = H / H(2, 3);
  if (((H - P).transpose() * (H - P)).trace() > 1e-6)
  {
    std::cout << "Error: decomposition failed" << std::endl;
  }
  return;
}

//----------------------------------------------------------------------------
void GetParametersFromMatrix(const Eigen::Matrix3d& K, const Eigen::Matrix3d& R, const Eigen::Vector3d& T, Eigen::Matrix<double, 11, 1>& W)
{
  Eigen::Vector3d eulerAngles = MatrixToRollPitchYaw(R);
  W(0) = eulerAngles(0); W(1) = eulerAngles(1); W(2) = eulerAngles(2);
  W(3) = T(0), W(4) = T(1), W(5) = T(2);
  W(6) = K(0, 0);
  W(7) = K(1, 1);
  W(8) = K(0, 2);
  W(9) = K(1, 2);
  W(10) = K(0, 1);
  return;
}

//----------------------------------------------------------------------------
double NonLinearPinholeCalibration(const std::vector<Eigen::Vector3d>& X, const std::vector<Eigen::Vector2d>& x, Eigen::Matrix<double, 11, 1>& W)
{
  Eigen::Matrix<double, 3, 4> P0, P1;
  GetMatrixFromParameters(W, P0);

  // We want to estimate our 11-DOF parameters using a non
  // linear least square minimization. The non linear part
  // comes from the Euler Angle parametrization of the rotation
  // endomorphism of SO(3) and the homographie rescaling
  // To minimize it, we use CERES to perform
  // the Levenberg-Marquardt algorithm.
  ceres::Problem problem;
  for (unsigned int k = 0; k < X.size(); ++k)
  {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctions::PinholeModelAlgebraicDistance, 1, 11>(
                                         new CostFunctions::PinholeModelAlgebraicDistance(X[k], x[k]));
    problem.AddResidualBlock(cost_function, nullptr, W.data());
  }

  // Solve the problem
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;

  // Compute mean error
  GetMatrixFromParameters(W, P1);
  double meanErr = 0;
  for (int k = 0; k < X.size(); ++k)
  {
    Eigen::Vector4d homoX(X[k](0), X[k](1), X[k](2), 1);
    Eigen::Vector3d projX = P1 * homoX;
    Eigen::Vector2d projXn(projX(0) / projX(2), projX(1) / projX(2));
    meanErr += ((x[k] - projXn).transpose() * (x[k] - projXn))(0);
  }
 return std::sqrt(meanErr / (1.0 * X.size()));
}

//----------------------------------------------------------------------------
double NonLinearFisheyeCalibration(const std::vector<Eigen::Vector3d>& X, const std::vector<Eigen::Vector2d>& x,
                                   Eigen::Matrix<double, 15, 1>& W, unsigned int it)
{
  // We want to estimate our 15-DOF parameters using a non
  // linear least square minimization. The non linear part
  // comes from the Euler Angle parametrization of the rotation
  // endomorphism of SO(3) and the homographie rescaling
  // To minimize it, we use CERES to perform
  // the Levenberg-Marquardt algorithm.
  ceres::Problem problem;
  for (unsigned int k = 0; k < X.size(); ++k)
  {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctions::FisheyeModelAlgebraicDistance, 1, 15>(
                                         new CostFunctions::FisheyeModelAlgebraicDistance(X[k], x[k]));
    problem.AddResidualBlock(cost_function, nullptr, W.data());
  }

  ceres::Solver::Options options;
  options.max_num_iterations = it;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  double meanErr = 0;
  for (int k = 0; k < X.size(); ++k)
  {
    meanErr += ((x[k] - FisheyeProjection(W, X[k])).transpose() * (x[k] - FisheyeProjection(W, X[k])))(0);
  }
  return std::sqrt(meanErr / (1.0 * X.size()));
}

//----------------------------------------------------------------------------
double BrownConradyPinholeCalibration(const std::vector<Eigen::Vector3d>& X, const std::vector<Eigen::Vector2d>& x,
                                      Eigen::Matrix<double, 17, 1>& W, unsigned int it,
                                      double initLossScale, double finalLossScale,
                                      const std::vector<bool>& shouldOptimizeParam)
{
  unsigned int N = 100;

  // The minimization algorithm will be ran multiple time
  // with an outlier rejection loss function more restrictive
  // at each iteration. We don't want to have a higly restrictive
  // outlier rejection at the beginning since the initial point can
  // be far from the global minimum and it could create convergence
  // issues.
  for (unsigned int minId = 0; minId < N; ++minId)
  {
    double lossScale = initLossScale + static_cast<double>(minId) * (finalLossScale - initLossScale) / (1.0 * N);

    // We want to estimate our 17-DOF parameters using a non
    // linear least square minimization. The non linear part
    // comes from the Euler Angle parametrization of the rotation
    // endomorphism of SO(3), the homographie rescaling and
    // the lens distortions
    // To minimize it, we use CERES to perform
    // the Levenberg-Marquardt algorithm.
    ceres::Problem problem;
    for (unsigned int k = 0; k < X.size(); ++k)
    {
      CostFunctions::BrownConradyAlgebraicDistance* resFct = new CostFunctions::BrownConradyAlgebraicDistance(X[k], x[k]);
      resFct->SetW0(W);
      resFct->SetActivatedParams(shouldOptimizeParam);
      ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctions::BrownConradyAlgebraicDistance, 1, 17>(resFct);
      problem.AddResidualBlock(cost_function, new ceres::ArctanLoss(lossScale), W.data());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = it;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
  }

  double meanErr = 0;
  for (int k = 0; k < X.size(); ++k)
  {
    meanErr += ((x[k] - BrownConradyPinholeProjection(W, X[k])).transpose() * (x[k] - BrownConradyPinholeProjection(W, X[k])))(0);
  }
  return std::sqrt(meanErr / (1.0 * X.size()));
}

//----------------------------------------------------------------------------
void GetMatrixFromParameters(const Eigen::Matrix<double, 11, 1>& W, Eigen::Matrix<double, 3, 4>& P)
{
  // Create current rotation
  Eigen::Matrix3d R = RollPitchYawToMatrix(W(0), W(1), W(2));
  // Create current position
  Eigen::Vector3d T(W(3), W(4), W(5));

  // Create current intrinsic parameters
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  K(0, 0) = W(6);
  K(1, 1) = W(7);
  K(0, 2) = W(8);
  K(1, 2) = W(9);
  K(0, 1) = W(10);
  K(2, 2) = 1;

  // Create current calibration matrix
  P.block(0, 0, 3, 3) = K * R.transpose();
  P.col(3) = -K * R.transpose() * T;
  P = P / P(2, 3);
}

//----------------------------------------------------------------------------
Eigen::Matrix<double, 3, 4> GetMatrixFromParameters(const Eigen::Matrix<double, 11, 1>& W)
{
  Eigen::Matrix<double, 3, 4> P;
  GetMatrixFromParameters(W, P);
  return P;
}

//----------------------------------------------------------------------------
Eigen::VectorXd FullCalibrationPipelineFromMatches(std::string filename, const std::vector<bool>& activatedParams)
{
  Eigen::VectorXd Wf = Eigen::VectorXd::Zero(17, 1);

  // Load the 3D - 2D matches
  std::vector<Eigen::Vector3d> X;
  std::vector<Eigen::Vector2d> x;
  LoadMatchesFromCSV(filename, X, x);
  if (X.size() == 0)
  {
    return Wf;
  }

  // First, launch a linear pinhole camera model
  // projection matrix estimation
  Eigen::Matrix<double, 3, 4> P;
  double rmse1 = LinearPinholeCalibration(X, x, P);

  // From this first linear pinhole projection matrix
  // estimation, extract the pinhole model parameters
  Eigen::Matrix3d K, R;
  Eigen::Vector3d T;
  CalibrationMatrixDecomposition(P, K, R, T);

  // Check that the optical axis of the camera is
  // correctly oriented according to the 3D points
  Eigen::Vector3d Xmean = Eigen::Vector3d::Zero();
  for (int i = 0; i < X.size(); ++i)
  {
    Xmean += X[i] / static_cast<double>(X.size());
  }
  Eigen::Vector3d ez = R.col(2);
  double angle = std::acos((Xmean.transpose() * ez)(0) / (Xmean.norm() * ez.norm())) / vtkMath::Pi() * 180.0;

  // In this case, the linear algorithm has provided
  // a solution where the camera is looking backward and
  // the resulting symmetry is handled by the intrinsic matrix.
  // To avoid that, we return the camera if it is not looking
  // forward
  if (angle > 90.0)
  {
    Eigen::Matrix3d S;
    S << -1.0, 0.0,  0.0,
          0.0, 1.0,  0.0,
          0.0, 0.0, -1.0;

    Eigen::Matrix3d Rtilde = R * S * R.transpose();

    // So that K' * R' = K * R, meaning that
    // the global projection is unchanged but
    // the camera orientation has made a 180
    // rotation around its y-axis
    K = K * Rtilde.transpose();
    K = K / K(2, 2);
    R = Rtilde * R;
  }

  Eigen::Matrix<double, 11, 1> Wpinhole;
  GetParametersFromMatrix(K, R, T, Wpinhole);
  Eigen::Matrix<double, 11, 1> Wpi = Wpinhole;

  // Then, refine the model obtained using linear
  // estimation by using a non-linear pinhole parameters
  // estimation
  double rmse2 = NonLinearPinholeCalibration(X, x, Wpinhole);

  // Finally, create a first parameter vector estimation
  // by using the pinhole parameters and setting the distortion
  // coefficients to 0
  Eigen::Matrix<double, 17, 1> West = Eigen::Matrix<double, 17, 1>::Zero();
  West.block(0, 0, 11, 1) = Wpinhole;

  double rmse3 = BrownConradyPinholeCalibration(X, x, West, 2500, 5.0, 0.6, activatedParams);

  // copy params
  for (int i = 0; i < 17; ++i)
  {
    Wf(i) = West(i);
  }

  Eigen::Vector3d angles(Wf(0), Wf(1), Wf(2));
  R = RollPitchYawToMatrix(angles);

  std::cout << "RMSE1: " << rmse1 << std::endl;
  std::cout << "RMSE2: " << rmse2 << std::endl;
  std::cout << "RMSE3: " << rmse3 << std::endl;
  std::cout << "W: ";
  for (int i = 0; i < 17; ++i)
  {
    std::cout << Wf(i) << ",";
  }
  std::cout << std::endl;
  return Wf;
}
