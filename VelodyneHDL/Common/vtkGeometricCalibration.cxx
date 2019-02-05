//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Data: 01-23-2019
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
#include "vtkGeometricCalibration.h"
#include "vtkVelodyneTransformInterpolator.h"
#include "vtkEigenTools.h"
#include "vtkConversions.h"
#include "vtkTemporalTransformsReader.h"
#include "CeresCostFunctions.h"

// STD
#include <stdlib.h>
#include <ctime>

// VTK
#include <vtkDoubleArray.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// CERES
#include <ceres/ceres.h>

//----------------------------------------------------------------------------
std::pair<double, AnglePositionVector> EstimateCalibrationFromPoses(const std::string& sourceSensorFilename,
                                                                    const std::string& targetSensorFilename)
{
  vtkSmartPointer<vtkTemporalTransforms> trans1, trans2;
  trans1 = vtkTemporalTransformsReader::OpenTemporalTransforms(sourceSensorFilename);
  trans2 = vtkTemporalTransformsReader::OpenTemporalTransforms(targetSensorFilename);
  return EstimateCalibrationFromPoses(trans1, trans2);
}

//----------------------------------------------------------------------------
std::pair<double, AnglePositionVector> EstimateCalibrationFromPoses(
                                              vtkSmartPointer<vtkTemporalTransforms> sourceSensor,
                                              vtkSmartPointer<vtkTemporalTransforms> targetSensor)
{
  // Multi resolution time analysis parameters
  const double multipleScaleTimeBound = 5.0; // in seconds
  const double deltaScaleTime = 0.2; // in seconds
  const double timeStep = 0.4; // in seconds

  Eigen::Matrix3d P1, P2, Q1, Q2;
  Eigen::Vector3d U1, U2, V1, V2;

  // Parameters to estimate
  // - Rotation euler angles from 0 to 2
  // - Translation coordinates from 3 to 5
  AnglePositionVector calibEstimation = AnglePositionVector::Zero();

  // Create the transforms interpolators
  vtkSmartPointer<vtkVelodyneTransformInterpolator> sourceSensorTransforms = sourceSensor->CreateInterpolator();
  vtkSmartPointer<vtkVelodyneTransformInterpolator> targetSensorTransforms = targetSensor->CreateInterpolator();
  sourceSensorTransforms->SetInterpolationTypeToLinear();
  targetSensorTransforms->SetInterpolationTypeToLinear();
  double tmin = std::max(sourceSensorTransforms->GetMinimumT(), targetSensorTransforms->GetMinimumT());
  double tmax = std::min(sourceSensorTransforms->GetMaximumT(), targetSensorTransforms->GetMaximumT());
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

  // We want to estimate our 6-DOF parameters using a non
  // linear least square minimization. The non linear part
  // comes from the Euler Angle parametrization of the rotation
  // endomorphism SO(3). To minimize it we use CERES to perform
  // the Levenberg-Marquardt algorithm.
  ceres::Problem problem;

  // Loop over the time index
  for (double time = tmin + multipleScaleTimeBound; time < tmax - multipleScaleTimeBound; time += timeStep)
  {
    // Loop over the deltaTime multi-resolution "solid-system" assumption constraint
    for (double dt = 0; dt <= multipleScaleTimeBound;  dt += deltaScaleTime)
    {
      // The two time positions that will be used to express
      // the solid-system geometric constraints that link
      // the two sensor poses trajectories
      double t0 = time - dt;
      double t1 = time + dt;

      //======================== Time: t0 ==================================
      // Sensor 1
      sourceSensorTransforms->InterpolateTransform(t0, transform);
      std::pair<Eigen::Vector3d, Eigen::Vector3d> sensor1T0Pose = GetPoseParamsFromTransform(transform);
      Q1 = RollPitchYawToMatrix(sensor1T0Pose.first); U1 = sensor1T0Pose.second;
      // Sensor 2
      targetSensorTransforms->InterpolateTransform(t0, transform);
      std::pair<Eigen::Vector3d, Eigen::Vector3d> sensor2T0Pose = GetPoseParamsFromTransform(transform);
      P1 = RollPitchYawToMatrix(sensor2T0Pose.first); V1 = sensor2T0Pose.second;

      //======================== Time: t1 ==================================
      // Sensor 1
      sourceSensorTransforms->InterpolateTransform(t1, transform);
      std::pair<Eigen::Vector3d, Eigen::Vector3d> sensor1T1Pose = GetPoseParamsFromTransform(transform);
      Q2 = RollPitchYawToMatrix(sensor1T1Pose.first); U2 = sensor1T1Pose.second;
      // Sensor 2
      targetSensorTransforms->InterpolateTransform(t1, transform);
      std::pair<Eigen::Vector3d, Eigen::Vector3d> sensor2T1Pose = GetPoseParamsFromTransform(transform);
      P2 = RollPitchYawToMatrix(sensor2T1Pose.first); V2 = sensor2T1Pose.second;

      // add this geometric constraint non-linear least square residu to the global
      // cost function that is the sum of all residuals functions
      ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctions::FrobeniusDistanceRotationAndTranslationCalibrationResidual, 1, 6>
                (new CostFunctions::FrobeniusDistanceRotationAndTranslationCalibrationResidual(P1, P2, Q1, Q2, V1, V2, U1, U2));
      problem.AddResidualBlock(cost_function, nullptr, calibEstimation.data());
    } // Loop over the deltaTime multi-resolution "solid-system" assumption constraint
  } // Loop over the time

  // Solve the optimization problem
  // Option of the solver
  ceres::Solver::Options options;
  options.max_num_iterations = 75;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << ", Number Blocks: " << summary.num_residuals << std::endl;

  return std::pair<double, AnglePositionVector>(summary.final_cost, calibEstimation);
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> EstimateCalibrationFromPosesAndApply(
                                            vtkSmartPointer<vtkTemporalTransforms> targetSensor,
                                            vtkSmartPointer<vtkTemporalTransforms> sourceSensor)
{
  // Estimate the cycloidic transform that match the source trajectory on the target one
  std::pair<double, AnglePositionVector> estimation = EstimateCalibrationFromPoses(targetSensor, sourceSensor);
  // Transform the source trajectory
  std::pair<Eigen::Vector3d, Eigen::Vector3d> dof6(estimation.second.segment(0, 3), estimation.second.segment(3, 3));
  vtkSmartPointer<vtkTransform> transform = GetTransformFromPosesParams(dof6);
  // return the result
  return sourceSensor->CycloidicTransform(transform);
}

//----------------------------------------------------------------------------
struct CombinationTry
{
  CombinationTry(int index_, int* angleOrder_, int* matrixOrder_,
                 double* signs_, Eigen::Vector3d angles_,
                 Eigen::Vector3d translation_, double finalVal_)
  {
    this->index = index_;
    std::copy(angleOrder_, angleOrder_ + 3, angleOrder);
    std::copy(matrixOrder_, matrixOrder_ + 3, matrixOrder);
    std::copy(signs_, signs_ + 3, signs);
    this->AnglesCalib = angles_ / vtkMath::Pi() * 180.0;
    this->Translation = translation_;
    this->finalErrorValue = finalVal_;
  }

  int index;
  int angleOrder[3];
  int matrixOrder[3];
  int signs[3];
  Eigen::Vector3d AnglesCalib;
  Eigen::Vector3d Translation;
  double finalErrorValue;

  void PrintSelf()
  {
    std::cout << "Final Cost value: " << this->finalErrorValue << " matrix combination: (" << this->matrixOrder[0]
              << "," << this->matrixOrder[1] << "," << this->matrixOrder[2] << ") angles combination: ("
              << this->angleOrder[0] << "," << this->angleOrder[1] << "," << this->angleOrder[2] << ") signs: ( "
              << this->signs[0] << " , " << this->signs[1] << " , " << this->signs[2] << " ) calibration angles: "
              << this->AnglesCalib.transpose() << " Translation: " << this->Translation.transpose() << std::endl;
  }
};

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> ChangeConvention(vtkSmartPointer<vtkTemporalTransforms> inputPoses,
                                                        int anglesOrder[3],
                                                        int matrixOrder[3],
                                                        double sign[3])
{
  // We will loop over the transform points and get the rotational part. Then, the rotation
  // matrix will be decomposed using the Euler-Angles and recomposed using the same euler angles
  // but using the current convention of angles signs, permutation and matrix multiplication order
  vtkSmartPointer<vtkTemporalTransforms> outputPoses = vtkSmartPointer<vtkTemporalTransforms>::New();
  outputPoses->DeepCopy(inputPoses);
  vtkDoubleArray* xyzwArray = vtkDoubleArray::SafeDownCast(outputPoses->GetOrientationArray());

  // Loop over the transforms points
  for (unsigned int transformIndex = 0; transformIndex < outputPoses->GetNumberOfPoints(); transformIndex++)
  {
    // Get the angle-axis representation
    double* xyzw = xyzwArray->GetTuple4(transformIndex);

    // Compute the corresponding rotation and get the euler-angles with respect
    // to the convention:
    // R(rx, ry, rz) = Rz(rz)*Ry(ry)*Rx(rx)
    Eigen::AngleAxisd angleAxis(xyzw[3], Eigen::Vector3d(xyzw[0], xyzw[1], xyzw[2]));
    Eigen::Matrix3d rotation = angleAxis.toRotationMatrix();
    Eigen::Vector3d eulerAngles = MatrixToRollPitchYaw(rotation);

    // Now, change the euler angle convention using the current
    // convention parameters
    double rx = sign[0] * eulerAngles(anglesOrder[0]); // rx can be +1/-1 * r x/y/z
    double ry = sign[1] * eulerAngles(anglesOrder[1]); // ry can be +1/-1 * r x/y/z
    double rz = sign[2] * eulerAngles(anglesOrder[2]); // rz can be +1/-1 * r x/y/z

    // Compute the rotation around canonic axis
    std::vector<Eigen::Matrix3d> Rot(3);
    Rot[0] = Eigen::Matrix3d(Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
    Rot[1] = Eigen::Matrix3d(Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()));
    Rot[2] = Eigen::Matrix3d(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d newRotation = Rot[matrixOrder[0]] * Rot[matrixOrder[1]] * Rot[matrixOrder[2]];

    // Get the axis angle representation of this new rotation
    Eigen::AngleAxisd newAngleAxis(newRotation);
    double newXyzw[4] = {newAngleAxis.axis()(0), newAngleAxis.axis()(1),
                         newAngleAxis.axis()(2), newAngleAxis.angle()};

    // Replace it
    xyzwArray->SetTuple4(transformIndex, newXyzw[0], newXyzw[1], newXyzw[2], newXyzw[3]);
  }

  return outputPoses;
}

//----------------------------------------------------------------------------
void EstimateEulerAngleConvention(const std::string& sourceSensorFilename,
                                  const std::string& targetSensorFilename)
{
  vtkSmartPointer<vtkTemporalTransforms> trans1, trans2;
  trans1 = vtkTemporalTransformsReader::OpenTemporalTransforms(sourceSensorFilename);
  trans2 = vtkTemporalTransformsReader::OpenTemporalTransforms(targetSensorFilename);
  EstimateEulerAngleConvention(trans1, trans2);
}

//-----------------------------------------------------------------------------
void EstimateEulerAngleConvention(vtkSmartPointer<vtkTemporalTransforms> sourceSensor, vtkSmartPointer<vtkTemporalTransforms> targetSensor)
{
  int anglesOrder[3] = {0, 1, 2};
  int matrixOrder[3] = {0, 1, 2};
  std::vector<CombinationTry> results;
  results.reserve(288); // 6 angles positions * 6 matrix order * 8 signs
  bool shouldPermutAngles = true;
  int combinationCount = 0;

  // Loop over the angles positions permutations
  while (shouldPermutAngles)
  {
    bool shouldPermutMatrix = true;
    // Loop over matrix multiplication order
    while (shouldPermutMatrix)
    {
      // Loop over the angles signs
      for (int signIndex = 0; signIndex < 8; ++signIndex)
      {
        double sign[3];
        // convert signIndex into binary code using -1 and 1
        int quotient = signIndex;
        sign[2] = (quotient % 2) ? 1.0 : -1.0;
        quotient = quotient / 2;
        sign[1] = (quotient % 2) ? 1.0 : -1.0;
        quotient = quotient / 2;
        sign[0] = (quotient % 2) ? 1.0 : -1.0;

        // Create a new vtkTemporalTransforms according to the current euler angle convention
        vtkSmartPointer<vtkTemporalTransforms> targetPosesCurrCombination = ChangeConvention(targetSensor,
                                                                                              anglesOrder,
                                                                                              matrixOrder,
                                                                                              sign);

        // Compute the calibration and get the final residual value
        std::pair<double, AnglePositionVector> estimation = EstimateCalibrationFromPoses(
                                                                       targetPosesCurrCombination, sourceSensor);

        // Store the combination try
        Eigen::Vector3d Angles = estimation.second.segment(0, 3);
        Eigen::Vector3d T = estimation.second.segment(3, 3);
        CombinationTry currentTry(combinationCount, anglesOrder, matrixOrder, sign,
                                  Angles, T, estimation.first);
        results.push_back(currentTry);

        combinationCount++;
      } // Loop over the angles signs
      shouldPermutMatrix = std::next_permutation(matrixOrder, matrixOrder + 3);
    } // Loop over matrix multiplication order
    shouldPermutAngles = std::next_permutation(anglesOrder, anglesOrder + 3);
  } // Loop over the angles positions permutations

  // Now, store the combination try using the final residual value
  std::sort(results.begin(), results.end(), [](CombinationTry const& a, CombinationTry const& b)
  {
     return a.finalErrorValue <= b.finalErrorValue;
  });
  for (unsigned int index = 0; index < results.size(); ++index)
  {
    results[index].PrintSelf();
  }
}

//-----------------------------------------------------------------------------
std::pair<double, AnglePositionVector> MatchTrajectoriesWithIsometry(vtkSmartPointer<vtkTemporalTransforms> sourceSensor,
                                                                     vtkSmartPointer<vtkTemporalTransforms> targetSensor)
{
  // Create the transforms interpolators
  vtkSmartPointer<vtkVelodyneTransformInterpolator> sourceSensorTransforms = sourceSensor->CreateInterpolator();
  vtkSmartPointer<vtkVelodyneTransformInterpolator> targetSensorTransforms = targetSensor->CreateInterpolator();
  sourceSensorTransforms->SetInterpolationTypeToLinear();
  targetSensorTransforms->SetInterpolationTypeToLinear();

  // Time interval
  double tmin = std::max(sourceSensorTransforms->GetMinimumT(), targetSensorTransforms->GetMinimumT());
  double tmax = std::min(sourceSensorTransforms->GetMaximumT(), targetSensorTransforms->GetMaximumT());
  const double deltaTime = 0.2; // 200ms

  // usefull transform
  auto currTransform = vtkSmartPointer<vtkTransform>::New();

  ceres::Problem problem;
  Eigen::Vector3d X, Y;
  AnglePositionVector transformParams = AnglePositionVector::Zero();

  // Loop over the time
  for (double time = tmin; time < tmax; time += deltaTime)
  {
    // Get the position of the sensor 1 for time
    sourceSensorTransforms->InterpolateTransform(time, currTransform);
    Y = GetPoseParamsFromTransform(currTransform).second;
    // Get the position of the sensor 2 for time
    targetSensorTransforms->InterpolateTransform(time, currTransform);
    X = GetPoseParamsFromTransform(currTransform).second;

    // Add the geometric contraint residual function
    // to the non-linear least square problem
    // add this geometric constraint non-linear least square residu to the global
    // cost function that is the sum of all residuals functions
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctions::EuclideanDistanceAffineIsometryResidual, 1, 6>
              (new CostFunctions::EuclideanDistanceAffineIsometryResidual(X, Y));
    problem.AddResidualBlock(cost_function, nullptr, transformParams.data());
  }

  // Solve the optimization problem
  // Option of the solver
  ceres::Solver::Options options;
  options.max_num_iterations = 75;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << ", Number Blocks: " << summary.num_residuals << std::endl;

  return std::pair<double, AnglePositionVector>(summary.final_cost, transformParams);
}

//-----------------------------------------------------------------------------
std::pair<double, AnglePositionVector> MatchTrajectoriesWithIsometry(const std::string& sourceSensorFilename,
                                                                     const std::string& targetSensorFilename)
{
  vtkSmartPointer<vtkTemporalTransforms> trans1, trans2;
  trans1 = vtkTemporalTransformsReader::OpenTemporalTransforms(sourceSensorFilename);
  trans2 = vtkTemporalTransformsReader::OpenTemporalTransforms(targetSensorFilename);
  return MatchTrajectoriesWithIsometry(trans1, trans2);
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> MatchTrajectoriesWithIsometryAndApply(
                                                         vtkSmartPointer<vtkTemporalTransforms> targetSensor,
                                                         vtkSmartPointer<vtkTemporalTransforms> sourceSensor)
{
  // Estimate the isometry that match the source trajectory on the target one
  std::pair<double, AnglePositionVector> estimation = MatchTrajectoriesWithIsometry(targetSensor, sourceSensor);
  // Transform the source trajectory
  std::pair<Eigen::Vector3d, Eigen::Vector3d> dof6(estimation.second.segment(0, 3), estimation.second.segment(3, 3));
  vtkSmartPointer<vtkTransform> transform = GetTransformFromPosesParams(dof6);
  // return the result
  return sourceSensor->IsometricTransform(transform);
}

//-----------------------------------------------------------------------------
void CreateSyntheticPosesData(const std::string& vehiclePosesFilename,
                              const std::string& imuPosesFilename,
                              const std::string& sensorPosesFilename)
{
  // Controls the number of point and the time
  // interval of the mock up datas
  double timeBounds[2] = {0, 20.0};
  double Npts = 2500.0;
  double dt = (timeBounds[1] - timeBounds[0]) / Npts;

  // Local usefull macro
  #define rad vtkMath::RadiansFromDegrees
  #define uRandom (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5)

  // Positions of the sensors in the vehicle reference frame
  Eigen::Vector3d Position0(0.45, 1.24, 0.078);
  Eigen::Vector3d Position1(2.78, 1.98, 1.04);

  // Orientations of the sensors in the vehicle reference frame
  Eigen::Matrix3d Orientation0 = RollPitchYawToMatrix(Eigen::Vector3d(rad(12.5), rad(-24.5), rad(45.0)));
  Eigen::Matrix3d Orientation1 = RollPitchYawToMatrix(Eigen::Vector3d(rad(26.5), rad(4.5), rad(75.0)));

  // The "orientation trajectory" will be a Bezier curve defined
  // by 4 controls points in the euler-angles space trajectory
  Eigen::Vector3d ainit(rad(13.45), rad(-20.45), rad(108.0));
  Eigen::Vector3d a0(rad(0.0), rad(0.0), rad(0.0));
  Eigen::Vector3d a1(rad(20.5), rad(45.0), rad(90.0));
  Eigen::Vector3d a2(rad(33.5), rad(-45.0), rad(180.0));
  Eigen::Vector3d a3(rad(0.0), rad(12.5), rad(270.0));

  // The trajectory will be a Bezier curve defined
  // by 4 controls points in 3D space.
  Eigen::Vector3d pinit(35.4, -12.45, 7.45);
  Eigen::Vector3d p0(0, 0, 0);
  Eigen::Vector3d p1(50, 50, 10);
  Eigen::Vector3d p2(50, -50, 20);
  Eigen::Vector3d p3(0, 0, 30);

  // Initialze the seed of the random
  // generator to add simulated noise
  // to the data
  std::srand(1992);
  double noiseFactorMeter = 0.05; // 5 cm error max
  double noiseFactorRad = 0.017; // 1 degree error max

  // export created data
  std::ofstream file0, file1, file2;
  file0.open(vehiclePosesFilename.c_str());
  file1.open(imuPosesFilename.c_str());
  file2.open(sensorPosesFilename.c_str());
  file0 << "Time,Rx(Roll),Ry(Pitch),Rz(Yaw),X,Y,Z" << std::endl;
  file1 << "Time,Rx(Roll),Ry(Pitch),Rz(Yaw),X,Y,Z" << std::endl;
  file2 << "Time,Rx(Roll),Ry(Pitch),Rz(Yaw),X,Y,Z" << std::endl;

  // Initial oorientation and position
  // of the SLAM sensor
  Eigen::Matrix3d Sensor2R_t0;
  Eigen::Vector3d Sensor2T_t0;

  for (double t = timeBounds[0]; t <= timeBounds[1]; t += dt)
  { 
    double u = (t - timeBounds[0]) / (timeBounds[1] - timeBounds[0]);
    double s = (1.0 - u);
    double ss = s * s; double uu = u * u;
    double sss = ss * s; double uuu = uu * u;

    // trajectory of the vehicle
    Eigen::Vector3d vehicleT = sss * p0 + ss * u * p1 + s * uu * p2 + uuu * p3 + pinit;
    vehicleT(0) += 0.10 * cos(2 * vtkMath::Pi() * 0.5 * t);
    vehicleT(1) += 0.05 * sin(2 * vtkMath::Pi() * 0.25 * t);
    vehicleT(2) += 0.07 * sin(2 * vtkMath::Pi() * 0.75 * t);

    // Orientation of the vehicle
    Eigen::Vector3d vehicleA = sss * a0 + ss * u * a1 + s * uu * a2 + uuu * a3 + ainit;
    vehicleA(0) += rad(12.5) * cos(2 * vtkMath::Pi() * 0.17 * t);
    vehicleA(1) += rad(7.45) * sin(2 * vtkMath::Pi() * 0.14 * t);
    vehicleA(2) += rad(8.98) * sin(2 * vtkMath::Pi() * 0.09 * t);
    Eigen::Matrix3d vehicleR = RollPitchYawToMatrix(vehicleA);

    // Orientation and position of the sensor 0 (IMU)
    Eigen::Vector3d T0 = vehicleR * Position0 + vehicleT;
    Eigen::Matrix3d R0 = vehicleR * Orientation0;

    // Orientation and position of the sensor 1 (SLAM - LiDAR)
    Eigen::Vector3d T1 = vehicleR * Position1 + vehicleT;
    Eigen::Matrix3d R1 = vehicleR * Orientation1;

    // store the first transform
    if (std::abs(t - timeBounds[0]) < 1e-14)
    {
      Sensor2R_t0 = R1; Sensor2T_t0 = T1;
    }

    // Express the poses of the sensor 2 in its
    // initial reference coordinate frame to simulate
    // SLAM data
    T1 = Sensor2R_t0.transpose() * (T1 - Sensor2T_t0);
    R1 = Sensor2R_t0.transpose() * R1;

    Eigen::Vector3d A0 = MatrixToRollPitchYaw(R0);
    Eigen::Vector3d A1 = MatrixToRollPitchYaw(R1);

    // Generate random noise in data
    for (unsigned int k = 0; k < 3; ++k)
    {
      T0(k) += noiseFactorMeter * uRandom;
      T1(k) += noiseFactorMeter * uRandom;
      A0(k) += noiseFactorRad * uRandom;
      A1(k) += noiseFactorRad * uRandom;
    }

    file0 << t << "," << vehicleA(0) << "," << vehicleA(1) << "," << vehicleA(2) << ","
          << vehicleT(0) << "," << vehicleT(1) << "," << vehicleT(2) << "," << std::endl;

    file1 << t << "," << A0(0) << "," << A0(1) << "," << A0(2) << ","
          << T0(0) << "," << T0(1) << "," << T0(2) << "," << std::endl;

    file2 << t << "," << A1(0) << "," << A1(1) << "," << A1(2) << ","
          << T1(0) << "," << T1(1) << "," << T1(2) << "," << std::endl;
  }
  file0.close();
  file1.close();
  file2.close();

  // Compute the calibration between the two sensors
  Eigen::Matrix3d Rcalib = Orientation0.transpose() * Orientation1;
  Eigen::Vector3d AnglesCalib = MatrixToRollPitchYaw(Rcalib) / vtkMath::Pi() * 180.0;
  Eigen::Vector3d Tcalib = Orientation0.transpose() * (Position1 - Position0);

  std::cout << "Calibration euler angles used for synthetic data: " << AnglesCalib.transpose() << std::endl;
  std::cout << "Calibration baseline used for synthetic data: " << Tcalib.transpose() << std::endl;
}
