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

// VTK
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkSmartPointer.h>

// EIGEN
#include <Eigen/Dense>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// LOCAL
#include "vtkEigenTools.h"
#include "TrajectoryReoptimization.h"
#include "CameraCalibration.h"
#include "CameraProjection.h"
#include "vtkEigenTools.h"
#include "vtkPCLConversions.h"
#include "vtkConversions.h"
#include "RegistrationTools.h"
#include "vtkTemporalTransforms.h"
#include "vtkCustomTransformInterpolator.h"
#include "CeresTools.h"
#include "vtkConversions.h"


//----------------------------------------------------------------------------
int TestRelocation(std::string referenceFilename, std::string toAlignFilename)
{
  // Load reference point cloud data
  vtkNew<vtkXMLPolyDataReader> readerCloudReference;
  readerCloudReference->SetFileName(referenceFilename.c_str());
  readerCloudReference->Update();
  vtkSmartPointer<vtkPolyData> vtkReferenceCloud = readerCloudReference->GetOutput();

  // load to align cloud data
  vtkNew<vtkXMLPolyDataReader> readerCloudToAligned;
  readerCloudToAligned->SetFileName(toAlignFilename.c_str());
  readerCloudToAligned->Update();
  vtkSmartPointer<vtkPolyData> vtkToAlignedCloud = readerCloudToAligned->GetOutput();

  // Convert vtk data into pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclReferenceCloud = vtkPCLConversions::PointCloudFromPolyData(vtkReferenceCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclToAlignedCloud = vtkPCLConversions::PointCloudFromPolyData(vtkToAlignedCloud);

  // Apply a voxel grid filter since the density of point
  // is not homogene
  double leaf = 0.6;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredReference(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredToAligned(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
  voxelGridFilter.setLeafSize(leaf, leaf, leaf);
  voxelGridFilter.setInputCloud(pclReferenceCloud);
  voxelGridFilter.filter(*filteredReference);
  voxelGridFilter.setInputCloud(pclToAlignedCloud);
  voxelGridFilter.filter(*filteredToAligned);

  // Compute the relocation
  Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
  std::vector<bool> pointsUsed;
  Eigen::Matrix4d Href = ICPPointToPlaneRegistration(filteredReference, filteredToAligned, H, pointsUsed, 10, 30);

  // compare obtained relocation and expected relocation
  Eigen::Matrix<double, 6, 1> DoF6Params;;
  DoF6Params.block(0, 0, 3, 1) = MatrixToRollPitchYaw(Href.block(0, 0, 3, 3));
  DoF6Params.block(3, 0, 3, 1) = Href.block(0, 3, 3, 1);;
  Eigen::Matrix<double, 6, 1> ExpectedDoF6Params;
  ExpectedDoF6Params << 0.00673364, 0.00140092, -0.0648394, 1.78213, 5.36607, -0.796278;

  double epsilon = 1e-2;
  for (int i = 0; i < 6; ++i)
  {
    if (std::abs(DoF6Params[i] - ExpectedDoF6Params[i]) / std::abs(ExpectedDoF6Params[i]) > epsilon)
    {
      std::cout << "got: " << DoF6Params[i] << " expected: " << ExpectedDoF6Params[i] << std::endl;
      return 1;
    }
  }

  return 0;
}

//----------------------------------------------------------------------------
int TestTrajectoryReoptimizationLoopClosure(std::string trajFileName, std::string expectedReoptimizedTraj)
{
  int errors = 0;

  // read input data
  vtkNew<vtkXMLPolyDataReader> reader;
  reader->SetFileName(trajFileName.c_str());
  reader->Update();

  // get the trajectory
  vtkSmartPointer<vtkPolyData> inputTraj = reader->GetOutput();

  // Anchor computed from relocation after a loop-closure
  Eigen::Matrix4d Hanchor = Eigen::Matrix4d::Identity();
  Hanchor <<     0.997898,   0.0648019,  0.000961645,    1.78213,
               -0.0647939,    0.997875,  -0.00681021,    5.36607,
              -0.00140092,  0.00673359,     0.999976,  -0.796278,
                        0,           0,            0,          1;

  // populate the poses
  PoseEstimationVector absolutePoses;
  ConvertPolyDataToPoseEstimation(inputTraj, absolutePoses);

  Hanchor = Hanchor * absolutePoses[absolutePoses.size() - 1].H;

  // Optimization
  PoseEstimationVector correctedPoses;
  correctedPoses = TrajectoryReoptimization(absolutePoses, Hanchor, inputTraj->GetNumberOfPoints() - 1, 3);

  // create arrays
  vtkNew<vtkDoubleArray> mahalanobis;
  vtkNew<vtkDoubleArray> anglesDist;
  vtkNew<vtkDoubleArray> positionDist;
  // name
  mahalanobis->SetName("Mahalanobis distance");
  anglesDist->SetName("Angle distance");
  positionDist->SetName("Position distance");
  // components
  mahalanobis->SetNumberOfComponents(1);
  anglesDist->SetNumberOfComponents(1);
  positionDist->SetNumberOfComponents(1);
  // Allocate
  mahalanobis->SetNumberOfTuples(inputTraj->GetNumberOfPoints());
  anglesDist->SetNumberOfTuples(inputTraj->GetNumberOfPoints());
  positionDist->SetNumberOfTuples(inputTraj->GetNumberOfPoints());

  // Convert to vtk
  vtkNew<vtkPolyData> correctedTraj;
  correctedTraj->DeepCopy(inputTraj);
  vtkDataArray* orientation = correctedTraj->GetPointData()->GetArray("Orientation(AxisAngle)");
  for (int poseIndex = 0; poseIndex < inputTraj->GetNumberOfPoints(); ++poseIndex)
  {
    // Orientation
    Eigen::Matrix3d R = correctedPoses[poseIndex].H.block(0, 0, 3, 3);
    Eigen::AngleAxisd angleAxis(R);
    orientation->SetTuple4(poseIndex, angleAxis.axis()(0), angleAxis.axis()(1), angleAxis.axis()(2), angleAxis.angle());

    // Pose
    Eigen::Vector3d T = correctedPoses[poseIndex].H.block(0, 3, 3, 1);
    correctedTraj->GetPoints()->SetPoint(poseIndex, T.data());

    // Fill reoptimization information
    mahalanobis->SetTuple1(poseIndex, correctedPoses[poseIndex].MahalanobisDistance);
    anglesDist->SetTuple1(poseIndex, correctedPoses[poseIndex].AnglesDistance);
    positionDist->SetTuple1(poseIndex, correctedPoses[poseIndex].PositionDistance);
  }
  correctedTraj->GetPointData()->AddArray(mahalanobis);
  correctedTraj->GetPointData()->AddArray(anglesDist);
  correctedTraj->GetPointData()->AddArray(positionDist);

  // Load the expected result
  reader->SetFileName(expectedReoptimizedTraj.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyData> expectedTraj = reader->GetOutput();
  vtkDataArray* expectedOrientation = inputTraj->GetPointData()->GetArray("Orientation(AxisAngle)");

  // Compare the results
  double epsilon = 1e-6;
  for (int pointIdx = 0; pointIdx < expectedTraj->GetNumberOfPoints(); ++pointIdx)
  {
    double pt0[3], pt1[3];
    correctedTraj->GetPoint(pointIdx, pt0);
    expectedTraj->GetPoint(pointIdx, pt1);

    for (int i = 0; i < 3; ++i)
    {
      if (std::abs(pt0[i] - pt1[i]) / std::abs(pt1[i]) > epsilon)
      {
        std::cout << "error, expected: " << pt1[i] << " got: " << pt0[i] << std::endl;
        return 1;
      }
    }
  }

  return errors;
}

//----------------------------------------------------------------------------
int TestTrajectoryReoptimizationGPSIMU(std::string trajFileName, std::string gpsIMUTrajFileName)
{
  int errors = 0;

  // SLAM trajectory
  vtkNew<vtkXMLPolyDataReader> reader;
  reader->SetFileName(trajFileName.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyData> slamTraj = reader->GetOutput();

  // GPS / IMU trajectory
  vtkNew<vtkXMLPolyDataReader> reader2;
  reader2->SetFileName(gpsIMUTrajFileName.c_str());
  reader2->Update();
  vtkSmartPointer<vtkPolyData> imuTraj = reader2->GetOutput();

  // Create a linear interpolator from the imu trajectory
  vtkSmartPointer<vtkTemporalTransforms> imuTransforms = vtkTemporalTransforms::CreateFromPolyData(imuTraj);
  vtkSmartPointer<vtkCustomTransformInterpolator> imuInterp = imuTransforms->CreateInterpolator();
  imuInterp->SetInterpolationTypeToLinear();

  // Now, take some anchors timestamp and compute the corresponding
  // anchor sensor pose using the imu trajectory and linearly interpolate
  // the pose between two samples
  std::vector<Eigen::Matrix4d> Hanchors;
  std::vector<int> anchorsIndex;
  int Nanchors = 15;
  for (int i = 1; i < Nanchors + 1; ++i)
  {
    // get tupleId an associated time
    int tupleId = std::floor(static_cast<double>(i * (slamTraj->GetNumberOfPoints() - 1)) / static_cast<double>(Nanchors));
    double time = slamTraj->GetPointData()->GetArray("Time")->GetTuple1(tupleId);
    time += 252018.0000; // time offset between the two clocks

    // Get the anchor associated to this timestamp
    vtkNew<vtkTransform> vtkH;
    imuInterp->InterpolateTransform(time, vtkH.Get());
    vtkNew<vtkMatrix4x4> M;
    vtkH->GetMatrix(M.Get());
    Eigen::Matrix4d currentHanchor;
    for (int col = 0; col < 4; col++)
    {
      for (int raw = 0; raw < 4; ++raw)
      {
        currentHanchor(raw, col) = M->Element[raw][col];
      }
    }
    Hanchors.push_back(currentHanchor);
    anchorsIndex.push_back(tupleId);
  }

  // populate the poses
  PoseEstimationVector absolutePoses;
  ConvertPolyDataToPoseEstimation(slamTraj, absolutePoses);

  // Optimization
  PoseEstimationVector correctedPoses;
  correctedPoses = TrajectoryReoptimization(absolutePoses, Hanchors, anchorsIndex, 1, MeasureProvided::PositionOnly);

  // Convert to vtk
  vtkNew<vtkPolyData> correctedTraj;
  correctedTraj->DeepCopy(slamTraj);
  vtkDataArray* orientation = correctedTraj->GetPointData()->GetArray("Orientation(AxisAngle)");
  for (int poseIndex = 0; poseIndex < slamTraj->GetNumberOfPoints(); ++poseIndex)
  {
    // Orientation
    Eigen::Matrix3d R = correctedPoses[poseIndex].H.block(0, 0, 3, 3);
    Eigen::AngleAxisd angleAxis(R);
    orientation->SetTuple4(poseIndex, angleAxis.axis()(0), angleAxis.axis()(1), angleAxis.axis()(2), angleAxis.angle());

    // Pose
    Eigen::Vector3d T = correctedPoses[poseIndex].H.block(0, 3, 3, 1);
    correctedTraj->GetPoints()->SetPoint(poseIndex, T.data());
  }
}

//----------------------------------------------------------------------------
int TestComputeSimilitude()
{
  std::vector<Eigen::Vector3d> X, Y;
  X.push_back(Eigen::Vector3d(784.151,229.155,35.5491));
  Y.push_back(Eigen::Vector3d(1227,604,100));
  X.push_back(Eigen::Vector3d(958.585,135.759,48.987));
  Y.push_back(Eigen::Vector3d(1432,424,-12));
  X.push_back(Eigen::Vector3d(796.33,-7.14873,51.3654));
  Y.push_back(Eigen::Vector3d(1234,306,14));
  X.push_back(Eigen::Vector3d(381.26,-166.006,37.2953));
  Y.push_back(Eigen::Vector3d(712,104,12.5));
  X.push_back(Eigen::Vector3d(278.693,146.043,13.9819));
  Y.push_back(Eigen::Vector3d(584,497,-1.14));
  X.push_back(Eigen::Vector3d(88.9332,33.0731,4.91454));
  Y.push_back(Eigen::Vector3d(343,356,1.454));

  Eigen::Matrix<double, 9, 1> W = Eigen::Matrix<double, 9, 1>::Zero();
  ComputeSimilitude(X, Y, W);

  Eigen::Matrix<double, 9, 1> Wexpected;
  Wexpected << -0.024524,0.061816,-0.0237559,186.127,248.167,-11.0479,1.25261,1.27832,-1.35793;
  for (int i = 0; i < 9; ++i)
  {
    if (std::abs(W(i) - Wexpected(i)) > 1e-2)
    {
      std::cout << "Got: " << W(i) << " expected: " << Wexpected(i) << std::endl;
      return 1;
    }
  }

  return 0;
}

//----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    return 1;
  }

  int errors = 0;
  std::string referenceFilename = std::string(argv[1]) + "/ReferenceCloudFiltered.vtp";
  std::string toAlignedFilename = std::string(argv[1]) + "/ToAlignedCloudFiltered.vtp";

  std::string rawTrajFileName = std::string(argv[1]) + "/TrajectoryWithVarianceCovariance.vtp";
  std::string optimizedTrajFileName = std::string(argv[1]) + "/ExpectedReoptimization.vtp";

  std::string gpsIMUTrajFileName = std::string(argv[1]) + "/Lidar_Calibrated.vtp";

  errors += TestComputeSimilitude();
  errors += TestRelocation(referenceFilename, toAlignedFilename);
  errors += TestTrajectoryReoptimizationLoopClosure(rawTrajFileName, optimizedTrajFileName);
  errors += TestTrajectoryReoptimizationGPSIMU(rawTrajFileName, gpsIMUTrajFileName);

  return errors;
}
