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

#include "vtkGeometricCalibration.h"
#include "vtkTemporalTransformsReader.h"
#include "vtkEigenTools.h"

#include <vtkMath.h>

int main(int argc, char* argv[])
{
  // Check that the correct number of
  // arguments is provided
  if (argc != 2)
  {
    return 1;
  }

  // number of error in the test
  int errors = 0;
  const double mm03_gt = -399618.0; // ground truth quality (+- 1e-4)
  const double mm04_gt = -252018.0; // ground truth quality (+- 1e-4)

  Eigen::Vector3d R_gt_ = (vtkMath::Pi() / 180.0) * Eigen::Vector3d(179.788, 179.891, 87.718);
  Eigen::Matrix3d R_gt = RollPitchYawToMatrix(R_gt_);
  double angular_error_tol = 1.5;

  // get command line filenames parameters
  std::string referenceFile;
  std::string alignedFile;
  vtkSmartPointer<vtkTemporalTransforms> r, a;


  // First dataset:

  referenceFile = std::string(argv[1]) + "/mm03/imu.csv";
  alignedFile = std::string(argv[1]) + "/mm03/lidar-slam.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)->ApplyTimeshift(- mm03_gt);

  std::pair<double, AnglePositionVector> calib = EstimateCalibrationFromPoses(r, a);
  Eigen::Matrix3d R1 = RollPitchYawToMatrix(calib.second(0), calib.second(1), calib.second(2));
  Eigen::Matrix3d difference1 = R1 * R_gt.transpose();
  auto aa1 = Eigen::AngleAxisd(difference1);
  double angularError1 = (180.0 / vtkMath::Pi()) * aa1.angle();
  errors += angularError1 < angular_error_tol ? 0 : 1;

  Eigen::Vector3d yprDegree1 = (180.0 / vtkMath::Pi()) * R1.eulerAngles(2,1,0);
  std::cout << "angles: " << yprDegree1[2] << ", " << yprDegree1[1] << ", " << yprDegree1[0] << std::endl;
  std::cout << "pos: " << std::abs(calib.second(3))
            << ", " << std::abs(calib.second(4))
            << ", " << std::abs(calib.second(5)) << std::endl;


  // Second dataset:

  referenceFile = std::string(argv[1]) + "/mm04/imu.csv";
  alignedFile = std::string(argv[1]) + "/mm04/lidar-slam.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)->ApplyTimeshift(- mm04_gt);

  calib = EstimateCalibrationFromPoses(r, a);
  Eigen::Matrix3d R2 = RollPitchYawToMatrix(calib.second(0), calib.second(1), calib.second(2));
  Eigen::Matrix3d difference2 = R2 * R_gt.transpose();
  auto aa2 = Eigen::AngleAxisd(difference2);
  double angularError2 = (180.0 / vtkMath::Pi()) * aa2.angle();
  errors += angularError2 < angular_error_tol ? 0 : 1;

  Eigen::Vector3d yprDegree2 = (180.0 / vtkMath::Pi()) * R2.eulerAngles(2,1,0);
  std::cout << "angles: " << yprDegree2[2] << ", " << yprDegree2[1] << ", " << yprDegree2[0] << std::endl;
  std::cout << "pos: " << std::abs(calib.second(3))
            << ", " << std::abs(calib.second(4))
            << ", " << std::abs(calib.second(5)) << std::endl;

  return errors;
}
