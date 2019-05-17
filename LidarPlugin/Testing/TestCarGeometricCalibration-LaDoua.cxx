#include <vtkMath.h>

#include "vtkEigenTools.h"
#include "vtkTemporalTransforms.h"
#include "vtkTemporalTransformsReader.h"
#include "vtkCarGeometricCalibration.h"

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    return 1;
  }
  int errors = 0;
  std::string referenceFile;
  std::string alignedFile;
  vtkSmartPointer<vtkTemporalTransforms> r, a;
  Eigen::Matrix3d R1, R2;
  double scale;
  bool rotationValid = false;
  bool verbose = true;

  Eigen::Vector3d R_gt_ = (vtkMath::Pi() / 180.0) * Eigen::Vector3d(94.16, -175.09, 108.50);
  Eigen::Matrix3d R_gt = RollPitchYawToMatrix(R_gt_);
  double angular_error_tol = 1.5;


  // First dataset:

  referenceFile = std::string(argv[1]) + "/slam_lidar_part2.csv";
  alignedFile = std::string(argv[1]) + "/orbslam2_gopro_part2.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)->ApplyTimeshift(1302.343);
  ComputeCarCalibrationRotationScale(r, a, 0.15, 1000, 5.0, 0.1, 0.15, R1, scale, rotationValid, verbose);
  errors += rotationValid ? 0 : 1;
  Eigen::Matrix3d difference1 = R1 * R_gt.transpose();
  auto aa1 = Eigen::AngleAxisd(difference1);
  double angularError1 = (180.0 / vtkMath::Pi()) * aa1.angle();
  errors += angularError1 < angular_error_tol ? 0 : 1;

  Eigen::Vector3d yprDegree1 = (180.0 / vtkMath::Pi()) * R1.eulerAngles(2,1,0);
  std::cout << "angles: " << yprDegree1[2] << ", " << yprDegree1[1] << ", " << yprDegree1[0] << std::endl;


  // Second dataset:

  referenceFile = std::string(argv[1]) + "/slam_lidar_part1.csv";
  alignedFile = std::string(argv[1]) + "/orbslam2_gopro_part1.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)->ApplyTimeshift(1302.343);
  ComputeCarCalibrationRotationScale(r, a, 0.15, 1000, 5.0, 0.1, 0.15, R2, scale, rotationValid, verbose);
  errors += rotationValid ? 0 : 1;
  Eigen::Matrix3d difference2 = R2 * R_gt.transpose();
  auto aa2 = Eigen::AngleAxisd(difference2);
  double angularError2 = (180.0 / vtkMath::Pi()) * aa2.angle();
  errors += angularError2 < angular_error_tol ? 0 : 1;

  Eigen::Vector3d yprDegree2 = (180.0 / vtkMath::Pi()) * R2.eulerAngles(2,1,0);
  std::cout << "angles: " << yprDegree2[2] << ", " << yprDegree2[1] << ", " << yprDegree2[0] << std::endl;


  Eigen::Matrix3d difference = R1 * R2.transpose();
  auto aa = Eigen::AngleAxisd(difference);
  double angularError = (180.0 / vtkMath::Pi()) * aa.angle();
  std::cout << "difference in degrees between the two datasets: " << angularError << std::endl;
  errors += angularError < 2.0 ? 0 : 1;

  std::cout << errors << std::endl;

  return errors == 0 ? 0 : 1;
}
