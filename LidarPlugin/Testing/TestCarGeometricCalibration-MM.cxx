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
  vtkSmartPointer<vtkTemporalTransforms> r, a;
  Eigen::Matrix3d R1, R2;
  double scale;
  bool rotationValid;

  Eigen::Vector3d R_gt_ = (vtkMath::Pi() / 180.0) * Eigen::Vector3d(179.788, 179.891, 87.718);
  Eigen::Matrix3d R_gt = RollPitchYawToMatrix(R_gt_);
  double angular_error_tol = 2.1;

  const double mm03_gt = -399618.0; // ground truth quality (+- 1e-4)
  const double mm04_gt = -252018.0; // ground truth quality (+- 1e-4)

  std::string referenceFile = std::string(argv[1]) + "/mm03/imu.csv";
  std::string alignedFile = std::string(argv[1]) + "/mm03/lidar-slam.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)->ApplyTimeshift(- mm03_gt);
  rotationValid = false;
  ComputeCarCalibrationRotationScale(r, a, 0.15, 10000, 4.0, 0.1, 0.5, R1, scale, rotationValid, true);
  errors += rotationValid ? 0 : 1;
  Eigen::Matrix3d difference1 = R1 * R_gt.transpose();
  auto aa1 = Eigen::AngleAxisd(difference1);
  double angularError1 = (180.0 / vtkMath::Pi()) * aa1.angle();
  std::cout << angularError1 << std::endl;
  errors += angularError1 < angular_error_tol ? 0 : 1;

  Eigen::Vector3d yprDegree1 = (180.0 / vtkMath::Pi()) * R1.eulerAngles(2,1,0);
  std::cout << "angles: " << yprDegree1[2] << ", " << yprDegree1[1] << ", " << yprDegree1[0] << std::endl;

  referenceFile = std::string(argv[1]) + "/mm04/imu.csv";
  alignedFile = std::string(argv[1]) + "/mm04/lidar-slam.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)->ApplyTimeshift(- mm04_gt);
  rotationValid = false;
  ComputeCarCalibrationRotationScale(r, a, 0.15, 1000, 5.0, 0.1, 0.5, R2, scale, rotationValid, true);
  errors += rotationValid ? 0 : 1;
  Eigen::Matrix3d difference2 = R2 * R_gt.transpose();
  auto aa2 = Eigen::AngleAxisd(difference2);
  double angularError2 = (180.0 / vtkMath::Pi()) * aa2.angle();
  std::cout << angularError2 << std::endl;
  errors += angularError2 < angular_error_tol ? 0 : 1;

  Eigen::Vector3d yprDegree2 = (180.0 / vtkMath::Pi()) * R2.eulerAngles(2,1,0);
  std::cout << "angles: " << yprDegree2[2] << ", " << yprDegree2[1] << ", " << yprDegree2[0] << std::endl;

  return errors;
}
