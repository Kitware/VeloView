#include <vtkMath.h>

#include "vtkGeometricCalibration.h"
#include "vtkTemporalTransformsReader.h"
#include "vtkCarGeometricCalibration.h"
#include "vtkEigenTools.h"

#include "vtkTimeCalibration.h" // TODO: remove

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

  Eigen::Vector3d R_gt_ = (vtkMath::Pi() / 180.0) * Eigen::Vector3d(94.16, -175.09, 108.50);
  Eigen::Matrix3d R_gt = RollPitchYawToMatrix(R_gt_);
  double angular_error_tol = 1.5;


  // First dataset

  referenceFile = std::string(argv[1]) + "/slam_lidar_part2.csv";
  alignedFile = std::string(argv[1]) + "/orbslam2_gopro_part2.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  // we do not forget to correct the scale because EstimateCalibrationFromPoses
  // requires the trajectories to be scaled and our orbslam2 was produced with
  // a monocular setup.
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)
      ->ApplyTimeshift(1302.343);

  a = a->ApplyScale(1.0 / 0.0120337);

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


  // Second dataset

  referenceFile = std::string(argv[1]) + "/slam_lidar_part1.csv";
  alignedFile = std::string(argv[1]) + "/orbslam2_gopro_part1.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)
      ->ApplyTimeshift(1302.343);
  a = a->ApplyScale(1.0 / 0.0291194);

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
