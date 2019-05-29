#include "vtkGeometricCalibration.h"
#include "vtkTemporalTransformsReader.h"
#include "vtkEigenTools.h"
#include "vtkTimeCalibration.h"

#include <vtkMath.h>

int main(int argc, char* argv[])
{
  // number of error in the test
  int errors = 0;
  const double mm03_gt = -399618.0; // ground truth quality (+- 1e-4)
  const double mm04_gt = -252018.0; // ground truth quality (+- 1e-4)

  // Check that the correct number of
  // arguments is provided
  if (argc != 2)
  {
    return 1;
  }

  // get command line filenames parameters
  std::string referenceFile;
  std::string alignedFile;
  vtkSmartPointer<vtkTemporalTransforms> complete_r, complete_a, r, a;

  referenceFile = std::string(argv[1]) + "/mm03/imu.csv";
  alignedFile = std::string(argv[1]) + "/mm03/lidar-slam.csv";
  complete_r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile);
  r = complete_r->ExtractTimes(400274, 400274+180)->Subsample(10);
  complete_a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile);
  a = complete_a->ExtractTimes(610, 610+180)->ApplyTimeshift(- mm03_gt);;

  errors += std::abs(1.0 - ComputeScale(r, a, CorrelationStrategy::SPEED_WINDOW, 1.0)) > 0.01;
  errors += std::abs(1.0 - ComputeScale(r, a, CorrelationStrategy::JERK_WINDOW, 6.0)) > 0.05;
  errors += std::abs(1.0 - ComputeScale(r, a, CorrelationStrategy::DPOS, 1.0)) > 0.01;
  errors += std::abs(1.0 - ComputeScale(r, a, CorrelationStrategy::ACC_WINDOW, 3.0)) > 0.05;
  errors += std::abs(1.0 - ComputeScale(r, a, CorrelationStrategy::DERIVATED_LENGTH, 1.0)) > 0.01;

  referenceFile = std::string(argv[1]) + "/mm04/imu.csv";
  alignedFile = std::string(argv[1]) + "/mm04/lidar-slam.csv";
  r = vtkTemporalTransformsReader::OpenTemporalTransforms(referenceFile)->ExtractTimes(255064, 255064+180)->Subsample(10);
  a = vtkTemporalTransformsReader::OpenTemporalTransforms(alignedFile)->ExtractTimes(3090, 3090+180)->ApplyTimeshift(- mm04_gt);

  errors += std::abs(1.0 - ComputeScale(r, a, CorrelationStrategy::SPEED_WINDOW, 1.0)) > 0.01;
  std::cout << errors << std::endl;

  return errors;
}
