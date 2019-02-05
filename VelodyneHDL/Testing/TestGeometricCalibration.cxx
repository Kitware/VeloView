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

// VTK
#include <vtkMath.h>

/**
 * @brief Test that the estimation of the calibration is correct
 * @param filename1 Data corresponding to the poses of the sensor 1
 * @param filename2 data corresponding to the poses of the sensor 2
 * @return 0 on success, >= 1 on failure
 */
int main(int argc, char* argv[])
{
  // number of error in the test
  int nbrError = 0;

  // Check that the correct number of
  // arguments is provided
  if (argc < 2)
  {
    std::cerr << "Wrong number of arguments. Usage: filename1 filename2" << std::endl;
    return 1;
  }

  // get command line filenames parameters
  std::string filename1 = argv[1];
  std::string filename2 = argv[2];

  // Estimate the calibration
  std::pair<double, AnglePositionVector> calib = EstimateCalibrationFromPoses(filename1, filename2);

  double groundtruthPosition = {2.3743, -1.10303, 0.219091};
  double groundtruthAngles = {27.5107, 31.8117, 27.5998};
  for (unsigned int k = 0; k < 3; ++k)
  {
    // 1 degree tolerance
    if (std::abs(calib.second(k) / vtkMath::Pi() * 180.0 - groundtruthAngles[k]) > 1.0)
      nbrError++;
    // 10 cm tolerance
    if (std::abs(calib.second(k + 3) - groundtruthPosition[k]) > 0.1)
      nbrError++;
  }

  return nbrError;
}
