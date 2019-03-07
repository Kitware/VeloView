// Copyright 2013 Velodyne Acoustics, Inc.
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

#include "TestHelpers.h"
#include "vtkLidarReader.h"
#include "vtkVelodynePacketInterpreter.h"

#include <vtkNew.h>
#include <vtkTimerLog.h>

/**
 * @brief TestFile Runs all the tests on a given pcap and its corresponding VTP files
 * @param pcapFileName The pcap file
 * @param referenceFileName The meta-file containing the list of VTP files (baseline) to test against each frames
 * @param correctionFileName The XML sensor calibration file
 * @return 0 on success, 1 on failure
 */
int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    std::cerr << "Wrong number of arguments. Usage: TestVelodyneHDLReader <pcapFileName> <referenceFileName> <correctionFileName>" << std::endl;

    return 1;
  }

    // return value indicate if the test passed
  int retVal = 0;

  // get command line parameter
  std::string pcapFileName = argv[1];
  std::string referenceFileName = argv[2];
  std::string correctionFileName = argv[3];

  std::cout << "-------------------------------------------------------------------------" << std::endl
            << "Pcap :\t" << pcapFileName << std::endl
            << "Baseline:\t" << referenceFileName << std::endl
            << "Corrections :\t" << correctionFileName << std::endl
            << "-------------------------------------------------------------------------" << std::endl;

  // get VTP file name from the reference file
  std::vector<std::string> referenceFilesList;
  referenceFilesList = GenerateFileList(referenceFileName);

  // Generate a Velodyne HDL reader
  vtkNew<vtkLidarReader> HDLReader;
  auto interp = vtkSmartPointer<vtkVelodynePacketInterpreter>::New();
  HDLReader->SetInterpreter(interp);
  HDLReader->SetFileName(pcapFileName);
  HDLReader->SetCalibrationFileName(correctionFileName);
  HDLReader->Update();

  // Check if we can read the PCAP file
  if (HDLReader->GetNumberOfFrames() == 0)
  {
      std::cout << "ERROR, the reader ouput 0 frame!!" << std::endl
                << "PLEASE CHECK YOUR PCAP FILE OR FILEPATH" << std::endl;
      return 1;
  }

  // Integrity tests.
  // Checks in the default VeloView environment that everything can be read correctly.
  std::cout << "Integrity tests..." << std::endl;

  // Checks frame count
  retVal += TestFrameCount(HDLReader->GetNumberOfFrames()-1, referenceFilesList.size());

  // Check properties frame by frame
  unsigned int nbReferences = referenceFilesList.size();

  for (int idFrame = 0; idFrame < nbReferences; ++idFrame)
  {
    std::cout << "---------------------" << std::endl
              << "FRAME " << idFrame << " ..." << std::endl
              << "---------------------" << std::endl;

    vtkPolyData* currentFrame = GetCurrentFrame(HDLReader.Get(), idFrame+1);
    vtkPolyData* currentReference = GetCurrentReference(referenceFilesList, idFrame);

    // Check points count
    retVal += TestPointCount(currentFrame, currentReference);

    // Check points position
    retVal += TestPointPositions(currentFrame, currentReference);

    // Check pointData structure
    retVal += TestPointDataStructure(currentFrame, currentReference);

    // Check pointData values
    retVal += TestPointDataValues(currentFrame, currentReference);

    // Check RPM values
    retVal += TestRPMValues(currentFrame, currentReference);
  }

  // Runtime tests
  // Modifies VeloView's processing options and check that everything run correctly
  std::cout << "Runtime tests..." << std::endl;

  // Check processing options
  retVal  += TestProcessingOptions(HDLReader.Get());


  return  retVal;
}
