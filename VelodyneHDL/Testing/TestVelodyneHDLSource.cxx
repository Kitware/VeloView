
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
#include "vtkLidarStream.h"
#include "vvPacketSender.h"
#include <vtkVelodynePacketInterpreter.h>

#include <vtkNew.h>
#include <vtkTimerLog.h>

#include <boost/thread/thread.hpp>

/**
 * @brief TestFile Runs all the tests on a given pcap and its corresponding VTP files
 * @param correctionFileName The corrections to use
 * @param pcapFileName Input PCAP file
 * @param vtpFileName meta-file containing the list of files to test against each frames
 * @return 0 on success, 1 on failure
 */
int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    std::cerr << "Wrong number of arguments. Usage: TestVelodyneHDLSource <pcapFileName> <referenceFileName> <correctionFileName>" << std::endl;

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

  //
  const std::string destinationIp = "127.0.0.1";
  const int dataPort = 2368;

  // Generate a Velodyne HDL source
  vtkNew<vtkLidarStream> HDLsource;
  auto interp = vtkSmartPointer<vtkVelodynePacketInterpreter>::New();
  HDLsource->SetInterpreter(interp);
  HDLsource->SetCalibrationFileName(correctionFileName);
  HDLsource->SetCacheSize(100);
  HDLsource->SetLIDARPort(dataPort);
  HDLsource->SetIsForwarding(false);
  HDLsource->SetIsCrashAnalysing(true);
  HDLsource->Start();

  std::cout << "Sending data... " << std::endl;
  const double startTime = vtkTimerLog::GetUniversalTime();
  try
  {
    vvPacketSender sender(pcapFileName, destinationIp, dataPort);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    sender.pumpPacket();
    while (!sender.IsDone())
    {
      sender.pumpPacket();
      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
  }
  catch (std::exception& e)
  {
    std::cout << "Caught Exception: " << e.what() << std::endl;
    return 1;
  }
  HDLsource->Stop();

  std::cout << "Done." << std::endl;

  double elapsedTime = vtkTimerLog::GetUniversalTime() - startTime;
  std::cout << "Data sent in " << elapsedTime << "s" << std::endl;

  if (correctionFileName == "" && HDLsource->GetInterpreter()->GetIsCalibrated())
  {
    HDLsource->UnloadFrames();

    std::cout << "Live Correction initialized, resend data..." << std::endl;
    const double resendingDataStartTime = vtkTimerLog::GetUniversalTime();
    try
    {
      HDLsource->Start();
      vvPacketSender sender(pcapFileName, destinationIp, dataPort);

      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
      sender.pumpPacket();

      while (!sender.IsDone())
      {
        sender.pumpPacket();
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
      }

      HDLsource->Stop();
      std::cout << "Done." << std::endl;

      elapsedTime = vtkTimerLog::GetUniversalTime() - resendingDataStartTime;
      std::cout << "Data sent after live calibration in " << elapsedTime << "s" << std::endl;
    }
    catch (std::exception& e)
    {
      std::cout << "Caught Exception: " << e.what() << std::endl;
      return 1;
    }
  }

  std::cout << "Integrity tests..." << std::endl;

  // Integrity tests.
  // Checks in the default VeloView environment that everything can be read correctly.

  retVal += TestFrameCount(GetNumberOfTimesteps(HDLsource.Get()), referenceFilesList.size());

  // Check properties frame by frame
  unsigned int nbReferences = referenceFilesList.size();

  // Skips the first & last frames. First and last frame aren't complete frames
  // In live mode, we don't skip the last firing belonging to the n-1 frame.
  // In live mode, the last frame is uncomplete.
  GetCurrentFrame(HDLsource.Get(), 0);

  for (int idFrame = 0; idFrame < nbReferences - 1; ++idFrame)
  {
    std::cout << "---------------------" << std::endl
              << "FRAME " << idFrame << " ..." << std::endl
              << "---------------------" << std::endl;

    vtkPolyData* currentFrame = GetCurrentFrame(HDLsource.Get(), idFrame + 1);
    vtkPolyData* currentReference = GetCurrentReference(referenceFilesList, idFrame);

    // Points count
    retVal += TestPointCount(currentFrame, currentReference);

    // Points position
    retVal += TestPointPositions(currentFrame, currentReference);

    // PointData structure
    retVal += TestPointDataStructure(currentFrame, currentReference);

    // PointData values
    retVal += TestPointDataValues(currentFrame, currentReference);

    // RPM values
    retVal += TestRPMValues(currentFrame, currentReference);
  }

  return retVal;
}
