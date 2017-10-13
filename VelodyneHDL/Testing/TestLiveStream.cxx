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
#include "vtkVelodyneHDLSource.h"
#include "vvPacketSender.h"

#include <vtkNew.h>

#include <boost/thread/thread.hpp>

// Multitest functions
//-----------------------------------------------------------------------------
/**
 * @brief TestFile Runs all the tests on a given pcap and its corresponding VTP files
 * @param correctionFileName The corrections to use
 * @param pcapFileName Input PCAP file
 * @param vtpFileName meta-file containing the list of files to test against each frames
 * @return 0 on success, 1 on failure
 */
int TestFile(const std::string& correctionFileName, const std::string& pcapFileName,
  const std::vector<std::string>& referenceFilesList)
{
  int retVal = 0;

  std::cout << "---------------------------------------------------" << std::endl;
  std::cout << "Testing " << pcapFileName << std::endl;
  std::cout << "Corrections file: " << correctionFileName << std::endl;

  // Generate a Velodyne HDL source
  static const std::string destinationIp = "127.0.0.1";
  static const int dataPort = 2368;

  vtkNew<vtkVelodyneHDLSource> HDLsource;
  HDLsource->SetCorrectionsFile(correctionFileName);
  HDLsource->SetCacheSize(100);
  HDLsource->SetLIDARPort(dataPort);
  HDLsource->SetGPSPort(8308);
  HDLsource->SetisForwarding(false);
  HDLsource->SetisCrashAnalysing(true);
  HDLsource->Start();

  std::cout << "Sending data... " << std::endl;
  try
  {
    vvPacketSender sender(pcapFileName, destinationIp, dataPort);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    sender.pumpPacket();
    while (!sender.done())
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

  if (correctionFileName == "" && HDLsource->GetCorrectionsInitialized())
  {
    HDLsource->UnloadDatasets();

    std::cout << "Live Correction initialized, resend data..." << std::endl;
    try
    {
      HDLsource->Start();
      vvPacketSender sender(pcapFileName, destinationIp, dataPort);

      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
      sender.pumpPacket();

      while (!sender.done())
      {
        sender.pumpPacket();
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
      }

      HDLsource->Stop();
      std::cout << "Done." << std::endl;
    }
    catch (std::exception& e)
    {
      std::cout << "Caught Exception: " << e.what() << std::endl;
      return 1;
    }
  }

  std::cout << "Testing..." << std::endl;

  retVal += TestFrameCount(GetNumberOfTimesteps(HDLsource.Get()), referenceFilesList.size());

  // Check properties frame by frame
  unsigned int nbReferences = referenceFilesList.size();

  // Skips the first & last frames. First and last frame aren't complete frames
  // In live mode, we don't skip the last firing belonging to the n-1 frame.
  // In live mode, the last frame is uncomplete.
  GetCurrentFrame(HDLsource.Get(), 0);

  for (int idFrame = 0; idFrame < nbReferences - 1; ++idFrame)
  {
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

  if (retVal == 0)
  {
    std::cout << "Every tests passed" << std::endl;
  }
  else
  {
    std::cout << retVal << " tests failed for this dataset" << std::endl;
  }

  return retVal;
}

//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cerr << "Wrong number of arguments. Usage: TestLiveStream TEST_DIR SHARE_DIR" << std::endl;

    return 1;
  }

  int retVal = 0;

  std::string testFolder = argv[1];
  std::string shareFolder = argv[2];

  std::vector<std::string> referenceFilesList;

  std::string correctionsFileName;
  std::string pcapFileName;
  std::string referenceFileName;

  // VLP-16 Single
  correctionsFileName = shareFolder;
  correctionsFileName += "VLP-16.xml";
  pcapFileName = testFolder;
  pcapFileName += "VLP-16_Single_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "VLP-16_Single_10to20/VLP-16_Single_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  // VLP-16 Dual
  pcapFileName = testFolder;
  pcapFileName += "VLP-16_Dual_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "VLP-16_Dual_10to20/VLP-16_Dual_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  // VLP-32c Single
  correctionsFileName = shareFolder;
  correctionsFileName += "VLP-32c.xml";
  pcapFileName = testFolder;
  pcapFileName += "VLP-32c_Single_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "VLP-32c_Single_10to20/VLP-32c_Single_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  // VLP-32c Dual
  pcapFileName = testFolder;
  pcapFileName += "VLP-32c_Dual_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "VLP-32c_Dual_10to20/VLP-32c_Dual_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);
  ;

  // HDL-64 Single
  // Note: Live calibration passes an empty string as correction file
  correctionsFileName = "";
  pcapFileName = testFolder;
  pcapFileName += "HDL-64_Single_10to70.pcap";
  referenceFileName = testFolder;
  referenceFileName += "HDL-64_Single_10to70/HDL-64_Single_10to70.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  // HDL-64 Dual
  correctionsFileName = "";
  pcapFileName = testFolder;
  pcapFileName += "HDL-64_Dual_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "HDL-64_Dual_10to20/HDL-64_Dual_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  std::cout << std::endl << "Total tests failed: " << retVal << std::endl;

  return retVal;
}
