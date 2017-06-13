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

#include "vtkVelodyneHDLReader.h"

#include <vtkMathUtilities.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataReader.h>

#include <sstream>

// Helper functions
//-----------------------------------------------------------------------------
bool compare(const double a, const double b)
{
  static const double epsilon = 1.0e-12;

  return vtkMathUtilities::FuzzyCompare(a, b, epsilon);
}

//-----------------------------------------------------------------------------
bool compare(const double *const a, const double *const b, const size_t N)
{
  for (size_t i = 0; i < N; ++i)
  {
    if (!compare(a[i], b[i]))
    {
      return false;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
template <size_t N>
bool compare(double const (&a)[N], double const (&b)[N])
{
  return compare(a, b, N);
}

//-----------------------------------------------------------------------------
std::string toString(const double *const d, const size_t N)
{
  std::ostringstream strs;
  strs << "(";

  strs << d[0];
  for (size_t i = 1; i < N; ++i)
  {
    strs << ", " << d[i];
  }

  strs << ")";

  return strs.str();
}

//-----------------------------------------------------------------------------
template <size_t N>
std::string toString(double const (&d)[N])
{
  return toString(d, N);
}

//-----------------------------------------------------------------------------
vtkPolyData* GetCurrentFrame(vtkVelodyneHDLReader* HDLreader, int index)
{
  HDLreader->Open();
  vtkPolyData* currentFrame = HDLreader->GetFrame(index);
  HDLreader->Close();

  return currentFrame;
}

//-----------------------------------------------------------------------------
std::vector<std::string> GenerateFileList(const std::string &metaFileName)
{
  int lastSeparator = metaFileName.find_last_of("/\\");
  std::string dir = metaFileName.substr(0, lastSeparator);

  std::ifstream metaFile(metaFileName.c_str());
  std::string line;
  std::vector<std::string> filenameList;

  if (metaFile.good())
  {
    while (metaFile >> line)
    {
      if (line.size() > 0)
      {
        std::string fullPath = dir + "/" + line;
        filenameList.push_back(fullPath);
      }
    }
  }

  return filenameList;
}

//-----------------------------------------------------------------------------
vtkPolyData* GetCurrentReference(const std::vector<std::string> &referenceFilesList,
  int index)
{
  vtkNew<vtkXMLPolyDataReader> reader;
  reader->SetFileName(referenceFilesList[index].c_str());
  reader->Update();

  reader->GetOutput()->Register(NULL);

  return reader->GetOutput();
}

// Test functions
//-----------------------------------------------------------------------------
/**
 * @brief TestFrameCount Checks the number of frame on the actual dataset
 * @param frameCount Number of frame in the current dataset
 * @param referenceCount Number of frame in the reference dataset
 * @return 0 on succes, 1 on failure
 */
int TestFrameCount(unsigned int frameCount, unsigned int referenceCount)
{
  if (frameCount != referenceCount)
  {
    std::cerr << "Wrong frame count. Expected " << referenceCount << ", got "
      << frameCount << std::endl;

    return 1;
  }

  return 0;
}

//-----------------------------------------------------------------------------
/**
 * @brief TestPointCount Checks the number of points on the actual dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointCount(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  // Compare the point counts
  vtkIdType currentFrameNbPoints = currentFrame->GetNumberOfPoints();
  vtkIdType currentReferenceNbPoints = currentReference->GetNumberOfPoints();

  if (currentFrameNbPoints != currentReferenceNbPoints)
  {
    std::cerr << "Wrong point count. Expected " << currentReferenceNbPoints
      << ", got " << currentFrameNbPoints << std::endl;

    return 1;
  }

  return 0;
}

//-----------------------------------------------------------------------------
/**
 * @brief TestPointDataStructure Checks the number of pointdata arrays & their
 * structure on the actual dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointDataStructure(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  // Get the current frame point data
  vtkPointData* currentFramePointData = currentFrame->GetPointData();

  // Get the reference point data corresponding to the current frame
  vtkPointData* currentReferencePointData = currentReference->GetPointData();

  // Compare the number of arrays
  int currentFrameNbPointDataArrays = currentFramePointData->GetNumberOfArrays();
  int currentReferenceNbPointDataArrays = currentReferencePointData->GetNumberOfArrays();

  if (currentFrameNbPointDataArrays != currentReferenceNbPointDataArrays)
  {
    std::cerr << "Wrong point data array count. Expected "
      << currentReferenceNbPointDataArrays << ", got "<< currentFrameNbPointDataArrays
      << std::endl;

    return 1;
  }

  // For each array, Checks the structure i.e. the name, number of components &
  // number of tuples matches
  for (int idArray = 0; idArray < currentFrameNbPointDataArrays; ++idArray)
  {
    vtkDataArray* currentFrameArray = currentFramePointData->GetArray(idArray);
    vtkDataArray* currentReferenceArray = currentReferencePointData->GetArray(idArray);

    std::string currentFrameArrayName = currentFrameArray->GetName();
    std::string currentReferenceArrayName = currentReferenceArray->GetName();

    if (currentFrameArrayName != currentReferenceArrayName)
    {
      std::cerr << "Wrong array name for frame. Expected "
        << currentReferenceArrayName << ", got" << currentFrameArrayName
        << std::endl;

      return 1;
    }

    int currentFrameNbComponents = currentFrameArray->GetNumberOfComponents();
    int currentReferenceNbComponents = currentReferenceArray->GetNumberOfComponents();

    if (currentFrameNbComponents != currentReferenceNbComponents)
    {
      std::cerr << "Wrong number of components for array " << currentReferenceArrayName
        << ". Expected " << currentReferenceNbComponents
        << ", got " << currentFrameNbComponents << std::endl;

      return 1;
    }

    vtkIdType currentFrameNbTuples = currentFrameArray->GetNumberOfTuples();
    vtkIdType currentReferenceNbTuples = currentReferenceArray->GetNumberOfTuples();

    if (currentFrameNbTuples != currentReferenceNbTuples)
    {
      std::cerr << "Wrong number of components for array " << currentReferenceArrayName
        << " at frame. Expected " << currentReferenceNbTuples << ", got "
        << currentFrameNbTuples << std::endl;

      return 1;
    }
  }

  return 0;
}

//-----------------------------------------------------------------------------
/**
 * @brief TestPointDataValues Checks each value on each point data array on the
 *  actual dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointDataValues(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  // Get the current frame point data
  vtkPointData* currentFramePointData = currentFrame->GetPointData();

  // Get the reference point data corresponding to the current frame
  vtkPointData* currentReferencePointData = currentReference->GetPointData();

  // For each array, Checks the values
  for (int idArray = 0; idArray < currentReferencePointData->GetNumberOfArrays(); ++idArray)
  {
    vtkDataArray* currentFrameArray = currentFramePointData->GetArray(idArray);
    vtkDataArray* currentReferenceArray = currentReferencePointData->GetArray(idArray);

    for (int idTuple = 0; idTuple < currentReferenceArray->GetNumberOfTuples(); ++idTuple)
    {
      int nbComp = currentReferenceArray->GetNumberOfComponents();

      double* frameTuple = currentFrameArray->GetTuple(idTuple);
      double* referenceTuple = currentReferenceArray->GetTuple(idTuple);

      if (!compare(frameTuple, referenceTuple, nbComp))
      {
        std::cerr << "Tuples " << idTuple << " doesn't match for array "
          << idArray << " ("<< currentReferenceArray->GetName() << "). Expected "
          << toString(referenceTuple, nbComp) << ", got "
          << toString(frameTuple, nbComp) << std::endl;

        return 1;
      }
    }
  }

  return 0;
}

//-----------------------------------------------------------------------------
/**
 * @brief TestPointPositions Checks the position of each point on the actual
 * dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointPositions(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{

  // Get the current frame points
  vtkPoints* currentFramePoints = currentFrame->GetPoints();

  // Get the reference points corresponding to the current frame
  vtkPoints* currentReferencePoints = currentReference->GetPoints();

  // Compare each points
  double framePoint[3];
  double referencePoint[3];

  for (int currentPointId = 0;
    currentPointId < currentFramePoints->GetNumberOfPoints(); ++currentPointId)
  {
    currentFramePoints->GetPoint(currentPointId, framePoint);
    currentReferencePoints->GetPoint(currentPointId, referencePoint);

    if (!compare(framePoint, referencePoint))
    {
      std::cerr << "Wrong point coordinates at point " << currentPointId
        << ". Expected (" << toString(referencePoint) << ", got "
        << toString(framePoint) << std::endl;

      return 1;
    }
  }

  return 0;
}

//-----------------------------------------------------------------------------
/**
 * @brief TestRPMValues Checks each value on each point data array on the actual
 * dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestRPMValues(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  // Get the current frame RPM
  double currentFrameRPM = currentFrame->GetFieldData()
    ->GetArray("RotationPerMinute")->GetTuple1(0);

  // Get the reference point data corresponding to the current frame
  double currentReferenceRPM = currentReference->GetFieldData()
    ->GetArray("RotationPerMinute")->GetTuple1(0);

  if (!compare(currentFrameRPM, currentReferenceRPM))
  {
    std::cerr << "Wrong RPM value. Expected " << currentReferenceRPM
      << ", got " << currentFrameRPM << std::endl;

    return 1;
  }

  return 0;
}

// Multitest functions
//-----------------------------------------------------------------------------
/**
 * @brief TestFile Runs all the tests on a given pcap and its corresponding VTP files
 * @param correctionFileName The corrections to use
 * @param pcapFileName Input PCAP file
 * @param vtpFileName meta-file containing the list of files to test against each frames
 * @return 0 on success, 1 on failure
 */
int TestFile(const std::string &correctionFileName, const std::string &pcapFileName,
  const std::vector<std::string> &referenceFilesList)
{
  int retVal = 0;

  std::cout << "---------------------------------------------------" << std::endl;
  std::cout << "Testing " << pcapFileName << std::endl;
  std::cout << "Corrections file: " << correctionFileName << std::endl;

  // Generate a Velodyne HDL reader
  vtkNew<vtkVelodyneHDLReader> HDLReader;

  HDLReader->SetFileName(pcapFileName);
  HDLReader->SetCorrectionsFile(correctionFileName);

  HDLReader->ReadFrameInformation();
  HDLReader->Update();

  std::cout << "Testing..." << std::endl;

  // Checks frame count
  retVal += TestFrameCount(HDLReader->GetNumberOfFrames(), referenceFilesList.size());

  // Check properties frame by frame
  unsigned int nbReferences = referenceFilesList.size();

  for (int idFrame = 0; idFrame < nbReferences; ++idFrame)
  {
    vtkPolyData* currentFrame = GetCurrentFrame(HDLReader.Get(), idFrame);
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

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);;

  // HDL-32 Single
  correctionsFileName = shareFolder;
  correctionsFileName += "HDL-32.xml";
  pcapFileName = testFolder;
  pcapFileName += "VLP-32c_Single_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "HDL-32_Single_10to20/HDL-32_Single_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  // HDL-32 Dual
  correctionsFileName = shareFolder;
  correctionsFileName += "HDL-32.xml";
  pcapFileName = testFolder;
  pcapFileName += "VLP-32c_Dual_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "HDL-32_Dual_10to20/HDL-32_Dual_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  // HDL-64 Single
  // Note: Live calibration passes an empty string as correction file
  correctionsFileName = "";
  pcapFileName = testFolder;
  pcapFileName += "HDL-64_Single_10to40.pcap";
  referenceFileName = testFolder;
  referenceFileName += "HDL-64_Single_10to40/HDL-64_Single_10to40.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  // HDL-64 Dual
  pcapFileName = testFolder;
  pcapFileName += "HDL-64_Dual_10to20.pcap";
  referenceFileName = testFolder;
  referenceFileName += "HDL-64_Dual_10to20/HDL-64_Dual_10to20.txt";

  referenceFilesList = GenerateFileList(referenceFileName);

  retVal += TestFile(correctionsFileName, pcapFileName, referenceFilesList);

  std::cout << std::endl << "Total tests failed: " << retVal << std::endl;

  return retVal;
}
