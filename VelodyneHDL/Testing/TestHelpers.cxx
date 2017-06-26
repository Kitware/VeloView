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

#include "vtkVelodyneHDLReader.h"
#include "vtkVelodyneHDLSource.h"

#include <vtkInformation.h>
#include <vtkMathUtilities.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkXMLPolyDataReader.h>

#include <sstream>

// Helper functions
//-----------------------------------------------------------------------------
bool compare(const double *const a, const double *const b, const size_t N, double epsilon)
{
  for (size_t i = 0; i < N; ++i)
  {
    if (!vtkMathUtilities::FuzzyCompare(a[i], b[i], epsilon))
    {
      return false;
    }
  }
  return true;
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
int GetNumberOfTimesteps(vtkVelodyneHDLSource* HDLSource)
{
  HDLSource->UpdateInformation();

  vtkInformation* outInfo = HDLSource->GetExecutive()->GetOutputInformation(0);

  return outInfo->Length(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
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
vtkPolyData* GetCurrentFrame(vtkVelodyneHDLSource* HDLsource, int index)
{
  HDLsource->UpdateInformation();

  vtkInformation* outInfo = HDLsource->GetExecutive()->GetOutputInformation(0);

  double* timeSteps =
      outInfo->Get(vtkStreamingDemandDrivenPipeline::TIME_STEPS());

  double updateTime = timeSteps[index];
  outInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(),
    updateTime);

  HDLsource->Update();

  HDLsource->GetOutput()->Register(NULL);

  vtkPolyData* currentFrame = HDLsource->GetOutput();

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
int TestPointDataStructure(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  int retVal = 0;

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

      retVal = 1;
    }

    int currentFrameNbComponents = currentFrameArray->GetNumberOfComponents();
    int currentReferenceNbComponents = currentReferenceArray->GetNumberOfComponents();

    if (currentFrameNbComponents != currentReferenceNbComponents)
    {
      std::cerr << "Wrong number of components for array " << currentReferenceArrayName
        << ". Expected " << currentReferenceNbComponents
        << ", got " << currentFrameNbComponents << std::endl;

      retVal = 1;
    }

    vtkIdType currentFrameNbTuples = currentFrameArray->GetNumberOfTuples();
    vtkIdType currentReferenceNbTuples = currentReferenceArray->GetNumberOfTuples();

    if (currentFrameNbTuples != currentReferenceNbTuples)
    {
      std::cerr << "Wrong number of components for array " << currentReferenceArrayName
        << " at frame. Expected " << currentReferenceNbTuples << ", got "
        << currentFrameNbTuples << std::endl;

      retVal = 1;
    }
  }

  return retVal;
}

//-----------------------------------------------------------------------------
int TestPointDataValues(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  int retVal = 0;

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

      if (!compare(frameTuple, referenceTuple, nbComp, 1e-12))
      {
        if ((long)(*referenceTuple - *frameTuple) % 3600*1e6 ==0) continue;
        std::cerr << "Tuples " << idTuple << " doesn't match for array "
          << idArray << " ("<< currentReferenceArray->GetName() << "). Expected "
          << toString(referenceTuple, nbComp) << ", got "
          << toString(frameTuple, nbComp) << std::endl;

          retVal = 1;
      }
    }
  }

  return retVal;
}

//-----------------------------------------------------------------------------
int TestPointPositions(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  int retVal = 0;

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

    if (!compare(framePoint, referencePoint, 1e-12))
    {
      std::cerr << "Wrong point coordinates at point " << currentPointId
        << ". Expected (" << toString(referencePoint) << ", got "
        << toString(framePoint) << std::endl;

      retVal = 1;
    }
  }

  return retVal;
}

//-----------------------------------------------------------------------------
int TestRPMValues(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  // Get the current frame RPM
  double currentFrameRPM = currentFrame->GetFieldData()
    ->GetArray("RotationPerMinute")->GetTuple1(0);

  // Get the reference point data corresponding to the current frame
  double currentReferenceRPM = currentReference->GetFieldData()
    ->GetArray("RotationPerMinute")->GetTuple1(0);

  if (!vtkMathUtilities::FuzzyCompare(currentFrameRPM, currentReferenceRPM, 1.0))
  {
    std::cerr << "Wrong RPM value. Expected " << currentReferenceRPM
      << ", got " << currentFrameRPM << std::endl;

    return 1;
  }

  return 0;
}