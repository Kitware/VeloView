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
#include "vtkLidarStream.h"
#include "vtkVelodynePacketInterpreter.h"

#include <vtkCommand.h>
#include <vtkExecutive.h>
#include <vtkInformation.h>
#include <vtkMathUtilities.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkXMLPolyDataReader.h>

#include <vvPacketSender.h>

#include <sstream>

#include <boost/thread/thread.hpp>

// Error observer workaround because VTK erros and warning are uncatchable
class vtkErrorObserver : public vtkCommand
{
public:
  vtkErrorObserver()
    : Error(false)
    , Warning(false)
    , ErrorMessage("")
    , WarningMessage("")
  {
  }

  static vtkErrorObserver* New() { return new vtkErrorObserver; }

  bool GetError() const { return this->Error; }

  bool GetWarning() const { return this->Warning; }

  void Clear()
  {
    this->Error = false;
    this->Warning = false;
    this->ErrorMessage = "";
    this->WarningMessage = "";
  }

  virtual void Execute(vtkObject* vtkNotUsed(caller), unsigned long event, void* calldata)
  {
    switch (event)
    {
      case vtkCommand::ErrorEvent:
        ErrorMessage = static_cast<char*>(calldata);
        this->Error = true;
        break;

      case vtkCommand::WarningEvent:
        WarningMessage = static_cast<char*>(calldata);
        this->Warning = true;
        break;
    }
  }

  std::string GetErrorMessage() { return ErrorMessage; }
  std::string GetWarningMessage() { return WarningMessage; }
private:
  bool Error;
  bool Warning;
  std::string ErrorMessage;
  std::string WarningMessage;
};

//-----------------------------------------------------------------------------
std::vector<int> parseOptions(vvProcessingOptionsType currentOptions, int numProcessingOptions)
{
  std::vector<int> parsedOptions;

  for (int i = 0; i < numProcessingOptions; ++i)
  {
    parsedOptions.push_back((currentOptions >> i) & 1);
  }

  return parsedOptions;
}

//-----------------------------------------------------------------------------
void SetProcessingOptions(
  vtkLidarReader* HDLReader, vvProcessingOptionsType currentOptions, int numProcessingOptions)
{
  vtkNew<vtkErrorObserver> errorObserver;

  HDLReader->AddObserver(vtkCommand::ErrorEvent, errorObserver.Get());
  HDLReader->AddObserver(vtkCommand::WarningEvent, errorObserver.Get());

  HDLReader->GetExecutive()->AddObserver(vtkCommand::ErrorEvent, errorObserver.Get());
  HDLReader->GetExecutive()->AddObserver(vtkCommand::WarningEvent, errorObserver.Get());

  std::vector<int> parsedOptions = parseOptions(currentOptions, numProcessingOptions);

  vtkVelodynePacketInterpreter* pk =
      dynamic_cast<vtkVelodynePacketInterpreter*>(HDLReader->GetInterpreter());
  if (!pk)
  {
    return;
  }
  pk->SetIgnoreEmptyFrames(parsedOptions[0]);
  pk->SetUseIntraFiringAdjustment(parsedOptions[1]);
  pk->SetIgnoreZeroDistances(parsedOptions[2]);
  pk->SetWantIntensityCorrection(parsedOptions[3]);

  HDLReader->Update();

  if (errorObserver->GetError())
  {
    std::cerr << "Error happend when setting processing options. Options were: " << std::endl
              << " - Ignore empty frames: " << parsedOptions[0] << std::endl
              << " - Intra firing adjust: " << parsedOptions[1] << std::endl
              << " - Ignore zero distances: " << parsedOptions[2] << std::endl
              << " - Intensity corrected: " << parsedOptions[3] << std::endl;

    return;
  }
}

////-----------------------------------------------------------------------------
//void SetProcessingOptions(vtkLidarStream* HDLSource, vvProcessingOptionsType currentOptions,
//  int numProcessingOptions, std::string pcapFileName, std::string destinationIp, int dataPort)
//{
//  vtkNew<vtkErrorObserver> errorObserver;

//  HDLSource->AddObserver(vtkCommand::ErrorEvent, errorObserver.Get());
//  HDLSource->AddObserver(vtkCommand::WarningEvent, errorObserver.Get());

//  HDLSource->GetExecutive()->AddObserver(vtkCommand::ErrorEvent, errorObserver.Get());
//  HDLSource->GetExecutive()->AddObserver(vtkCommand::WarningEvent, errorObserver.Get());

//  std::vector<int> parsedOptions = parseOptions(currentOptions, numProcessingOptions);

//  HDLSource->SetIgnoreEmptyFrames(parsedOptions[0]);
//  HDLSource->SetIntraFiringAdjust(parsedOptions[1]);
//  HDLSource->SetIgnoreZeroDistances(parsedOptions[2]);

////  if (HDLSource->getIsHDL64Data())
////  {
////    HDLSource->SetIntensitiesCorrected(parsedOptions[3]);
////  }

//  HDLSource->UnloadFrames();

//  try
//  {
//    HDLSource->Start();
//    vvPacketSender sender(pcapFileName, destinationIp, dataPort);

//    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
//    sender.pumpPacket();

//    while (!sender.done())
//    {
//      sender.pumpPacket();
//      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
//    }

//    HDLSource->Stop();
//    std::cout << "Done." << std::endl;
//  }
//  catch (std::exception& e)
//  {
//    std::cout << "Caught Exception: " << e.what() << std::endl;

//    return;
//  }

//  if (errorObserver->GetError())
//  {
//    std::cerr << "Error happend when setting processing options. Options were: " << std::endl
//              << " - Ignore empty frames: " << parsedOptions[0] << std::endl
//              << " - Intra firing adjust: " << parsedOptions[1] << std::endl
//              << " - Ignore zero distances: " << parsedOptions[2] << std::endl
//              << " - Intensity corrected: " << parsedOptions[3] << std::endl;

//    return;
//  }
//}

// Processing tests
//-----------------------------------------------------------------------------
int TestProcessingOptions(vtkLidarReader* HDLReader)
{
  // Total number of processing options
  int nbProcessingOptions = 4;

  // Number of case to cover
  int maxOptionStatus = pow(2, nbProcessingOptions);

  for (int currentOptionsCase = 0; currentOptionsCase < maxOptionStatus; ++currentOptionsCase)
  {
    SetProcessingOptions(HDLReader, currentOptionsCase, nbProcessingOptions);

    int nbFrames = HDLReader->GetNumberOfFrames();

    for (int idFrame = 0; idFrame < nbFrames; ++idFrame)
    {
      vtkPolyData* currentFrame = GetCurrentFrame(HDLReader, idFrame);

      if (!currentFrame)
      {
        std::vector<int> parsedOptions = parseOptions(currentOptionsCase, nbProcessingOptions);

        std::cerr << "Cannot run HDLReader with options: " << std::endl
                  << " - Ignore empty frames: " << parsedOptions[0] << std::endl
                  << " - Intra firing adjust: " << parsedOptions[1] << std::endl
                  << " - Ignore zero distances: " << parsedOptions[2] << std::endl
                  << " - Intensity corrected: " << parsedOptions[3] << std::endl;

        return 1;
      }
    }
  }

  return 0;
}

////-----------------------------------------------------------------------------
//int TestProcessingOptions(vtkLidarStream* HDLSource, std::string pcapFileName,
//  std::string destinationIp, int dataPort)
//{
//  // Total number of processing options
//  int nbProcessingOptions = 4;

//  // Number of case to cover
//  int maxOptionStatus = pow(2, nbProcessingOptions);

//  for (int currentOptionsCase = 0; currentOptionsCase < maxOptionStatus; ++currentOptionsCase)
//  {
//    SetProcessingOptions(
//      HDLSource, currentOptionsCase, nbProcessingOptions, pcapFileName, destinationIp, dataPort);

//    int nbFrames = GetNumberOfTimesteps(HDLSource);

//    for (int idFrame = 0; idFrame < nbFrames; ++idFrame)
//    {
//      vtkPolyData* currentFrame = GetCurrentFrame(HDLSource, idFrame);

//      if (!currentFrame)
//      {
//        std::vector<int> parsedOptions = parseOptions(currentOptionsCase, nbProcessingOptions);

//        std::cerr << "Cannot run HDLReader with options: " << std::endl
//                  << " - Ignore empty frames: " << parsedOptions[0] << std::endl
//                  << " - Intra firing adjust: " << parsedOptions[1] << std::endl
//                  << " - Ignore zero distances: " << parsedOptions[2] << std::endl
//                  << " - Intensity corrected: " << parsedOptions[3] << std::endl;

//        return 1;
//      }
//    }
//  }

//  return 0;
//}

// Helper functions
//-----------------------------------------------------------------------------
bool compare(const double* const a, const double* const b, const size_t N, double epsilon)
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
std::string toString(const double* const d, const size_t N)
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
int GetNumberOfTimesteps(vtkLidarStream* HDLSource)
{
  HDLSource->UpdateInformation();

  vtkInformation* outInfo = HDLSource->GetExecutive()->GetOutputInformation(0);

  return outInfo->Length(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
}

//-----------------------------------------------------------------------------
vtkPolyData* GetCurrentFrame(vtkLidarReader* HDLreader, int index)
{
  HDLreader->Open();
  vtkPolyData* currentFrame = HDLreader->GetFrame(index);
  HDLreader->Close();

  return currentFrame;
}

//-----------------------------------------------------------------------------
vtkPolyData* GetCurrentFrame(vtkLidarStream* HDLsource, int index)
{
  HDLsource->UpdateInformation();

  vtkInformation* outInfo = HDLsource->GetExecutive()->GetOutputInformation(0);

  double* timeSteps = outInfo->Get(vtkStreamingDemandDrivenPipeline::TIME_STEPS());

  double updateTime = timeSteps[index];
  outInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(), updateTime);

  HDLsource->Update();

  HDLsource->GetOutput()->Register(NULL);

  vtkPolyData* currentFrame = HDLsource->GetOutput();

  return currentFrame;
}

//-----------------------------------------------------------------------------
std::vector<std::string> GenerateFileList(const std::string& metaFileName)
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
vtkPolyData* GetCurrentReference(const std::vector<std::string>& referenceFilesList, int index)
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
  std::cout << "Frame count : \t";
  if (frameCount != referenceCount)
  {
    std::cerr << "failed : expected " << referenceCount << ", got " << frameCount
              << std::endl;

    return 1;
  }
  std::cout << "passed" << std::endl;
  return 0;
}

//-----------------------------------------------------------------------------
int TestPointCount(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  std::cout << "Point Count : \t";
  // Compare the point counts
  vtkIdType currentFrameNbPoints = currentFrame->GetNumberOfPoints();
  vtkIdType currentReferenceNbPoints = currentReference->GetNumberOfPoints();

  if (currentFrameNbPoints != currentReferenceNbPoints)
  {
    std::cerr << "failed : expected " << currentReferenceNbPoints << ", got "
              << currentFrameNbPoints << std::endl;

    return 1;
  }
  std::cout << "passed" << std::endl;
  return 0;
}

//-----------------------------------------------------------------------------
int TestPointDataStructure(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  std::cout << "Data Structure : \t";
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
    std::cerr << "failed :Wrong point data array count. Expected " << currentReferenceNbPointDataArrays
              << ", got " << currentFrameNbPointDataArrays << std::endl;

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
      std::cerr << "failed : Wrong array name for frame. Expected " << currentReferenceArrayName << ", got"
                << currentFrameArrayName << std::endl;

      return 1;
    }

    int currentFrameNbComponents = currentFrameArray->GetNumberOfComponents();
    int currentReferenceNbComponents = currentReferenceArray->GetNumberOfComponents();

    if (currentFrameNbComponents != currentReferenceNbComponents)
    {
      std::cerr << " failed : Wrong number of components for array " << currentReferenceArrayName
                << ". Expected " << currentReferenceNbComponents << ", got "
                << currentFrameNbComponents << std::endl;

      return 1;
    }

    vtkIdType currentFrameNbTuples = currentFrameArray->GetNumberOfTuples();
    vtkIdType currentReferenceNbTuples = currentReferenceArray->GetNumberOfTuples();

    if (currentFrameNbTuples != currentReferenceNbTuples)
    {
      std::cerr << "failed : Wrong number of components for array " << currentReferenceArrayName
                << " at frame. Expected " << currentReferenceNbTuples << ", got "
                << currentFrameNbTuples << std::endl;

      return 1;
    }
  }
  std::cout << "passed" << std::endl;
  return retVal;
}

//-----------------------------------------------------------------------------
int TestPointDataValues(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  std::cout << "Point Data Value : \t";
  int retVal = 0;

  // Get the current frame point data
  vtkPointData* currentFramePointData = currentFrame->GetPointData();

  // Get the reference point data corresponding to the current frame
  vtkPointData* currentReferencePointData = currentReference->GetPointData();

  // For each array, Checks the values
  for (int idArray = 0; idArray < currentReferencePointData->GetNumberOfArrays(); ++idArray)
  {
    const char * arrayName = currentFramePointData->GetArrayName(idArray);
    vtkDataArray* currentFrameArray = currentFramePointData->GetArray(arrayName);
    vtkDataArray* currentReferenceArray = currentReferencePointData->GetArray(arrayName);

    for (int idTuple = 0; idTuple < currentReferenceArray->GetNumberOfTuples(); ++idTuple)
    {
      int nbComp = currentReferenceArray->GetNumberOfComponents();

      double* frameTuple = currentFrameArray->GetTuple(idTuple);
      double* referenceTuple = currentReferenceArray->GetTuple(idTuple);

      if (!compare(frameTuple, referenceTuple, nbComp, 1e-12))
      {
        if ((long)(*referenceTuple - *frameTuple) % 3600 * 1e6 == 0)
          continue;
        std::cerr << "failed : Tuples " << idTuple << " doesn't match for array " << idArray << " ("
                  << currentReferenceArray->GetName() << "). Expected "
                  << toString(referenceTuple, nbComp) << ", got " << toString(frameTuple, nbComp)
                  << std::endl;

        return 1;
      }
    }
  }
  std::cout << "passed" << std::endl;
  return retVal;
}

//-----------------------------------------------------------------------------
int TestPointPositions(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  std::cout << "Point Position : \t";
  int retVal = 0;

  // Get the current frame points
  vtkPoints* currentFramePoints = currentFrame->GetPoints();

  // Get the reference points corresponding to the current frame
  vtkPoints* currentReferencePoints = currentReference->GetPoints();

  // Compare each points
  double framePoint[3];
  double referencePoint[3];

  for (int currentPointId = 0; currentPointId < currentFramePoints->GetNumberOfPoints();
       ++currentPointId)
  {
    currentFramePoints->GetPoint(currentPointId, framePoint);
    currentReferencePoints->GetPoint(currentPointId, referencePoint);

    if (!compare(framePoint, referencePoint, 1e-12))
    {
      std::cerr << "failed : Wrong point coordinates at point " << currentPointId << ". Expected ("
                << toString(referencePoint) << ", got " << toString(framePoint) << std::endl;

      return 1;
    }
  }
  std::cout << "passed" << std::endl;
  return retVal;
}

//-----------------------------------------------------------------------------
int TestRPMValues(vtkPolyData* currentFrame, vtkPolyData* currentReference)
{
  std::cout << "RPM Value : \t";
  // Get the current frame RPM
  double currentFrameRPM =
    currentFrame->GetFieldData()->GetArray("RotationPerMinute")->GetTuple1(0);

  // Get the reference point data corresponding to the current frame
  double currentReferenceRPM =
    currentReference->GetFieldData()->GetArray("RotationPerMinute")->GetTuple1(0);

  if (!vtkMathUtilities::FuzzyCompare(currentFrameRPM, currentReferenceRPM, 1.0))
  {
    std::cerr << "failed : Wrong RPM value. Expected " << currentReferenceRPM << ", got " << currentFrameRPM
              << std::endl;

    return 1;
  }
  std::cout << "passed" << std::endl;
  return 0;
}
