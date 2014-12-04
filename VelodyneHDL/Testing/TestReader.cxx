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


#include <vtkPacketFileReader.h>
#include <vtkVelodyneHDLSource.h>
#include <vtkVelodyneHDLReader.h>

#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkTimerLog.h>
#include <vtkNew.h>

#include <map>
#include <string>
#include <cmath>
#include <cstdio>

typedef bool (*TestActionFunction)(vtkSmartPointer<vtkPolyData>&,
                                   std::istream&);

typedef std::map<std::string, TestActionFunction> TestActionFunctionMap;

vtkVelodyneHDLReader* reader;

//-----------------------------------------------------------------------------
bool compare(double a, double b)
{
  return std::fabs(a - b) <= 0.0001;
}

//-----------------------------------------------------------------------------
template <size_t N>
bool compare(double (&a)[N], double (&b)[N])
{
  for (size_t i = 0; i < N; ++i)
    {
    if (!compare(a[i], b[i])) return false;
    }
  return true;
}

//-----------------------------------------------------------------------------
bool selectFrame(vtkSmartPointer<vtkPolyData>& frame, std::istream& is)
{
  int frameId;
  is >> frameId;
  if (!is)
    {
    std::cerr << "frame: missing required argument" << std::endl;
    return false;
    }

  std::cout << "read frame " << frameId << "... ";
  const double startTime = vtkTimerLog::GetUniversalTime();

  reader->Open(); // must reset state, or reader crashes 'going backwards'
  frame = reader->GetFrame(frameId);
  reader->Close();

  if (!frame)
    {
    std::cout << "FAILED" << std::endl;
    return false;
    }

  const double elapsed = vtkTimerLog::GetUniversalTime() - startTime;
  std::cout << elapsed << "s" << std::endl;

  return true;
}

//-----------------------------------------------------------------------------
bool testFrameCount(vtkSmartPointer<vtkPolyData>& frame, std::istream& is)
{
  int expectedCount;
  is >> expectedCount;
  if (!is)
    {
    std::cerr << "frame_count: missing required argument" << std::endl;
    return false;
    }

  const int actualCount = reader->GetNumberOfFrames();
  if (actualCount != expectedCount)
    {
    std::cerr << "frame_count: expected " << expectedCount
              << ", got " << actualCount << std::endl;
    return false;
    }

  return true;
}


//-----------------------------------------------------------------------------
bool testPointCount(vtkSmartPointer<vtkPolyData>& frame, std::istream& is)
{
  vtkIdType expectedCount;
  is >> expectedCount;
  if (!is)
    {
    std::cerr << "point_count: missing required argument" << std::endl;
    return false;
    }

  if (!frame)
    {
    std::cerr << "point_count: no valid frame" << std::endl;
    return false;
    }

  const vtkIdType actualCount = frame->GetNumberOfPoints();
  if (actualCount != expectedCount)
    {
    std::cerr << "point_count: expected " << expectedCount
              << ", got " << actualCount << std::endl;
    return false;
    }

  return true;
}

//-----------------------------------------------------------------------------
bool testCropSettings(vtkSmartPointer<vtkPolyData>& frame, std::istream& is)
{
  bool cropEnabled, cropInside;
  double ptcropregion[6];
  is >> cropEnabled >> cropInside;
  for(int i = 0; i < 6; ++i)
    {
    is >> ptcropregion[i];
    }

  reader->SetCropReturns(cropEnabled);
  reader->SetCropInside(cropInside);
  reader->SetCropRegion(ptcropregion);

  return true;
}

//-----------------------------------------------------------------------------
bool testPointValues(vtkSmartPointer<vtkPolyData>& frame, std::istream& is)
{
  vtkIdType pointId;
  double expectedPosition[3];
  int expectedIntensity;
  int expectedRelativeDistance, expectedRelativeIntensity;
  bool isDual;
  is >> pointId
     >> expectedPosition[0] >> expectedPosition[1] >> expectedPosition[2]
     >> expectedIntensity
     >> isDual;
  if(isDual) {
    is >> expectedRelativeDistance >> expectedRelativeIntensity;
  }
  if (!is)
    {
    std::cerr << "point_values: missing required argument" << std::endl;
    return false;
    }

  vtkPoints* const points = frame->GetPoints();
  vtkDataArray* const intensityData =
    frame->GetPointData()->GetArray("intensity");
  vtkDataArray* const dualDistanceFlagData =
    frame->GetPointData()->GetArray("dual_distance");
  vtkDataArray* const dualIntensityFlagData =
    frame->GetPointData()->GetArray("dual_intensity");

  if (pointId < 0 || pointId > points->GetNumberOfPoints())
    {
    std::cerr << "point_values: point index out of range" << std::endl;
    return false;
    }
  if (!intensityData || intensityData->GetDataSize() < pointId)
    {
    std::cerr << "point_values: frame intensity data missing or truncated"
              << std::endl;
    }
  if (isDual && (!dualDistanceFlagData || dualDistanceFlagData->GetDataSize() < pointId))
    {
    std::cerr << "point_values: frame dual-return relative distance data"
                 " missing or truncated" << std::endl;
    }
  if (isDual && (!dualIntensityFlagData || dualIntensityFlagData->GetDataSize() < pointId))
    {
    std::cerr << "point_values: frame dual-return relative intensity data"
                 " missing or truncated" << std::endl;
    }

  double actualPosition[3];
  points->GetPoint(pointId, actualPosition);

  const double actualIntensity =
    intensityData->GetComponent(pointId, 0);
  double actualRelativeDistance;
  if(isDual)
    {
    actualRelativeDistance = dualDistanceFlagData->GetComponent(pointId, 0);
    }
  double actualRelativeInteensity;
  if(isDual)
    {
    actualRelativeInteensity = dualIntensityFlagData->GetComponent(pointId, 0);
    }

  if (!compare(expectedPosition, actualPosition) ||
      !compare(expectedIntensity, actualIntensity) ||
      (isDual && !compare(expectedRelativeDistance, actualRelativeDistance)) ||
      (isDual && !compare(expectedRelativeIntensity, actualRelativeInteensity)))
    {
    std::cerr << "pount_values: expected "
              << expectedPosition[0] << ' '
              << expectedPosition[1] << ' '
              << expectedPosition[2] << ' '
              << expectedIntensity << ' '
              << isDual << ' ';
    if(isDual)
      {
      std::cerr << expectedRelativeDistance << ' '
                << expectedRelativeIntensity;
      }
    std::cerr << ", got "
              << actualPosition[0] << ' '
              << actualPosition[1] << ' '
              << actualPosition[2] << ' '
              << actualIntensity << ' '
              << isDual << ' ';
    if(isDual)
      {
      std::cerr << actualRelativeDistance << ' '
                << actualRelativeInteensity;
      }
    std::cerr << std::endl;
    return false;
    }

  return true;
}

//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  TestActionFunctionMap testActions;
  testActions["frame"] = &selectFrame;
  testActions["frame_count"] = &testFrameCount;
  testActions["point_count"] = &testPointCount;
  testActions["point_values"] = &testPointValues;
  testActions["crop_settings"] = &testCropSettings;

  const TestActionFunctionMap::const_iterator invalidCommand =
    testActions.end();

  if (argc < 3)
    {
    std::cout << "Usage: " << argv[0] << " <pcap file> <calibration> [<expect file>]"
              << std::endl;
    return 1;
    }

  const std::string filename = argv[1];
  const std::string calibrationfilename = argv[2];
  const std::string testScriptFilename = (argc > 3 ? argv[3] : "");

  vtkNew<vtkVelodyneHDLReader> _reader;
  reader = _reader.GetPointer();

  reader->SetFileName(filename);
  reader->SetCorrectionsFile(calibrationfilename);

  double startTime = vtkTimerLog::GetUniversalTime();
  reader->ReadFrameInformation();

  double elapsed = vtkTimerLog::GetUniversalTime() - startTime;
  printf("elapsed time: %f\n", elapsed);

  printf("number of frames: %d\n", reader->GetNumberOfFrames());

  vtkSmartPointer<vtkPolyData> frame;

  if (testScriptFilename.empty())
    {
    std::vector<int> frames;
    frames.push_back(40);
    frames.push_back(10);
    frames.push_back(100);
    frames.push_back(101);
    frames.push_back(534);
    frames.push_back(213);

    for (int i = 0; i < frames.size(); ++i)
      {
      int frameId = frames[i];
      startTime = vtkTimerLog::GetUniversalTime();
      reader->Open(); // must reset state, or reader crashes 'going backwards'
      frame = reader->GetFrame(frameId);
      reader->Close();
      elapsed = vtkTimerLog::GetUniversalTime() - startTime;
      std::cout << "elapsed time: " << elapsed << std::endl;
      std::cout << "  frame " << frameId << " has points: " << (frame ? frame->GetNumberOfPoints() : 0) << std::endl;
      }
    }
  else
    {
    std::ifstream testScript(testScriptFilename.c_str());
    for (;;)
      {
      // read next command
      std::string command;
      testScript >> command;
      if (testScript.eof()) break;

      // look up command
      const TestActionFunctionMap::const_iterator iter =
        testActions.find(command);
      if (iter == invalidCommand)
        {
        std::cerr << "unknown command: '" << command << "'" << std::endl;
        return 1;
        }
      else
        {
        // execute command
        if (!(*iter->second)(frame, testScript))
          {
          // command failed
          return 1;
          }
        }
      }
    }

  return 0;
}
