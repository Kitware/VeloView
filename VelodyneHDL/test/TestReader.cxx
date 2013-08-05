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
#include <vtkTimerLog.h>
#include <vtkNew.h>

#include <string>
#include <cstdio>


int main(int argc, char* argv[])
{
  if (argc < 2)
    {
    std::cout << "Usage: " << argv[0] << " <pcap file>" << std::endl;
    return 1;
    }

  std::string filename = argv[1];

  vtkNew<vtkVelodyneHDLReader> reader;

  reader->SetFileName(filename);


  double startTime = vtkTimerLog::GetUniversalTime();
  reader->ReadFrameInformation();

  double elapsed = vtkTimerLog::GetUniversalTime() - startTime;
  printf("elapsed time: %f\n", elapsed);


  printf("number of frames: %d\n", reader->GetNumberOfFrames());

  vtkSmartPointer<vtkPolyData> frame;

  std::vector<int> frames;
  frames.push_back(40);
  frames.push_back(10);
  frames.push_back(100);
  frames.push_back(101);
  frames.push_back(534);
  frames.push_back(213);

  reader->Open();

  for (int i = 0; i < frames.size(); ++i)
    {
    int frameId = frames[i];
    startTime = vtkTimerLog::GetUniversalTime();
    reader->Open();
    frame = reader->GetFrame(frameId);
    reader->Close();
    elapsed = vtkTimerLog::GetUniversalTime() - startTime;
    printf("elapsed time: %f\n", elapsed);
    printf("  frame %d has points: %d\n", frameId, frame->GetNumberOfPoints());
    }

  reader->Close();

  return 0;
}
