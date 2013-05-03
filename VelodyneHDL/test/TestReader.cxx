

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
