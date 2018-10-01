#include "vtkLidarReaderInternal.h"

#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

#include "vtkLidarReader.h"
#include "vtkPacketFileReader.h"
#include "LidarPacketInterpretor.h"

//-----------------------------------------------------------------------------
vtkLidarReaderInternal::vtkLidarReaderInternal(vtkLidarReader* obj)
{
  this->Lidar = obj;
  this->Reader = nullptr;
  this->FileName = "";
}

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::Open()
{
  this->Close();
  this->Reader = new vtkPacketFileReader;
  if (!this->Reader->Open(this->FileName))
  {
    vtkErrorWithObjectMacro(this->Lidar, "Failed to open packet file: " << this->FileName << endl
                                                 << this->Reader->GetLastError());
    this->Close();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::Close()
{
  delete this->Reader;
  this->Reader = 0;
}

//-----------------------------------------------------------------------------
int vtkLidarReaderInternal::ReadFrameInformation()
{
  vtkPacketFileReader reader;
  if (!reader.Open(this->FileName))
  {
    vtkErrorWithObjectMacro(this->Lidar, "Failed to open packet file: " << this->FileName << endl
                                          << reader.GetLastError());
    return 0;
  }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  bool isNewFrame = false;
  int framePositionInPacket = 0;
  double timeSinceStart = 0;

  this->FilePositions.clear();
  this->FilePositionsSkip.clear();
  fpos_t lastFilePosition;
  reader.GetFilePosition(&lastFilePosition);

  while (reader.NextPacket(data, dataLength, timeSinceStart))
  {
    if (!this->Lidar->Interpretor->IsLidarPacket(const_cast<unsigned char*>(data), dataLength))
    {
      reader.GetFilePosition(&lastFilePosition);
      continue;
    }

    this->Lidar->Interpretor->PreProcessPacket(const_cast<unsigned char*>(data), dataLength, isNewFrame, framePositionInPacket);
    if (isNewFrame)
    {
      this->FilePositions.push_back(lastFilePosition);
      this->FilePositionsSkip.push_back(framePositionInPacket);
    }
    reader.GetFilePosition(&lastFilePosition);
  }
  if (!this->Lidar->Interpretor->GetIsCalibrated())
  {
    vtkErrorWithObjectMacro(this->Lidar, "The calibration could not be loaded from the pcap file");
  }
  return this->Lidar->GetNumberOfFrames();
}

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::SetTimestepInformation(vtkInformation *info)
{
  const size_t numberOfTimesteps = this->FilePositions.size();
  std::vector<double> timesteps;
  for (size_t i = 0; i < numberOfTimesteps; ++i)
  {
    timesteps.push_back(i);
  }

  if (numberOfTimesteps)
  {
    double timeRange[2] = { timesteps.front(), timesteps.back() };
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
  }
  else
  {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
  }
}
