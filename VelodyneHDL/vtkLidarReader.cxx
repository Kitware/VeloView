#include "vtkLidarReader.h"

#include "vtkLidarReaderInternal.h"
#include "vtkLidarProviderInternal.h"
#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"

#include <vtkTransform.h>
#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

//-----------------------------------------------------------------------------
vtkLidarReader::vtkLidarReader()
{

}

//-----------------------------------------------------------------------------
vtkLidarReader::vtkLidarReader(vtkLidarReaderInternal* internal)
{
  this->Internal = internal;
  this->SetPimpInternal(internal);
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SetPimpInternal(vtkLidarReaderInternal *internal)
{
  vtkLidarProvider::SetPimpInternal(internal);
  this->Internal = internal;
}

//-----------------------------------------------------------------------------
void vtkLidarReader::PrintSelf( ostream& os, vtkIndent indent )
{
  return this->Superclass::PrintSelf( os, indent );
}

//-----------------------------------------------------------------------------
std::string vtkLidarReader::GetFileName()
{
  return this->Internal->FileName;
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SetFileName(const std::string &filename)
{
  if (filename == this->Internal->FileName)
  {
    return;
  }

  this->Internal->FileName = filename;
  this->Internal->FilePositions.clear();
  this->Internal->UnloadPerFrameData();
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarReader::GetNumberOfFrames()
{
  return this->Internal->FilePositions.size();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkLidarReader::GetFrame(int frameNumber, int wantedNumberOfTrailingFrames)
{
  this->Internal->UnloadPerFrameData();
  if (!this->Internal->Reader)
  {
    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
  }
  if (!this->Internal->IsCalibrated)
  {
    vtkErrorMacro("Corrections have not been set");
    return 0;
  }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[frameNumber]);
  this->Internal->SplitCounter = wantedNumberOfTrailingFrames;

  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
  {
    this->Internal->ProcessPacket(const_cast<unsigned char*>(data), dataLength);

    if (this->Internal->Datasets.size())
    {
      this->Internal->SplitCounter = 0;
      return this->Internal->Datasets.back();
    }
  }

  this->Internal->SplitFrame(true);
  this->Internal->SplitCounter = 0;
  return this->Internal->Datasets.back();
}

//-----------------------------------------------------------------------------
vtkPolyData* vtkLidarReader::GetFramePointer(int frameNumber, int wantedNumberOfTrailingFrames)
{
  return this->GetFrame(frameNumber, wantedNumberOfTrailingFrames).Get();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::Open()
{
  return this->Internal->Open();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::Close()
{
  return this->Internal->Close();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::ProcessPacket(unsigned char *data, unsigned int bytesReceived)
{
  return this->Internal->ProcessPacket(data, bytesReceived);
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SaveFrame(int startFrame, int endFrame, const std::string &filename)
{
  if (!this->Internal->Reader)
  {
    vtkErrorMacro("SaveFrame() called but packet file reader is not open.");
    return;
  }

  vtkPacketFileWriter writer;
  if (!writer.Open(filename))
  {
    vtkErrorMacro("Failed to open packet file for writing: " << filename);
    return;
  }

  pcap_pkthdr* header = 0;
  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  unsigned int dataHeaderLength = 0;
  double timeSinceStart = 0;

  int currentFrame = startFrame;

  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrame]);
  while (this->Internal->Reader->NextPacket(
           data, dataLength, timeSinceStart, &header, &dataHeaderLength) &&
    currentFrame <= endFrame)
  {
    if (this->Internal->IsLidarPacket(data, dataLength, &header, &dataHeaderLength))
    {
      writer.WritePacket(header, const_cast<unsigned char*>(data) - dataHeaderLength);

      currentFrame += this->Internal->CountNewFrameInPacket(data, dataLength, &header, &dataHeaderLength);
    }
  }
  writer.Close();
}

//-----------------------------------------------------------------------------
int vtkLidarReader::RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector);
  vtkInformation* info = outputVector->GetInformationObject(0);

  if (this->Internal->FileName.empty())
  {
    vtkErrorMacro("FileName has not been set.");
    return 0;
  }

  if (!this->Internal->IsCalibrated)
  {
    vtkErrorMacro("Corrections have not been set");
    return 0;
  }

  int timestep = 0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    double timeRequest = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
    timestep = static_cast<int>(floor(timeRequest + 0.5));
  }

  if (timestep < 0 || timestep >= this->GetNumberOfFrames())
  {
    vtkErrorMacro("Cannot meet timestep request: " << timestep << ".  Have "
                                                   << this->GetNumberOfFrames() << " datasets.");
    return 0;
  }

  // check if the reported sensor is consistent with the calibration sensor
  this->Internal->CheckSensorCalibrationConsistency();

  // TODO we should no open the pcap file everytime a frame is requested !!!
  this->Internal->Open();
  output->ShallowCopy(this->GetFrame(
    timestep - this->Internal->NumberOfTrailingFrames, this->Internal->NumberOfTrailingFrames));
  this->Internal->Close();
  return 1;
}

//-----------------------------------------------------------------------------
int vtkLidarReader::RequestInformation(vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* info = outputVector->GetInformationObject(0);
  this->SetTimestepInformation(info);
  return 1;
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SetTimestepInformation(vtkInformation* info)
{
  const size_t numberOfTimesteps = this->Internal->FilePositions.size();
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
