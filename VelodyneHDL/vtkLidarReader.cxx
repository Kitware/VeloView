#include "vtkLidarReader.h"

#include "vtkLidarReaderInternal.h"
#include "LidarPacketInterpretor.h"
#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"

#include <vtkTransform.h>
#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>


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
  this->Internal->FilePositionsSkip.clear();
  this->Interpretor->ResetDataForNewFrame();
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
  this->Interpretor->ResetDataForNewFrame();
  if (!this->Internal->Reader)
  {
    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
  }
  if (!this->Interpretor->GetIsCalibrated())
  {
    vtkErrorMacro("Corrections have not been set");
    return 0;
  }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart;
  int startFrameToProcess = std::max(frameNumber - wantedNumberOfTrailingFrames, 0);
  int firstFramePositionInPacket = this->Internal->FilePositionsSkip[startFrameToProcess];

  // indicate how many frame should be process as one frame
  this->Interpretor->SetSplitCounter(frameNumber - startFrameToProcess);
  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrameToProcess]);
  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
  {

    if (!this->Interpretor->IsLidarPacket(const_cast<unsigned char*>(data), dataLength))
    {
      continue;
    }

    this->Interpretor->ProcessPacket(const_cast<unsigned char*>(data), dataLength, firstFramePositionInPacket);

    // check if the required frames are ready
    if (this->Interpretor->IsNewFrameReady())
    {
      this->Interpretor->SetSplitCounter(0);
      return this->Interpretor->GetLastFrameAvailable();
    }
    firstFramePositionInPacket = 0;
  }

  this->Interpretor->SplitFrame(true);
  this->Interpretor->SetSplitCounter(0);
  return this->Interpretor->GetLastFrameAvailable();
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

  // because fpos_t is plateform specific and should not be used for comparaison
  // it's not possible to simply interate from FiePositions[start] to FilePositions[end]
  // we need to detect new frame in the pcap directly once again
  pcap_pkthdr* header = 0;
  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  unsigned int dataHeaderLength = 0;
  double timeSinceStart = 0;
  int currentFrame = startFrame;
  bool isNewFrame;
  int notUsed;

  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrame]);
  while (this->Internal->Reader->NextPacket(
           data, dataLength, timeSinceStart, &header, &dataHeaderLength) &&
    currentFrame <= endFrame)
  {
    if (this->Interpretor->IsLidarPacket(const_cast<unsigned char*>(data), dataLength))
    {
      writer.WritePacket(header, const_cast<unsigned char*>(data) - dataHeaderLength);

      this->Interpretor->PreProcessPacket(const_cast<unsigned char*>(data),dataLength, isNewFrame, notUsed);
      currentFrame += static_cast<int> (isNewFrame);
      this->UpdateProgress(0.0);
    }
  }
    writer.Close();
}

//-----------------------------------------------------------------------------
vtkLidarReader::vtkLidarReader()
{

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

  if (!this->Interpretor->GetIsCalibrated())
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

  //! @todo we should no open the pcap file everytime a frame is requested !!!
  this->Internal->Open();
  output->ShallowCopy(this->GetFrame(timestep, this->Interpretor->GetNumberOfTrailingFrames()));
  this->Internal->Close();
  return 1;
}

//-----------------------------------------------------------------------------
int vtkLidarReader::RequestInformation(vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  if (!this->Internal->FileName.empty() && this->Internal->FilePositions.empty())
  {
    this->Internal->ReadFrameInformation();
  }
  vtkInformation* info = outputVector->GetInformationObject(0);
  this->Internal->SetTimestepInformation(info);
  return 1;
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SetPimpInternal(vtkLidarReaderInternal *internal, LidarPacketInterpretor *interpretor)
{
  vtkLidarProvider::SetPimpInternal(interpretor);
  this->Internal = internal;
}
