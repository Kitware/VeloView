#include "vtkLidarReader.h"

#include "vtkLidarPacketInterpreter.h"
#include "vtkPacketFileWriter.h"
#include "vtkPacketFileReader.h"

#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

//-----------------------------------------------------------------------------
int vtkLidarReader::ReadFrameInformation()
{
  vtkPacketFileReader reader;
  if (!reader.Open(this->FileName))
  {
    vtkErrorMacro(<< "Failed to open packet file: " << this->FileName << endl
                                          << reader.GetLastError());
    return 0;
  }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  bool isNewFrame = false;
  int framePositionInPacket = 0;
  double timeSinceStart = 0;

  this->FilePositions.clear();
  fpos_t lastFilePosition;
  reader.GetFilePosition(&lastFilePosition);

  while (reader.NextPacket(data, dataLength, timeSinceStart))
  {
    // This command sends a signal that can be observed from outside
    // and that is used to diplay a Qt progress dialog from Python
    // This progress dialog is not displaying a progress percentage,
    // thus it is ok to pass 0.0
    this->UpdateProgress(0.0);


    if (!this->Interpreter->IsLidarPacket(data, dataLength))
    {
      reader.GetFilePosition(&lastFilePosition);
      continue;
    }

    {
      FramePosition newPosition(lastFilePosition,framePositionInPacket, timeSinceStart);
      this->FilePositions.push_back(newPosition);
      // check if the packet content indicate a new frame should be created
      this->Interpreter->PreProcessPacket(data, dataLength, isNewFrame, framePositionInPacket);
      if (isNewFrame)
      {
        FramePosition newPosition(lastFilePosition,framePositionInPacket, timeSinceStart);
        this->FilePositions.push_back(newPosition);
      }
    }

    reader.GetFilePosition(&lastFilePosition);
  }

  if (!this->Interpreter->GetIsCalibrated())
  {
    vtkErrorMacro( << "The calibration could not be loaded from the pcap file");
  }
  return this->GetNumberOfFrames();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SetTimestepInformation(vtkInformation *info)
{
  size_t numberOfTimesteps = this->FilePositions.size();
  std::vector<double> timesteps(numberOfTimesteps);
  double timeOffset = this->GetInterpreter()->GetTimeOffset();
  for (size_t i = 0; i < numberOfTimesteps; ++i)
  {
    timesteps[i] = this->FilePositions[i].Time + timeOffset;
  }

  if (this->FilePositions.size())
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

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLidarReader)

//-----------------------------------------------------------------------------
void vtkLidarReader::SetFileName(const std::string &filename)
{
  if (filename == this->FileName)
  {
    return;
  }

  this->FileName = filename;
  this->FilePositions.clear();
  this->Modified();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkLidarReader::GetFrame(int frameNumber)
{
  this->Interpreter->ResetCurrentFrame();
  if (!this->Reader)
  {
    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
  }
  if (!this->Interpreter->GetIsCalibrated())
  {
    vtkErrorMacro("Corrections have not been set");
    return 0;
  }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart;
  int firstFramePositionInPacket = this->FilePositions[frameNumber].Skip;

  this->Reader->SetFilePosition(&this->FilePositions[frameNumber].Position);
  while (this->Reader->NextPacket(data, dataLength, timeSinceStart))
  {

    if (!this->Interpreter->IsLidarPacket(data, dataLength))
    {
      continue;
    }

    this->Interpreter->ProcessPacket(data, dataLength, firstFramePositionInPacket);

    // check if the required frames are ready
    if (this->Interpreter->IsNewFrameReady())
    {
      return this->Interpreter->GetLastFrameAvailable();
    }
    firstFramePositionInPacket = 0;
  }

  this->Interpreter->SplitFrame(true);
  return this->Interpreter->GetLastFrameAvailable();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::Open()
{
  this->Close();
  this->Reader = new vtkPacketFileReader;
  if (!this->Reader->Open(this->FileName))
  {
    vtkErrorMacro(<< "Failed to open packet file: " << this->FileName << endl
                                                 << this->Reader->GetLastError())
    this->Close();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarReader::Close()
{
  delete this->Reader;
  this->Reader = 0;
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SaveFrame(int startFrame, int endFrame, const std::string &filename)
{
  if (!this->Reader)
  {
    vtkErrorMacro("SaveFrame() called but packet file reader is not open.")
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

  this->Reader->SetFilePosition(&this->FilePositions[startFrame].Position);
  while (this->Reader->NextPacket(data, dataLength, timeSinceStart, &header, &dataHeaderLength)
         && currentFrame <= endFrame)
  {
    // writing all packets, even those that do not contain lidar frames,
    // such as the 512 bytes packets of Velodyne IMU data + forwarded GPS data
    writer.WritePacket(header, const_cast<unsigned char*>(data) - dataHeaderLength);
    if (this->Interpreter->IsLidarPacket(data, dataLength))
    {
      // we need to count frames and some are split in multiple packets
      this->Interpreter->PreProcessPacket(data, dataLength, isNewFrame, notUsed);
      currentFrame += static_cast<int>(isNewFrame);
      this->UpdateProgress(0.0);
    }
  }
    writer.Close();
}

//-----------------------------------------------------------------------------
int vtkLidarReader::RequestData(vtkInformation *vtkNotUsed(request),
                                vtkInformationVector **vtkNotUsed(inputVector),
                                vtkInformationVector *outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector);
  vtkTable* calibration = vtkTable::GetData(outputVector,1);

  vtkInformation* info = outputVector->GetInformationObject(0);

  if (this->FileName.empty())
  {
    vtkErrorMacro("FileName has not been set.");
    return 0;
  }

  if (!this->Interpreter->GetIsCalibrated())
  {
    vtkErrorMacro("Corrections have not been set");
    return 0;
  }

  double timestep = 0.0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    timestep = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
  }

  // iterating over all timesteps until finding the first one with a greater time value
  auto idx = std::lower_bound(this->FilePositions.begin(),
                              this->FilePositions.end(),
                              timestep,
                              [](FramePosition& fp, double d)
                                { return fp.Time < d; });

  auto frameRequested = std::distance(this->FilePositions.begin(), idx);

  if (idx == this->FilePositions.end())
  {
    vtkErrorMacro("Cannot meet timestep request: " << frameRequested << ".  Have "
                                                   << this->GetNumberOfFrames() << " datasets.");
    return 0;
  }

  //! @todo we should no open the pcap file everytime a frame is requested !!!
  this->Open();
  output->ShallowCopy(this->GetFrame(frameRequested));
  this->Close();

  vtkTable *t = this->Interpreter->GetCalibrationTable();
  calibration->ShallowCopy(t);

  return 1;
}

//-----------------------------------------------------------------------------
int vtkLidarReader::RequestInformation(vtkInformation* request,
                                       vtkInformationVector** inputVector,
                                       vtkInformationVector* outputVector)
{
  this->Superclass::RequestInformation(request, inputVector, outputVector);
  if (!this->FileName.empty() && this->FilePositions.empty())
  {
    this->ReadFrameInformation();
  }
  vtkInformation* info = outputVector->GetInformationObject(0);
  this->SetTimestepInformation(info);
  return 1;
}
