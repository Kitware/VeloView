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
  bool firstIteration = true;

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

    // add an index for the first Lidar packet
    if (firstIteration)
    {
      // it is possible that the first packet contains 2 frames
      // (end and start of one), and as we rely on the packet header time
      // this 2 frames will have the same timestep. So to avoid that we
      // artificatially move the first timeStep back by one.
      FramePosition newPosition(lastFilePosition, 0, timeSinceStart-1);
      this->FilePositions.push_back(newPosition);
      firstIteration = false;
    }

    // check if the packet content indicate a new frame should be created
    this->Interpreter->PreProcessPacket(data, dataLength, isNewFrame, framePositionInPacket);
    if (isNewFrame)
    {
      FramePosition newPosition(lastFilePosition,framePositionInPacket, timeSinceStart);
      this->FilePositions.push_back(newPosition);
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
    double* firstTimestepPointer = &timesteps.front();
    double* lastTimestepPointer = &timesteps.back();
    // Remove the first and last frame timestep
    // in case you have less that 3 timstep we will still show the partial
    // frame to avoid showing nothing
    if (!this->ShowFirstAndLastFrame && numberOfTimesteps >= 3)
    {
      firstTimestepPointer++;
      lastTimestepPointer--;
      numberOfTimesteps -= 2;

    }
    double timeRange[2] = { *firstTimestepPointer, *lastTimestepPointer };
    // In order to avoid to display
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), firstTimestepPointer, numberOfTimesteps);
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


  // Ensure that frame indexes match between what is effectively shown
  // and what is present inside the PCAP
  size_t numberOfTimesteps = this->FilePositions.size();
  if (!this->ShowFirstAndLastFrame && numberOfTimesteps >= 3)
  {
    startFrame++;
    endFrame++;
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

  bool isNewFrame = false;
  int notUsed = 0;

  // Explanation for why we need to allow currentFrame to go to endFrame + 1:
  // If '[]' represents a packet, '|' represents the separation between frames,
  // Then the contents of a Velodyne Lidar PCAP is in general*:
  // [-- incomplete frame --|-- begin frame 0 --]
  // [-- content of frame 0 --]
  // ... many packets ...
  // [-- end frame 0 --|-- begin frame1 --]
  // ... end of the PCAP
  // Here we see that the first separation between two frame happens in the
  // first packet. We do need to see one more separation than the number of
  // packets to write. This is due to the fact that the first incomplete frame
  // is hidden by VeloView.
  // A possible improvement to VeloView would be to not always hide this first
  // frame, depending on a flag set on the reader.
  // *if you are very lucky the first frame will start at the begining of the
  // first packet, and there will be no "incomplete frame".
  //
  // In my test, writing all frames of the PCAP results in a .pcap file exactly
  // identical to the one that is read, if you enable "ShowFirstAndLastFrame".

  this->Reader->SetFilePosition(&this->FilePositions[startFrame].Position);

  while (this->Reader->NextPacket(
           data, dataLength, timeSinceStart, &header, &dataHeaderLength)
         && currentFrame <= endFrame + 1) // see explanation above for "+ 1"
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
