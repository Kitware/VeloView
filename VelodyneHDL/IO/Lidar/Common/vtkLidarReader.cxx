#include "vtkLidarReader.h"

#include "LidarPacketInterpreter.h"
#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"

#include <vtkTransform.h>
#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

//namespace {
struct  vtkLidarReaderInternal
{
  //! Backwward pointer to public class interface
  vtkLidarReader* Lidar;

  vtkLidarReaderInternal(vtkLidarReader* obj);

  /**
   * @brief Open open the pcap file
   */
  void Open();

  /**
   * @brief Close close the pcap file

   */
  void Close();

  /**
   * @brief ReadFrameInformation read the whole pcap and create a frame index. In case the
   * calibration is also contain in the pcap file, this will also read it
   */
  int ReadFrameInformation();

  /**
   * @brief SetTimestepInformation indicate to vtk which time step are available
   * @param info
   */
  void SetTimestepInformation(vtkInformation* info);

  //! Name of the pcap file to read
  std::string FileName;

  //! pcap packet index for every frame which enable to jump quicky from one frame to another
  std::vector<fpos_t> FilePositions;

  //! Frame do not new to start at the begin of a packet, this indicate
  std::vector<int> FilePositionsSkip;

  //! libpcap wrapped reader which enable to get the raw pcap packet from the pcap file
  vtkPacketFileReader* Reader;
};

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
    if (!this->Lidar->Interpreter->IsLidarPacket(const_cast<unsigned char*>(data), dataLength))
    {
      reader.GetFilePosition(&lastFilePosition);
      continue;
    }

    this->Lidar->Interpreter->PreProcessPacket(const_cast<unsigned char*>(data), dataLength, isNewFrame, framePositionInPacket);
    if (isNewFrame)
    {
      this->FilePositions.push_back(lastFilePosition);
      this->FilePositionsSkip.push_back(framePositionInPacket);
    }
    reader.GetFilePosition(&lastFilePosition);
  }
  if (!this->Lidar->Interpreter->GetIsCalibrated())
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
//}
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
  this->Interpreter->ResetCurrentFrame();
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
  this->Interpreter->ResetCurrentFrame();
  if (!this->Internal->Reader)
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
  int startFrameToProcess = std::max(frameNumber - wantedNumberOfTrailingFrames, 0);
  int firstFramePositionInPacket = this->Internal->FilePositionsSkip[startFrameToProcess];

  // indicate how many frame should be process as one frame
  this->Interpreter->SetSplitCounter(frameNumber - startFrameToProcess);
  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrameToProcess]);
  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
  {

    if (!this->Interpreter->IsLidarPacket(const_cast<unsigned char*>(data), dataLength))
    {
      continue;
    }

    this->Interpreter->ProcessPacket(const_cast<unsigned char*>(data), dataLength, firstFramePositionInPacket);

    // check if the required frames are ready
    if (this->Interpreter->IsNewFrameReady())
    {
      this->Interpreter->SetSplitCounter(0);
      return this->Interpreter->GetLastFrameAvailable();
    }
    firstFramePositionInPacket = 0;
  }

  this->Interpreter->SplitFrame(true);
  this->Interpreter->SetSplitCounter(0);
  return this->Interpreter->GetLastFrameAvailable();
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
    // writing all packets, even those that do not contain lidar frames,
    // such as the 512 bytes packets of Velodyne IMU data + forwarded GPS data
    writer.WritePacket(header, const_cast<unsigned char*>(data) - dataHeaderLength);
    if (this->Interpreter->IsLidarPacket(const_cast<unsigned char*>(data), dataLength))
    {
      // we need to count frames and some are split in multiple packets
      this->Interpreter->PreProcessPacket(const_cast<unsigned char*>(data), dataLength, isNewFrame, notUsed);
      currentFrame += static_cast<int>(isNewFrame);
      this->UpdateProgress(0.0);
    }
  }
    writer.Close();
}

//-----------------------------------------------------------------------------
vtkLidarReader::vtkLidarReader()
{
  this->Internal = new vtkLidarReaderInternal(this);
}

//-----------------------------------------------------------------------------
vtkLidarReader::~vtkLidarReader()
{
  delete this->Internal;
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

  if (!this->Interpreter->GetIsCalibrated())
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
  output->ShallowCopy(this->GetFrame(timestep, this->Interpreter->GetNumberOfTrailingFrames()));
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
