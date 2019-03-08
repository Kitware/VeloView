//=========================================================================
//
// Copyright 2018 Kitware, Inc.
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
//=========================================================================

// LOCAL
#include "vtkLidarStream.h"
#include "NetworkSource.h"
#include "PacketConsumer.h"
#include "PacketFileWriter.h"

// VTK
#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

class vtkLidarStreamInternal
{
public:
  vtkLidarStreamInternal(int argLIDARPort, int ForwardedLIDARPort,
                         std::string ForwardedIpAddress, bool isForwarding, bool isCrashAnalysing)
    : Consumer(new PacketConsumer)
    , Writer(new PacketFileWriter)
    , Network(std::unique_ptr<NetworkSource>(new NetworkSource(this->Consumer, argLIDARPort, ForwardedLIDARPort,
                                                               ForwardedIpAddress, isForwarding, isCrashAnalysing))) {}


  //! where to save a live record of the sensor
  std::string OutputFileName;


  std::shared_ptr<PacketConsumer> Consumer;
  std::shared_ptr<PacketFileWriter> Writer;
  std::unique_ptr<NetworkSource> Network;
};


//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLidarStream)

//-----------------------------------------------------------------------------
vtkLidarStream::vtkLidarStream()
{
  this->Internal = new vtkLidarStreamInternal(2368, 2369, "127.0.0.1", false, false);
}

//-----------------------------------------------------------------------------
vtkLidarStream::~vtkLidarStream()
{
  this->Stop();
  delete this->Internal;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetNumberOfFrames()
{
  std::cerr << "this is not implemented yet" << std::endl;
  return 0;
}

//-----------------------------------------------------------------------------
std::string vtkLidarStream::GetOutputFile()
{
  return this->Internal->OutputFileName;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetOutputFile(const std::string &filename)
{
  this->Internal->OutputFileName  = filename;
}

//-----------------------------------------------------------------------------
std::string vtkLidarStream::GetForwardedIpAddress()
{
  return this->Internal->Network->ForwardedIpAddress;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedIpAddress(const std::string &ipAddress)
{
  this->Internal->Network->ForwardedIpAddress = ipAddress;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetLIDARPort()
{
  return this->Internal->Network->LIDARPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetLIDARPort(const int value)
{
  this->Internal->Network->LIDARPort = value;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetGPSPort()
{
  return this->Internal->Network->GPSPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetGPSPort(const int value)
{
  this->Internal->Network->GPSPort = value;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetForwardedLIDARPort()
{
  return this->Internal->Network->ForwardedLIDARPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedLIDARPort(const int value)
{
  this->Internal->Network->ForwardedLIDARPort = value;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetForwardedGPSPort()
{
  return this->Internal->Network->ForwardedGPSPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedGPSPort(const int value)
{
  this->Internal->Network->ForwardedGPSPort = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetIsForwarding()
{
  return this->Internal->Network->IsForwarding;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::EnableGPSListening(const bool value)
{
  this->Internal->Network->ListenGPS = value;
}


//-----------------------------------------------------------------------------
void vtkLidarStream::SetIsForwarding(bool value)
{
  this->Internal->Network->IsForwarding = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetIsCrashAnalysing()
{
  return this->Internal->Network->IsCrashAnalysing;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetIsCrashAnalysing(const bool value)
{
  this->Internal->Network->IsCrashAnalysing = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetNeedsUpdate()
{
  Poll();
  return true;
}

//----------------------------------------------------------------------------
void vtkLidarStream::Start()
{
  if (!this->Interpreter)
  {
    vtkErrorMacro(<< "Please set a Interpreter")
  }
  this->Internal->Consumer->SetInterpreter(this->Interpreter);
  if (this->Internal->OutputFileName.length())
  {
    this->Internal->Writer->Start(this->Internal->OutputFileName);
  }

  this->Internal->Network->Writer.reset();

  if (this->Internal->Writer->IsOpen())
  {
    this->Internal->Network->Writer = this->Internal->Writer;
  }

  // Check if the IP address is valid
//  {
//    boost::system::error_code ec;
//    boost::asio::ip::address::from_string(this->ForwardedIpAddress, ec);
//    if (ec)
//    {
//      this->ForwardedIpAddress = "0.0.0.0";
//      this->isForwarding = false;
//    }
//  }

  this->Internal->Consumer->Start();
//  this->Internal->Network->LIDARPort = this->LIDARPort;
//  this->Internal->Network->ForwardedLIDARPort = this->ForwardedLIDARPort;
//  this->Internal->Network->ForwardedIpAddress = this->ForwardedIpAddress;
//  this->Internal->Network->isForwarding = this->isForwarding;
//  this->Internal->Network->isCrashAnalysing = this->isCrashAnalysing;
  this->Internal->Network->Start();
}

//----------------------------------------------------------------------------
void vtkLidarStream::Stop()
{
  this->Internal->Network->Stop();
  this->Internal->Consumer->Stop();
  this->Internal->Writer->Stop();
}

//----------------------------------------------------------------------------
void vtkLidarStream::Poll()
{
  if (this->Internal->Consumer->CheckForNewData())
  {
    this->Modified();
  }
}

//----------------------------------------------------------------------------
int vtkLidarStream::GetCacheSize()
{
  return this->Internal->Consumer->GetMaxNumberOfFrames();
}

//----------------------------------------------------------------------------
void vtkLidarStream::SetCacheSize(int cacheSize)
{
  if (cacheSize == this->GetCacheSize())
  {
    return;
  }

  this->Internal->Consumer->SetMaxNumberOfFrames(cacheSize);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarStream::UnloadFrames()
{
  this->Internal->Consumer->UnloadData();
}


//-----------------------------------------------------------------------------
int vtkLidarStream::RequestInformation(vtkInformation* request,
                                       vtkInformationVector** inputVector,
                                       vtkInformationVector* outputVector)
{
  this->Superclass::RequestInformation(request, inputVector, outputVector);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  std::vector<double> timesteps = this->Internal->Consumer->GetTimesteps();
  const size_t nTimesteps = timesteps.size();
  if (nTimesteps > 0)
  {
    outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(),
      static_cast<int>(nTimesteps));
  }
  else
  {
    outInfo->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
  }

  double timeRange[2] = { 0.0, nTimesteps ? nTimesteps - 1.0 : 0.0 };
  outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);

  return 1;
}

//----------------------------------------------------------------------------
int vtkLidarStream::RequestData(vtkInformation* vtkNotUsed(request),
                                vtkInformationVector** vtkNotUsed(inputVector),
                                vtkInformationVector* outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkDataSet* output = vtkDataSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  double timeRequest = 0;
  if (outInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    timeRequest = outInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
  }

  {
    boost::lock_guard<boost::mutex> lock(this->Internal->Consumer->ConsumerMutex);
    double actualTime;
    vtkSmartPointer<vtkPolyData> polyData(NULL);
//  if (this->Internal->Consumer->GetNumberOfTrailingFrames() > 0)
//  {
//    polyData = this->Internal->Consumer->GetFramesForTime(
//      timeRequest, actualTime, this->Internal->Consumer->GetNumberOfTrailingFrames());
//  }
//  else
//  {
    polyData = this->Internal->Consumer->GetFrameForTime(timeRequest, actualTime);
//  }

    if (polyData)
    {
      // printf("request %f, returning %f\n", timeRequest, actualTime);
      output->GetInformation()->Set(vtkDataObject::DATA_TIME_STEP(), actualTime);
      output->ShallowCopy(polyData);
    }
  }

  vtkTable* calibration = vtkTable::GetData(outputVector,1);
  vtkTable *t = this->Interpreter->GetCalibrationTable();
  calibration->ShallowCopy(t);

  return 1;
}
