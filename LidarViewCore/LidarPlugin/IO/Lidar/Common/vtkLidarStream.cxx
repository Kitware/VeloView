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

#include "vtkLidarStream.h"

#include <sstream>

#include "NetworkSource.h"
#include "PacketConsumer.h"
#include "PacketFileWriter.h"

#include <vtkInformationVector.h>
#include <vtkInformation.h>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLidarStream)

//-----------------------------------------------------------------------------
vtkLidarStream::vtkLidarStream()
{
  this->Consumer = std::make_shared<PacketConsumer>();
  this->Writer = std::make_shared<PacketFileWriter>();
  this->Network = std::make_unique<NetworkSource>(this->Consumer, 2368, 2369, "127.0.0.1", false, false);
}

//-----------------------------------------------------------------------------
vtkLidarStream::~vtkLidarStream()
{
  this->Stop();
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
  return this->OutputFileName;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetOutputFile(const std::string &filename)
{
  this->OutputFileName  = filename;
}

//-----------------------------------------------------------------------------
std::string vtkLidarStream::GetForwardedIpAddress()
{
  return this->Network->ForwardedIpAddress;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedIpAddress(const std::string &ipAddress)
{
  this->Network->ForwardedIpAddress = ipAddress;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetLidarPort()
{
  return this->Network->LidarPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetLidarPort(int value)
{
  if (this->Network->LidarPort != value)
  {
    bool wasRunning = this->Network != nullptr
        && this->Network->Thread != nullptr && this->Network->Thread->joinable()
        && this->Network->LidarPortReceiver != nullptr;
    this->Stop();
    this->Network->LidarPort = value;
    if (wasRunning)
    {
      this->Start();
    }
  }
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetGPSPort()
{
  return this->Network->GPSPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetGPSPort(int value)
{
  this->Network->GPSPort = value;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetForwardedLidarPort()
{
  return this->Network->ForwardedLidarPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedLidarPort(int value)
{
  this->Network->ForwardedLidarPort = value;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetForwardedGPSPort()
{
  return this->Network->ForwardedGPSPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedGPSPort(int value)
{
  this->Network->ForwardedGPSPort = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetIsForwarding()
{
  return this->Network->IsForwarding;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::EnableGPSListening(bool value)
{
  this->Network->ListenGPS = value;
}


//-----------------------------------------------------------------------------
void vtkLidarStream::SetIsForwarding(bool value)
{
  this->Network->IsForwarding = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetIsCrashAnalysing()
{
  return this->Network->IsCrashAnalysing;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetIsCrashAnalysing(bool value)
{
  this->Network->IsCrashAnalysing = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetNeedsUpdate()
{
  boost::lock_guard<boost::mutex> lock(this->Consumer->ConsumerMutex);
  if (this->Consumer->CheckForNewData())
  {
    this->Modified();
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------
void vtkLidarStream::Start()
{
  if (!this->Interpreter)
  {
    vtkErrorMacro("no interpreter is set")
  }
  this->Consumer->SetInterpreter(this->Interpreter);
  if (this->OutputFileName.length())
  {
    this->Writer->Start(this->OutputFileName);
  }

  this->Network->Writer.reset();

  if (this->Writer->IsOpen())
  {
    this->Network->Writer = this->Writer;
  }

  this->Consumer->Start();

  this->Network->Start();
}

//----------------------------------------------------------------------------
void vtkLidarStream::Stop()
{
  this->Network->Stop();
  this->Consumer->Stop();
  this->Writer->Stop();
}

//----------------------------------------------------------------------------
int vtkLidarStream::RequestData(vtkInformation* vtkNotUsed(request),
                                vtkInformationVector** vtkNotUsed(inputVector),
                                vtkInformationVector* outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector);

  int numberOfFrameAvailable = 0;
  {
    boost::lock_guard<boost::mutex> lock(this->Consumer->ConsumerMutex);
    numberOfFrameAvailable = this->Consumer->CheckForNewData();
    if (numberOfFrameAvailable != 0)
    {
      vtkSmartPointer<vtkPolyData> polyData = this->Consumer->GetLastAvailableFrame();
      output->ShallowCopy(polyData);
      this->Consumer->ClearAllFrames();
      this->LastFrameProcessed += numberOfFrameAvailable;
    }
  }

  if (this->DetectFrameDropping)
  {
    if (numberOfFrameAvailable > 1)
    {
      std::stringstream text;
      text << "WARNING : At frame " << std::right << std::setw(6) << this->LastFrameProcessed
           << " Drop " << std::right << std::setw(2) << numberOfFrameAvailable-1 << " frame(s)\n";
      vtkWarningMacro( << text.str() )
    }
  }

  vtkTable* calibration = vtkTable::GetData(outputVector,1);
  vtkTable *t = this->Interpreter->GetCalibrationTable();
  calibration->ShallowCopy(t);

  return 1;
}
