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
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVelodyneHDLSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkVelodyneHDLSource.h"
#include "vtkAppendPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"

#include "vtkPolyData.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkTransform.h"
#include "vtkVelodyneHDLReader.h"
#include "NetworkSource.h"
#include "PacketConsumer.h"
#include "PacketFileWriter.h"
#include "VelodynePacketInterpretor.h"


#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <pqApplicationCore.h>

#include <stdlib.h>

using DataPacketFixedLength::HDL_MAX_NUM_LASERS;

//----------------------------------------------------------------------------
/**
* \class vtkVelodyneHDLSource::vtkInternal
* \brief This class is responsible for Consumer, the Writer and the NetWorkSource classes
*/
class vtkInternal : public vtkLidarStreamInternal
{
public:
  /**
* \function vtkVelodyneHDLSource::vtkInternal
* \brief Constructor allowing customizable listening port
* @param argLIDARPort The used port to receive the lidar data
*/
  vtkInternal()
    : vtkLidarStreamInternal(2368, 5555, "0.0.0.0", true, false)
  {
  }

  ~vtkInternal() {}
};

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLSource);


//-----------------------------------------------------------------------------
std::string vtkVelodyneHDLSource::GetSensorInformation()
{
  VelodynePacketInterpretor* tmp = reinterpret_cast < VelodynePacketInterpretor* > (this->Interpretor);
  std::stringstream streamInfo;
  streamInfo << "Factory Field 1: " << tmp->ReportedFactoryField1 << " (hex: 0x"
             << std::hex << tmp->ReportedFactoryField1 << std::dec << " ) "
             << DataPacketFixedLength::DualReturnSensorModeToString(
                  static_cast<DataPacketFixedLength::DualReturnSensorMode>(tmp->ReportedFactoryField1))
             << "  |  "
             << "Factory Field 2: " << (int)tmp->ReportedFactoryField2 << " (hex: 0x"
             << std::hex << (int)tmp->ReportedFactoryField2 << std::dec << " ) "
             << DataPacketFixedLength::SensorTypeToString(
                  static_cast<SensorType>(tmp->ReportedFactoryField2));

  return std::string(streamInfo.str());
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetCalibrationFileName(const std::string &filename)
{
  if (filename.empty())
  {
    // no calibration file: HDL64 with autocalibration
    this->Interpretor->SetIsCalibrated(false);
    this->Interpretor->IsCorrectionFromLiveStream = true;
  }
  else
  {
    vtkLidarProvider::SetCalibrationFileName(filename);
    this->Interpretor->IsCorrectionFromLiveStream = false;
  }
}

//----------------------------------------------------------------------------
vtkVelodyneHDLSource::vtkVelodyneHDLSource()
{
  this->Interpretor = new VelodynePacketInterpretor;
  this->Internal = new vtkInternal;
  this->Internal->Consumer->SetInterpretor(this->Interpretor);
  vtkLidarStream::SetPimpInternal(this->Internal, this->Interpretor);
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkVelodyneHDLSource::~vtkVelodyneHDLSource()
{
  this->Stop();
  delete this->Internal;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLSource::GetHasDualReturn()
{
  return this->Interpretor->HasDualReturn;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::UnloadDatasets()
{
  this->Internal->Consumer->UnloadData();
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLSource::GetIntraFiringAdjust() const
{
//  return this->Internal->Consumer->GetReader()->GetIntraFiringAdjust();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetIntraFiringAdjust(int value)
{
//  this->Internal->Consumer->GetReader()->SetIntraFiringAdjust(value);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
  double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
  double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
  double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
  double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
  double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
  double maxIntensity[HDL_MAX_NUM_LASERS])
{
  boost::lock_guard<boost::mutex> lock(this->Internal->Consumer->ReaderMutex);

//  this->Internal->Consumer->GetReader()->GetLaserCorrections(verticalCorrection,
//    rotationalCorrection, distanceCorrection, distanceCorrectionX, distanceCorrectionY,
//    verticalOffsetCorrection, horizontalOffsetCorrection, focalDistance, focalSlope, minIntensity,
//    maxIntensity);
  this->Modified();
}
//-----------------------------------------------------------------------------
unsigned int vtkVelodyneHDLSource::GetDualReturnFilter() const
{
//  return this->Internal->Consumer->GetReader()->GetDualReturnFilter();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetDualReturnFilter(unsigned int filter)
{
//  this->Internal->Consumer->GetReader()->SetDualReturnFilter(filter);
  this->Modified();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Start()
{
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
void vtkVelodyneHDLSource::Stop()
{
  this->Internal->Network->Stop();
  this->Internal->Consumer->Stop();
  this->Internal->Writer->Stop();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Poll()
{
  if (this->Internal->Consumer->CheckForNewData())
  {
    this->Modified();
  }
}

//----------------------------------------------------------------------------
int vtkVelodyneHDLSource::GetCacheSize()
{
  return this->Internal->Consumer->GetMaxNumberOfDatasets();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetCacheSize(int cacheSize)
{
  if (cacheSize == this->GetCacheSize())
  {
    return;
  }

  this->Internal->Consumer->SetMaxNumberOfDatasets(cacheSize);
  this->Modified();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetIntensitiesCorrected(const bool& state)
{
//  this->Internal->Consumer->GetReader()->SetIntensitiesCorrected(state);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetFiringsSkip(int pr)
{
//  this->Internal->Consumer->GetReader()->SetFiringsSkip(pr);
//  this->Internal->Consumer->GetReader()->Modified();
}
