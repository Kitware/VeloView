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
  Module:    vtkVelodyneHDLReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/


#include "vtkVelodyneHDLReader.h"
#include "VelodynePacketInterpretor.h"

#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <sstream>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLReader);

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::vtkVelodyneHDLReader()
  : vtkLidarReader()
{
  this->Interpretor = new VelodynePacketInterpretor;
  vtkLidarProvider::SetInterpretor(this->Interpretor);
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::GetFrame(int frameNumber, int wantedNumberOfTrailingFrames)
{
  vtkSmartPointer<vtkPolyData> output = vtkLidarReader::GetFrame(frameNumber, wantedNumberOfTrailingFrames);
  if (this->Interpretor->ShouldAddDualReturnArray)
  {
    output->GetPointData()->AddArray(this->Interpretor->SelectedDualReturn);
  }
  return output;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FileName: " << this->GetFileName() << endl;
  os << indent << "CalibrationFile: " << this->Interpretor->GetCalibrationFileName() << endl;
}

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::~vtkVelodyneHDLReader()
{
}

//-----------------------------------------------------------------------------
std::string vtkVelodyneHDLReader::GetSensorInformation()
{
  std::stringstream streamInfo;
  streamInfo << "Factory Field 1: " << (int)this->Interpretor->ReportedFactoryField1 << " (hex: 0x"
             << std::hex << (int)this->Interpretor->ReportedFactoryField1 << std::dec << " ) "
             << DataPacketFixedLength::DualReturnSensorModeToString(
                  static_cast<DataPacketFixedLength::DualReturnSensorMode>(this->Interpretor->ReportedFactoryField1))
             << "  |  "
             << "Factory Field 2: " << (int)this->Interpretor->ReportedFactoryField2 << " (hex: 0x"
             << std::hex << (int)this->Interpretor->ReportedFactoryField2 << std::dec << " ) "
             << DataPacketFixedLength::SensorTypeToString(
                  static_cast<SensorType>(this->Interpretor->ReportedFactoryField2));

  return std::string(streamInfo.str());
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetOutputPacketProcessingDebugInfo() const
{
  return this->Interpretor->OutputPacketProcessingDebugInfo;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetOutputPacketProcessingDebugInfo(int value)
{
  if (this->Interpretor->OutputPacketProcessingDebugInfo != value)
  {
    this->Interpretor->OutputPacketProcessingDebugInfo = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetIntraFiringAdjust() const
{
  return this->Interpretor->UseIntraFiringAdjustment;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetIntraFiringAdjust(int value)
{
  if (this->Interpretor->UseIntraFiringAdjustment != value)
  {
    this->Interpretor->UseIntraFiringAdjustment = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
unsigned int vtkVelodyneHDLReader::GetDualReturnFilter() const
{
  return this->Interpretor->DualReturnFilter;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetDualReturnFilter(unsigned int filter)
{
  if (this->Interpretor->DualReturnFilter != filter)
  {
    this->Interpretor->DualReturnFilter = filter;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
  double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
  double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
  double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
  double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
  double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
  double maxIntensity[HDL_MAX_NUM_LASERS])
{
  for (int i = 0; i < HDL_MAX_NUM_LASERS; ++i)
  {
    verticalCorrection[i] = this->Interpretor->laser_corrections_[i].verticalCorrection;
    rotationalCorrection[i] = this->Interpretor->laser_corrections_[i].rotationalCorrection;
    distanceCorrection[i] = this->Interpretor->laser_corrections_[i].distanceCorrection;
    distanceCorrectionX[i] = this->Interpretor->laser_corrections_[i].distanceCorrectionX;
    distanceCorrectionY[i] = this->Interpretor->laser_corrections_[i].distanceCorrectionY;
    verticalOffsetCorrection[i] = this->Interpretor->laser_corrections_[i].verticalOffsetCorrection;
    horizontalOffsetCorrection[i] =
      this->Interpretor->laser_corrections_[i].horizontalOffsetCorrection;
    focalDistance[i] = this->Interpretor->laser_corrections_[i].focalDistance;
    focalSlope[i] = this->Interpretor->laser_corrections_[i].focalSlope;
    minIntensity[i] = this->Interpretor->laser_corrections_[i].minIntensity;
    maxIntensity[i] = this->Interpretor->laser_corrections_[i].maxIntensity;
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::GetXMLColorTable(double XMLColorTable[4 * HDL_MAX_NUM_LASERS])
{
  for (int i = 0; i < HDL_MAX_NUM_LASERS; ++i)
  {
    XMLColorTable[i * 4] = static_cast<double>(i) / 63.0 * 255.0;
    for (int j = 0; j < 3; ++j)
    {
      XMLColorTable[i * 4 + j + 1] = this->Interpretor->XMLColorTable[i][j];
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetFiringsSkip(int pr)
{
  this->Interpretor->FiringsSkip = pr;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCalibrationFileName(const std::string& filename)
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

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetShouldAddDualReturnArray(bool input)
{
  this->Interpretor->ShouldAddDualReturnArray = input;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetSelectedPointsWithDualReturn(double* data, int Npoints)
{
  this->Interpretor->SelectedDualReturn = vtkSmartPointer<vtkDoubleArray>::New();
  this->Interpretor->SelectedDualReturn->Allocate(60000);
  this->Interpretor->SelectedDualReturn->SetName("dualReturn_of_selectedPoints");

  for (unsigned int k = 0; k < Npoints; ++k)
  {
    this->Interpretor->SelectedDualReturn->InsertNextValue(data[k]);
  }
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::GetHasDualReturn()
{
  return this->Interpretor->HasDualReturn;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::getIsHDL64Data()
{
  return this->Interpretor->IsHDL64Data;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::IsIntensityCorrectedBySensor()
{
  return this->Interpretor->SensorPowerMode == CorrectionOn;
}

//-----------------------------------------------------------------------------
const bool& vtkVelodyneHDLReader::GetWantIntensityCorrection()
{
  return this->Interpretor->WantIntensityCorrection;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetIntensitiesCorrected(const bool& state)
{

  if (state != this->Interpretor->WantIntensityCorrection)
  {
    this->Interpretor->WantIntensityCorrection = state;
    this->Modified();
  }
}
