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
  Module:    vtkVelodyneHDLPositionReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkVelodyneHDLPositionReader - class for reading Velodyne HDL data
// .Section Description
//

#ifndef _vtkVelodyneHDLPositionReader_h
#define _vtkVelodyneHDLPositionReader_h

#include <string>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

class vtkTransform;
class vtkVelodyneTransformInterpolator;

class VTK_EXPORT vtkVelodyneHDLPositionReader : public vtkPolyDataAlgorithm
{
public:
  static vtkVelodyneHDLPositionReader* New();
  vtkTypeMacro(vtkVelodyneHDLPositionReader, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  //
  const std::string& GetFileName();
  void SetFileName(const std::string& filename);
  void SetShouldWarnOnWeirdGPSData(bool ShouldWarnOnWeirdGPSData_);
  void SetCalibrationTransform(vtkTransform* transform);
  std::string GetTimeSyncInfo();

  // Default is false (disabled)
  // If disabled, only GPRMC sentences will be used, they do not provide altitude
  // so z = 0 is used.
  // If enabled, GPRMC sentences will be ignored and GPGGA sentences will be
  // used.
  // If available, the altitude used will be the height above the ellipsoid,
  // because that is what was used as datum when projecting.
  // (could be changed to height above geoid).
  void SetUseGPGGASentences(bool useGPGGASentences);

  vtkVelodyneTransformInterpolator* GetInterpolator();

  // field names starts with PPS_ because accessed from python wrapping which
  // does not scope using the name of the enum
  // Warning: PPS_LOCKED does not mean that Lidar is synchronized with GPS:
  // see documentation "Webserver User Guide (VLP-16 & HDL-32E)".
  enum PPSState { PPS_ABSENT = 0, PPS_ATTEMPTING_TO_SYNC, PPS_LOCKED, PPS_ERROR };
  vtkGetMacro(LastPPSState, PPSState)
  vtkGetMacro(PPSSynced, bool)
  vtkGetMacro(HasTimeshiftEstimation, bool)
  vtkGetMacro(AssumedHardwareLag, double)
  vtkSetMacro(AssumedHardwareLag, double)
  double GetTimeshiftEstimation();

protected:
  vtkVelodyneHDLPositionReader();
  virtual ~vtkVelodyneHDLPositionReader();

  virtual int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*);

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*);

  void Open();
  void Close();

  std::string FileName;

  class vtkInternal;
  vtkInternal* Internal;

private:
  bool ShouldWarnOnWeirdGPSData;
  bool UseGPGGASentences;
  bool PPSSynced;
  PPSState LastPPSState;
  // HasTimeshiftEstimation can be used to tell if the timeshift between GPS UTC time
  // and Lidar ToH time could becomputed.
  bool HasTimeshiftEstimation;
  // If HasTimeshiftEstimation is true, all measurements of the timeshift are
  // available in this vector. Add the timeshift to lidar time ToH time to get
  // gps time UTC (mod 3600).
  std::vector<double> TimeshiftMeasurements;
  // AssumedHardwareLag is used to compute TimeshiftEstimation,
  // when there is no PPS sync, but we have some NMEA messages with valid fixes.
  // AssumedHardwareLag is the sum of:
  // 1) the time it takes the GPS to emit the first NMEA message after a new fix
  // 2) the time it take this NMEA message to travel over serial link to lidar
  // 3) the time it takes the lidar to decode this NMEA message and place it in
  // a position packet.
  // This value depends of hardware, firmware version and maybe temperature.
  // 0.094 s was measured on two private datasets recorded using the same
  // hardware setup. Precision of this measure is approximately 3e-3 seconds.
  double AssumedHardwareLag;
  vtkVelodyneHDLPositionReader(const vtkVelodyneHDLPositionReader&);
  void operator=(const vtkVelodyneHDLPositionReader&);
};
#endif
