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
// .NAME vtkVelodyneHDLReader - class for reading Velodyne HDL data
// .Section Description
//

#ifndef _vtkVelodyneHDLReader_h
#define _vtkVelodyneHDLReader_h

#include "vtkLidarReader.h"
#include "vtkDataPacket.h"
#include <string>
#include <vtkSmartPointer.h>

class vtkVelodyneTransformInterpolator;
class VelodynePacketInterpretor;
class vtkInternal;

using DataPacketFixedLength::HDL_MAX_NUM_LASERS;

class VTK_EXPORT vtkVelodyneHDLReader : public vtkLidarReader
{
public:

public:
  static vtkVelodyneHDLReader* New();
  vtkTypeMacro(vtkVelodyneHDLReader, vtkLidarReader);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Information about the sensor from dataPacket
  std::string GetSensorInformation() override;
  void SetCalibrationFileName(const std::string& filename) override;
  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrames = 0) override;

  // Description:
  //
  bool IsIntensityCorrectedBySensor();
  const bool& GetWantIntensityCorrection();
  void SetIntensitiesCorrected(const bool& state);

  void GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
    double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
    double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
    double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
    double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
    double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
    double maxIntensity[HDL_MAX_NUM_LASERS]);

  void GetXMLColorTable(double XMLColorTable[]);

  int GetOutputPacketProcessingDebugInfo() const;
  void SetOutputPacketProcessingDebugInfo(int);

  int GetIntraFiringAdjust() const;
  void SetIntraFiringAdjust(int);

  unsigned int GetDualReturnFilter() const;
  void SetDualReturnFilter(unsigned int);

  void SetFiringsSkip(int);

  bool getIsHDL64Data();

  bool GetHasDualReturn();

  // This function permits to know which are the points selected
  // with a corresponding dual return
  void SetSelectedPointsWithDualReturn(double* data, int Npoints);
  void SetShouldAddDualReturnArray(bool input);

private:
  vtkInternal* Internal;
  VelodynePacketInterpretor* Interpretor;

  vtkVelodyneHDLReader();
  ~vtkVelodyneHDLReader();
  vtkVelodyneHDLReader(const vtkVelodyneHDLReader&); // not implemented
  void operator=(const vtkVelodyneHDLReader&); // not implemented
};
#endif
