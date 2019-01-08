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
  Module:    vtkVelodyneHDLStream.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkVelodyneHDLStream -
// .SECTION Description
//


#ifndef __vtkVelodyneHDLStream_h
#define __vtkVelodyneHDLStream_h

#include "vtkLidarStream.h"
#include "vtkDataPacket.h"
#include "VelodynePacketInterpreter.h"

using DataPacketFixedLength::HDL_MAX_NUM_LASERS;

class vtkInternal;

class VTK_EXPORT vtkVelodyneHDLStream : public vtkLidarStream
{
public:
  static vtkVelodyneHDLStream* New();
  vtkTypeMacro(vtkVelodyneHDLStream, vtkLidarStream);
  void PrintSelf(ostream& os, vtkIndent indent);

  std::string GetSensorInformation() override;
  void SetCalibrationFileName(const std::string& filename) override;

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
  VelodynePacketInterpreter* Interpreter;

  vtkVelodyneHDLStream();
  virtual ~vtkVelodyneHDLStream();

  vtkVelodyneHDLStream(const vtkVelodyneHDLStream&); // Not implemented.
  void operator=(const vtkVelodyneHDLStream&);       // Not implemented.
};

#endif
