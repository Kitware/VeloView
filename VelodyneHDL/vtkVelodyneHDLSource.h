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
  Module:    vtkVelodyneHDLSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkVelodyneHDLSource -
// .SECTION Description
//

#ifndef __vtkVelodyneHDLSource_h
#define __vtkVelodyneHDLSource_h

#include <vtkPolyDataAlgorithm.h>

#include "vtkDataPacket.h"
using DataPacketFixedLength::HDL_MAX_NUM_LASERS;

class vtkTransform;

class VTK_EXPORT vtkVelodyneHDLSource : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkVelodyneHDLSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkVelodyneHDLSource* New();

  void Poll();

  void Start();
  void Stop();

  int GetCacheSize();
  void SetCacheSize(int cacheSize);

  void ReadNextFrame();

  const std::string& GetCorrectionsFile();
  void SetCorrectionsFile(const std::string& correctionsFile);

  const std::string& GetOutputFile();
  void SetOutputFile(const std::string& filename);

  const std::string& GetForwardedIpAddress();
  void SetForwardedIpAddress(const std::string& ipAddress);

  vtkSetMacro(LIDARPort, int);
  vtkGetMacro(LIDARPort, int);

  vtkSetMacro(GPSPort, int);
  vtkGetMacro(GPSPort, int);

  vtkSetMacro(ForwardedLIDARPort, int);
  vtkGetMacro(ForwardedLIDARPort, int);

  vtkSetMacro(ForwardedGPSPort, int);
  vtkGetMacro(ForwardedGPSPort, int);

  vtkSetMacro(isForwarding, bool);
  vtkGetMacro(isForwarding, bool);

  vtkSetMacro(isCrashAnalysing, bool);
  vtkGetMacro(isCrashAnalysing, bool);

  void SetLaserSelection(int LaserSelection[HDL_MAX_NUM_LASERS]);
  void GetLaserSelection(int LaserSelection[HDL_MAX_NUM_LASERS]);

  double GetDistanceResolutionM();

  void SetCropMode(int);
  void SetCropReturns(int);
  void SetCropOutside(int);
  void SetCropRegion(double[6]);
  void SetCropRegion(double, double, double, double, double, double);

  void GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
    double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
    double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
    double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
    double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
    double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
    double maxIntensity[HDL_MAX_NUM_LASERS]);

  unsigned int GetDualReturnFilter() const;
  void SetDualReturnFilter(unsigned int);

  void SetSensorTransform(vtkTransform*);

  // A trick to workaround failure to wrap LaserSelection
  void SetDummyProperty(int);

  int GetNumberOfChannels();

  void SetIntensitiesCorrected(const bool& state);

  bool GetHasDualReturn();

  int GetIgnoreZeroDistances() const;
  void SetIgnoreZeroDistances(int);

  int GetIgnoreEmptyFrames() const;
  void SetIgnoreEmptyFrames(int);

  int GetIntraFiringAdjust() const;
  void SetIntraFiringAdjust(int);

  bool GetCorrectionsInitialized();

  void UnloadDatasets();

protected:
  virtual int RequestInformation(vtkInformation* request, vtkInformationVector** inputVector,
    vtkInformationVector* outputVector);

  virtual int RequestData(vtkInformation* request, vtkInformationVector** inputVector,
    vtkInformationVector* outputVector);

  vtkVelodyneHDLSource();
  virtual ~vtkVelodyneHDLSource();

  int LIDARPort;                  /*!< The port to receive LIDAR information. Default is 2368 */
  int GPSPort;                    /*!< The port to receive GPS information. Default is 8308 */
  int ForwardedLIDARPort;         /*!< The port to send LIDAR forwarded packets*/
  int ForwardedGPSPort;           /*!< The port to send GPS forwarded packets*/
  std::string ForwardedIpAddress; /*!< The ip to send forwarded packets*/
  bool isForwarding;              /*!< Allowing the forwarding of the packets*/
  bool isCrashAnalysing;
  std::string PacketFile;
  std::string OutputFile;
  std::string CorrectionsFile;

private:
  vtkVelodyneHDLSource(const vtkVelodyneHDLSource&); // Not implemented.
  void operator=(const vtkVelodyneHDLSource&);       // Not implemented.

  class vtkInternal;
  vtkInternal* Internal;
};

#endif
