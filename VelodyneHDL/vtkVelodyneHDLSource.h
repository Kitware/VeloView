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

class vtkTransform;

class VTK_EXPORT vtkVelodyneHDLSource : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkVelodyneHDLSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkVelodyneHDLSource *New();

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

  vtkSetMacro(SensorPort, int);
  vtkGetMacro(SensorPort, int);

  void SetLaserSelection(int LaserSelection[64]);
  void GetLaserSelection(int LaserSelection[64]);

  void SetCropReturns(int);
  void SetCropInside(int);
  void SetCropRegion(double[6]);
  void SetCropRegion(double, double, double, double, double, double);

  void GetLaserCorrections(
    double verticalCorrection[64],
    double rotationalCorrection[64],
    double distanceCorrection[64],
    double distanceCorrectionX[64],
    double distanceCorrectionY[64],
    double verticalOffsetCorrection[64],
    double horizontalOffsetCorrection[64],
    double focalDistance[64],
    double focalSlope[64],
    double minIntensity[64],
    double maxIntensity[64]);

  unsigned int GetDualReturnFilter() const;
  void SetDualReturnFilter(unsigned int);

  void SetSensorTransform(vtkTransform*);

  // A trick to workaround failure to wrap LaserSelection
  void SetDummyProperty(int);

  int GetNumberOfChannels();

protected:


  virtual int RequestInformation(vtkInformation *request,
                         vtkInformationVector **inputVector,
                         vtkInformationVector *outputVector);

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);

  vtkVelodyneHDLSource();
  virtual ~vtkVelodyneHDLSource();


  int SensorPort;
  std::string PacketFile;
  std::string OutputFile;
  std::string CorrectionsFile;

private:
  vtkVelodyneHDLSource(const vtkVelodyneHDLSource&);  // Not implemented.
  void operator=(const vtkVelodyneHDLSource&);  // Not implemented.

  class vtkInternal;
  vtkInternal * Internal;
};

#endif
