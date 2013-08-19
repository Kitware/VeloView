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

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>
#include <string>

class VTK_EXPORT vtkVelodyneHDLReader : public vtkPolyDataAlgorithm
{
public:
  static vtkVelodyneHDLReader *New();
  vtkTypeMacro(vtkVelodyneHDLReader, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  //Description:
  //
  const std::string& GetFileName();
  void SetFileName(const std::string& filename);

  //Description:
  //
  const std::string& GetCorrectionsFile();
  void SetCorrectionsFile(const std::string& correctionsFile);

  //Description:
  //
  int CanReadFile(const char* fname);

  void Open();
  void Close();
  int ReadFrameInformation();
  int GetNumberOfFrames();
  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber);

  void DumpFrames(int startFrame, int endFrame, const std::string& filename);

  void ProcessHDLPacket(unsigned char *data, unsigned int bytesReceived);
  std::vector<vtkSmartPointer<vtkPolyData> >& GetDatasets();

  class vtkInternal;

protected:
  vtkVelodyneHDLReader();
  ~vtkVelodyneHDLReader();

  int RequestInformation(vtkInformation *,
                         vtkInformationVector **,
                         vtkInformationVector *);

  int RequestData(vtkInformation *,
                  vtkInformationVector **,
                  vtkInformationVector *);


  void UnloadData();
  void SetTimestepInformation(vtkInformation *info);

  std::string CorrectionsFile;
  std::string FileName;


  vtkInternal* Internal;

private:

  vtkVelodyneHDLReader(const vtkVelodyneHDLReader&);
  void operator = (const vtkVelodyneHDLReader&);

};
#endif
