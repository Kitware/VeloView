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

class vtkVelodyneHDLSource : public vtkPolyDataAlgorithm
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

  const std::string& GetPacketFile();
  void SetPacketFile(const std::string& filename);

  const std::string& GetCorrectionsFile();
  void SetCorrectionsFile(const std::string& correctionsFile);

  const std::string& GetOutputFile();
  void SetOutputFile(const std::string& filename);

  vtkSetMacro(SensorPort, int);
  vtkGetMacro(SensorPort, int);

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
