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

  bool HasNewData();

  void Poll();

  void Start();
  void Stop();

  // Description:
  // Set/get the packet file to read.
  vtkSetStringMacro(PacketFile);
  vtkGetStringMacro(PacketFile);

  vtkSetMacro(SensorPort, int);
  vtkGetMacro(SensorPort, int);

protected:

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);

  vtkVelodyneHDLSource();
  virtual ~vtkVelodyneHDLSource();

  char *PacketFile;
  int SensorPort;

private:
  vtkVelodyneHDLSource(const vtkVelodyneHDLSource&);  // Not implemented.
  void operator=(const vtkVelodyneHDLSource&);  // Not implemented.

  class vtkInternal;
  vtkInternal * Internal;
};

#endif


