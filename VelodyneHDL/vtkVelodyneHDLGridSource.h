/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVelodyneHDLGridSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkVelodyneHDLGridSource - generates a vtkPolyData measurement grid plane
// .Section Description
//

#ifndef _vtkVelodyneHDLGridSource_h
#define _vtkVelodyneHDLGridSource_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

class VTK_EXPORT vtkVelodyneHDLGridSource : public vtkPolyDataAlgorithm
{
public:
  static vtkVelodyneHDLGridSource *New();
  vtkTypeMacro(vtkVelodyneHDLGridSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  vtkSetMacro(GridSize, int);
  vtkGetMacro(GridSize, int);

  vtkSetMacro(Scale, double);
  vtkGetMacro(Scale, double);

  vtkSetVector3Macro(Origin, double);
  vtkGetVector3Macro(Origin, double);

  vtkSetVector3Macro(Normal, double);
  vtkGetVector3Macro(Normal, double);

  static vtkSmartPointer<vtkPolyData> CreateGrid(int gridSize, double scale, double origin[3], double normal[3]);

protected:
  vtkVelodyneHDLGridSource();
  ~vtkVelodyneHDLGridSource();

  int RequestData(vtkInformation *,
                  vtkInformationVector **,
                  vtkInformationVector *);

  int GridSize;
  double Scale;
  double Origin[3];
  double Normal[3];

private:

  vtkVelodyneHDLGridSource(const vtkVelodyneHDLGridSource&);
  void operator = (const vtkVelodyneHDLGridSource&);

};
#endif
