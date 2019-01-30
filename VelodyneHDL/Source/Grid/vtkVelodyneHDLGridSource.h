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
  static vtkVelodyneHDLGridSource* New();
  vtkTypeMacro(vtkVelodyneHDLGridSource, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent);

  vtkSetMacro(GridNbTicks, int);
  vtkGetMacro(GridNbTicks, int);

  vtkSetMacro(Scale, double);
  vtkGetMacro(Scale, double);

  vtkSetMacro(LineWidth, int);
  vtkGetMacro(LineWidth, int);

  vtkSetMacro(DistanceResolutionM, double);
  vtkGetMacro(DistanceResolutionM, double);

  vtkSetVector3Macro(Origin, double);
  vtkGetVector3Macro(Origin, double);

  vtkSetVector3Macro(Normal, double);
  vtkGetVector3Macro(Normal, double);

  vtkSetVector3Macro(Color, double);
  vtkGetVector3Macro(Color, double);

  static vtkSmartPointer<vtkPolyData> CreateGrid(
    int gridNbTicks, double scale, double origin[3], double normal[3]);

protected:
  vtkVelodyneHDLGridSource();
  ~vtkVelodyneHDLGridSource();

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*);

  // width of the grid in number of square
  int GridNbTicks;
  double Scale;
  double DistanceResolutionM;
  int LineWidth;
  double Origin[3];
  double Normal[3];
  double Color[3];

private:
  vtkVelodyneHDLGridSource(const vtkVelodyneHDLGridSource&);
  void operator=(const vtkVelodyneHDLGridSource&);
};
#endif
