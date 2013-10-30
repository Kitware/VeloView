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
// .NAME vtkVelodyneOffsetFilter -
// .SECTION Description
//

#ifndef __vtkVelodyneOffsetFilter_h
#define __vtkVelodyneOffsetFilter_h

#include <vtkPolyDataAlgorithm.h>

class vtkTupleInterpolator;

class vtkVelodyneOffsetFilter : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkVelodyneOffsetFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkVelodyneOffsetFilter *New();

  void SetInterp(vtkTupleInterpolator* interp);

protected:
  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);

  vtkVelodyneOffsetFilter();
  virtual ~vtkVelodyneOffsetFilter();

private:
  vtkVelodyneOffsetFilter(const vtkVelodyneOffsetFilter&);  // Not implemented.
  void operator=(const vtkVelodyneOffsetFilter&);  // Not implemented.

  class vtkInternal;
  vtkInternal * Internal;
};

#endif
