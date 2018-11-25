/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vvtkPCLRansacModel.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef vtkPCLRansacModel_h
#define vtkPCLRansacModel_h

// vtk includes
#include <vtkPolyData.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

class VTK_EXPORT vtkPCLRansacModel : public vtkPolyDataAlgorithm
{
    public:
  static vtkPCLRansacModel *New();
  vtkTypeMacro(vtkPCLRansacModel, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  // constructor / destructor
  vtkPCLRansacModel();
  ~vtkPCLRansacModel();

  // Request data
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

  class vtkInternal;
  vtkInternal* Internal;

private:
  // copy operators
  vtkPCLRansacModel(const vtkPCLRansacModel&);
  void operator=(const vtkPCLRansacModel&);
};

#endif // vtkPCLRansacModel_h