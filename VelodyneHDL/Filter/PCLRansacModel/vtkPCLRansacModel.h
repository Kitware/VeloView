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

/**
 * @brief The vtkPCLRansacModel class will quickly be replace by classes from the pcl plugin
 * so no time should be spend developping this class
 */
class VTK_EXPORT vtkPCLRansacModel : public vtkPolyDataAlgorithm
{
  public:
  static vtkPCLRansacModel *New();
  vtkTypeMacro(vtkPCLRansacModel, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  enum Model {
    Circle2D = 0,
    Circle3D,
    Cone,       // not implemented
    Cylinder,   // not implemented
    Shpere,
    Line,
    Plane
  };

  vtkGetMacro(DistanceThreshold, double)
  vtkSetMacro(DistanceThreshold, double)

  vtkGetMacro(ModelType, int)
  vtkSetMacro(ModelType, int)

protected:
  // constructor / destructor
  vtkPCLRansacModel();
  ~vtkPCLRansacModel();

  // Request data
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

  //! Maxinum distance from point to model, to consider the point part of the model
  double DistanceThreshold;

  //! Model to approximate
  int ModelType;


private:
  // copy operators
  vtkPCLRansacModel(const vtkPCLRansacModel&);
  void operator=(const vtkPCLRansacModel&);
};

#endif // vtkPCLRansacModel_h
