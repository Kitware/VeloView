/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkRansacPlaneModel.h
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_RANSAC_PLANE_MODEL_H
#define VTK_RANSAC_PLANE_MODEL_H

// VTK
#include <vtkPolyData.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

class VTK_EXPORT vtkRansacPlaneModel : public vtkPolyDataAlgorithm
{
    public:
  static vtkRansacPlaneModel *New();
  vtkTypeMacro(vtkRansacPlaneModel, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Set the maximum number of iteration for ransac
  void SetMaximumIteration(unsigned int maxIt);

  // Set the threshold
  void SetThreshold(double thresh);

  // Set the Ratio inlier
  void SetRatioInlierRequired(double ratio);

  // Get the fitted plane parameters
  void GetPlaneParam(double param[4]);

protected:
  // constructor / destructor
  vtkRansacPlaneModel();
  ~vtkRansacPlaneModel();

  // Request data
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  // copy operators
  vtkRansacPlaneModel(const vtkRansacPlaneModel&);
  void operator=(const vtkRansacPlaneModel&);

  // maximum ransac iteration
  unsigned int MaxRansacIteration;

  // distance to plane inlier / outlier threshold
  double Threshold;

  // ratio of inliers required to break the
  // ransac algorithm loop
  double RatioInliersRequired;

  // plane fitted parameters
  double Param[4];
};

#endif // VTK_RANSAC_PLANE_MODEL_H