/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkBirdEyeViewSnap.h
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_BIRD_EYE_VIEW_H
#define VTK_BIRD_EYE_VIEW_H

// VTK
#include <vtkPolyData.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

// EIGEN
#include <Eigen/Dense>

class VTK_EXPORT vtkBirdEyeViewSnap : public vtkPolyDataAlgorithm
{
public:
  static vtkBirdEyeViewSnap *New();
  vtkTypeMacro(vtkBirdEyeViewSnap, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent);

  // Set Orientation of the plane
  // to generate the bird eye view
  void SetPlaneParam(double params[4]);

  // Set the folder to save the views generated
  void SetFolderName(std::string filename);

  // Set the pixel size in meters of the output image
  void SetResolution(double sX, double sY);

  // set the count value used to name files
  void SetCount(unsigned int count);

protected:
  // constructor / destructor
  vtkBirdEyeViewSnap();
  ~vtkBirdEyeViewSnap();

  // Request data
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  // copy operators
  vtkBirdEyeViewSnap(const vtkBirdEyeViewSnap&);
  void operator=(const vtkBirdEyeViewSnap&);

  // folder to save the bird eye
  // views generated
  std::string RadicalFileName;
  std::string ExtensionFileName;

  // Rotation to apply to the input
  // lidar frame before applying the
  // orthogonal projection on the OXY plane
  Eigen::Matrix<double, 3, 3> Orientation;

  // count of generated views
  unsigned int Count;

  // size of a pixel in meters
  double pixelResX;
  double pixelResY;
};

#endif // VTK_BIRD_EYE_VIEW_H