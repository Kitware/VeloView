/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLaplacianInfilling.h
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_LAPLACIAN_INFILLING_H
#define VTK_LAPLACIAN_INFILLING_H

// VTK
#include <vtkImageAlgorithm.h>

/**
 * @brief vtkLaplacianInfilling fill missing data in an image
 *        solving the Dirichlet problem.
 */
class VTK_EXPORT vtkLaplacianInfilling : public vtkImageAlgorithm
{
public:
  static vtkLaplacianInfilling *New();
  vtkTypeMacro(vtkLaplacianInfilling, vtkImageAlgorithm)

protected:
  vtkLaplacianInfilling() = default;
  ~vtkLaplacianInfilling() = default;

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

private:
  vtkLaplacianInfilling(const vtkLaplacianInfilling&) = delete;
  void operator=(const vtkLaplacianInfilling&) = delete;
};

#endif // VTK_LAPLACIAN_INFILLING_H
