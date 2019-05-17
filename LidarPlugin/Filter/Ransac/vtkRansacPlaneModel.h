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

#include <vtkPolyDataAlgorithm.h>

class VTK_EXPORT vtkRansacPlaneModel : public vtkPolyDataAlgorithm
{
public:
  static vtkRansacPlaneModel *New();
  vtkTypeMacro(vtkRansacPlaneModel, vtkPolyDataAlgorithm)

  /// Get/Set the maximum number of ransac iterations
  vtkGetMacro(MaxRansacIteration, unsigned int)
  vtkSetMacro(MaxRansacIteration, unsigned int)

  /// Get/Set the distance to plane inlier / outlier threshold
  vtkGetMacro(Threshold, double)
  vtkSetMacro(Threshold, double)

  /// Get/Set the ratio of inliers required to break the ransac algorithm loop
  vtkGetMacro(RatioInliersRequired, double)
  vtkSetMacro(RatioInliersRequired, double)

  /// Get/Set the plane fitted parameters
  vtkGetVector4Macro(PlaneParam, double)
  vtkSetVector4Macro(PlaneParam, double)

  /// Get/Set the option to apply alignment to the output polydata
  vtkGetMacro(AlignOutput, bool)
  vtkSetMacro(AlignOutput, bool)

  /// Get/Set temporal averaging
  vtkGetMacro(TemporalAveraging, bool)
  vtkSetMacro(TemporalAveraging, bool)

  /// Get/Set the maximal angle difference between the new plane estimate and the previous one
  vtkGetMacro(MaxTemporalAngleChange, double)
  vtkSetMacro(MaxTemporalAngleChange, double)

  /// Get/Set how much the previous estimation is used in temporal averaging
  vtkGetMacro(PreviousEstimationWeight, double)
  vtkSetMacro(PreviousEstimationWeight, double)

protected:
  vtkRansacPlaneModel() = default;

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  vtkRansacPlaneModel(const vtkRansacPlaneModel&) = delete;
  void operator=(const vtkRansacPlaneModel&) = delete;

  /// maximum ransac iteration
  unsigned int MaxRansacIteration = 500;

  /// distance to plane inlier / outlier threshold
  double Threshold = 0.5;

  /// ratio of inliers required to break the ransac algorithm loop
  double RatioInliersRequired = 0.3;

  /// plane fitted parameters
  double PlaneParam[4] = {0, 0, 0, 0};

  /// apply alignment to the output polydata
  bool AlignOutput = false;

  /// temporal averaging
  bool TemporalAveraging = true;

  /// maximal angle difference between the new plane estimate and the previous one
  double MaxTemporalAngleChange = 45.0;

  /// how much the previous estimation is used in temporal averaging
  double PreviousEstimationWeight = 0.9;
};

#endif // VTK_RANSAC_PLANE_MODEL_H
