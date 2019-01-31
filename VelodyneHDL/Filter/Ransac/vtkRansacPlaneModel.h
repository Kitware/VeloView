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
  vtkTypeMacro(vtkRansacPlaneModel, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent);

  /// Get the maximum number of ransac iterations
  vtkGetMacro(MaxRansacIteration, unsigned int)

  /// Set the maximum number of ransac iterations
  vtkSetMacro(MaxRansacIteration, unsigned int)

  /// Get the distance to plane inlier / outlier threshold
  vtkGetMacro(Threshold, double)

  /// Set the distance to plane inlier / outlier threshold
  vtkSetMacro(Threshold, double)

  /// Get the ratio of inliers required to break the ransac algorithm loop
  vtkGetMacro(RatioInliersRequired, double);

  /// Set the ratio of inliers required to break the ransac algorithm loop
  vtkSetMacro(RatioInliersRequired, double);

  /// Get the plane fitted parameters
  vtkGetVector4Macro(PlaneParam, double)

  /// Set the plane fitted parameters
  vtkSetVector4Macro(PlaneParam, double)

  /// Get the option to apply alignment to the output polydata
  vtkGetMacro(AlignOutput, bool)

  /// Set the option to apply alignment to the output polydata
  vtkSetMacro(AlignOutput, bool)

  /// Get temporal averaging
  vtkGetMacro(TemporalAveraging, bool)

  /// Set temporal averaging
  vtkSetMacro(TemporalAveraging, bool)

  /// Get the maximal angle difference between the new plane estimate and the previous one
  vtkGetMacro(MaxTemporalAngleChange, double)

  /// Set the maximal angle difference between the new plane estimate and the previous one
  vtkSetMacro(MaxTemporalAngleChange, double)

  /// Get how much the previous estimation is used in temporal averaging
  vtkGetMacro(PreviousEstimationWeight, double)

  /// Set how much the previous estimation is used in temporal averaging
  vtkSetMacro(PreviousEstimationWeight, double)

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

  /// maximum ransac iteration
  unsigned int MaxRansacIteration;

  /// distance to plane inlier / outlier threshold
  double Threshold;

  /// ratio of inliers required to break the ransac algorithm loop
  double RatioInliersRequired;

  /// plane fitted parameters
  double PlaneParam[4];

  /// apply alignment to the output polydata
  bool AlignOutput;

  /// temporal averaging
  bool TemporalAveraging;

  /// maximal angle difference between the new plane estimate and the previous one
  double MaxTemporalAngleChange;

  /// how much the previous estimation is used in temporal averaging
  double PreviousEstimationWeight;
};

#endif // VTK_RANSAC_PLANE_MODEL_H