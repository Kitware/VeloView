/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPointCloudLinearProjector.h
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_POINTCLOUD_LINEAR_PROJECTOR_H
#define VTK_POINTCLOUD_LINEAR_PROJECTOR_H

// VTK
#include <vtkPolyData.h>
#include <vtkImageAlgorithm.h>
#include <vtkSmartPointer.h>

// EIGEN
#include <Eigen/Dense>

/**
 * @brief vtkPointCloudLinearProjector Projects a 3D point cloud
 *        onto a plane and creates an image whose pixels represent
 *        a rank filter value of the projected 3D points that lies
 *        within a same pixel
 */
class VTK_EXPORT vtkPointCloudLinearProjector : public vtkImageAlgorithm
{
public:
  static vtkPointCloudLinearProjector *New();
  vtkTypeMacro(vtkPointCloudLinearProjector, vtkImageAlgorithm)

  vtkGetVector2Macro(Resolution, unsigned int)
  vtkSetVector2Macro(Resolution, unsigned int)

  vtkGetMacro(RankPercentile, double)
  vtkSetMacro(RankPercentile, double)

  vtkGetMacro(HeightMap, bool)
  vtkSetMacro(HeightMap, bool)

  vtkGetMacro(ExportAsChar, bool)
  vtkSetMacro(ExportAsChar, bool)

  vtkGetMacro(ShiftToZero, bool)
  vtkSetMacro(ShiftToZero, bool)

  vtkGetMacro(ShouldMedianFilter, bool)
  vtkSetMacro(ShouldMedianFilter, bool)

  vtkGetMacro(MedianFilterWidth, int)
  vtkSetMacro(MedianFilterWidth, int)

  // set the plane normal coordinates on which points are projected
  void SetPlaneNormal(double w0, double w1, double w2);

protected:
  vtkPointCloudLinearProjector() = default;
  ~vtkPointCloudLinearProjector() = default;

  int FillInputPortInformation(int port, vtkInformation *info) override;
  int RequestInformation(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector) override;
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

private:
  vtkPointCloudLinearProjector(const vtkPointCloudLinearProjector&) = delete;
  void operator=(const vtkPointCloudLinearProjector&) = delete;

  // Information of the Image data
  double Origin[3] = {0, 0, 0};
  unsigned int Resolution[2] = {750, 750};
  double PixelSize[2] = {0.1, 0.1};
  double Spacing[3] = {1, 1, 1};

  // Percentile to extract when performing rank filter.
  double RankPercentile = 0.5;

  // Show the height of the projected points instead of one of the array values.
  bool HeightMap = false;

  // Export as double if false or unsigned char if true.
  bool ExportAsChar = false;

  // Shift all values so that the minimum lies at 0.
  bool ShiftToZero = false;

  // Run median filter or nit
  bool ShouldMedianFilter = false;
  int MedianFilterWidth = 3;

  // Information about the projector
  Eigen::Matrix3d DiagonalizedProjector = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d ChangeOfBasis = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Projector = Eigen::Matrix3d::Identity();
};

#endif // VTK_POINTCLOUD_LINEAR_PROJECTOR_H
