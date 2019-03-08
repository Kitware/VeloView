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

  vtkGetVector2Macro(Dimensions, int)
  vtkSetVector2Macro(Dimensions, int)

  vtkGetMacro(RankPercentil, double)
  vtkSetMacro(RankPercentil, double)

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
  int Dimensions[2] = {750, 750};
  double Spacing[3] = {1, 1, 1};

  // percentil to extract when performing rank filter
  double RankPercentil = 0.5;

  // Information about the projector
  Eigen::Matrix3d DiagonalizedProjector = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d ChangeOfBasis = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Projector = Eigen::Matrix3d::Identity();
};

#endif // VTK_POINTCLOUD_LINEAR_PROJECTOR_H
