/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPointCloudLinearProjector.cxx
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// LOCAL
#include "vtkPointCloudLinearProjector.h"
#include "vtkEigenTools.h"

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

// VTK
#include <vtkObjectFactory.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>

// BOOST
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Dense>

// Implementation of the New function
vtkStandardNewMacro(vtkPointCloudLinearProjector)

//-----------------------------------------------------------------------------
int vtkPointCloudLinearProjector::FillInputPortInformation(int port, vtkInformation *info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
int vtkPointCloudLinearProjector::RequestInformation(vtkInformation *vtkNotUsed(request),
                                                vtkInformationVector **vtkNotUsed(inputVector),
                                                vtkInformationVector *outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(),
               0, this->Dimensions[0] - 1,
               0, this->Dimensions[1] - 1,
               0, 0);

  outInfo->Set(vtkDataObject::ORIGIN(),this->Origin, 3);
  outInfo->Set(vtkDataObject::SPACING(),this->Spacing, 3);

  vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_DOUBLE, 1);
  return VTK_OK;
}

//-----------------------------------------------------------------------------
int vtkPointCloudLinearProjector::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get the input
  vtkPolyData * input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  double point[3];
  Eigen::Vector3d X;

  auto transformedPoints = vtkSmartPointer<vtkPoints>::New();
  transformedPoints->Resize(input->GetNumberOfPoints());
  // Transform the input polydata
  for (vtkIdType pointIndex = 0; pointIndex < input->GetNumberOfPoints(); ++pointIndex)
  {
    input->GetPoint(pointIndex, point);
    X << point[0], point[1], point[2];
    X = this->Projector * X;
    point[0] = X(0); point[1] = X(1); point[2] = X(2);
    transformedPoints->SetPoint(pointIndex, point);
  }
  transformedPoints->Modified();

  // Get the point cloud bounding box parameters
  double boundingBox[6];
  transformedPoints->GetBounds(boundingBox);
  this->Spacing[0] = (boundingBox[1] - boundingBox[0]) / static_cast<double>(this->Dimensions[0]);
  this->Spacing[1] = (boundingBox[3] - boundingBox[2]) / static_cast<double>(this->Dimensions[1]);

  // Get the output image and fill with zeros
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
  image->SetDimensions(this->Dimensions[0], this->Dimensions[1], 1);
  image->SetSpacing(this->Spacing);
  image->SetOrigin(this->Origin);
  image->AllocateScalars(VTK_DOUBLE, 1);
  unsigned char* dataPointer = reinterpret_cast<unsigned char*>(image->GetScalarPointer());
  std::fill(dataPointer, dataPointer + this->Dimensions[0] * this->Dimensions[1], 0);

  // Compute distribution about the height of the data lying in a pixel
  std::vector<std::vector<double> > perPixelDistribution(this->Dimensions[0] * this->Dimensions[1]);
  double scaleX = (this->Dimensions[0] - 1) / (boundingBox[1] - boundingBox[0]);
  double scaleY = (this->Dimensions[1] - 1) / (boundingBox[3] - boundingBox[2]);
  for (vtkIdType pointIndex = 0; pointIndex < input->GetNumberOfPoints(); ++pointIndex)
  {
    transformedPoints->GetPoint(pointIndex, point);
    int xPixelCoord = std::floor((point[0] - boundingBox[0]) * scaleX);
    int yPixelCoord = std::floor((point[1] - boundingBox[2]) * scaleY);
    perPixelDistribution[xPixelCoord + this->Dimensions[0] * yPixelCoord].push_back(point[2]);
  }

  // fill the image
  for (int x = 0; x < this->Dimensions[0]; ++x)
  {
    for (int y = 0; y < this->Dimensions[1]; ++y)
    {
      // if the pixel is empty, skip it
      if (perPixelDistribution[x + this->Dimensions[0] * y].size() == 0)
      {
        continue;
      }

      // sort the heights values
      std::sort(perPixelDistribution[x + this->Dimensions[0] * y].begin(),
                perPixelDistribution[x + this->Dimensions[0] * y].end());

      int rankIndex = std::floor((perPixelDistribution[x + this->Dimensions[0] * y].size() - 1) * this->RankPercentil);
      double rankValue = perPixelDistribution[x + this->Dimensions[0] * y][rankIndex];
      double value = rankValue - boundingBox[4];
      image->SetScalarComponentFromDouble(x, y, 0, 0, value);
    }
  }

  vtkImageData* outputImage = vtkImageData::GetData(outputVector->GetInformationObject(0));
  outputImage->ShallowCopy(image);
  return 1;
}

//-----------------------------------------------------------------------------
void vtkPointCloudLinearProjector::SetPlaneNormal(double w0, double w1, double w2)
{
  // Here we will construct a new base of R3 using
  // the normal of the plane as the Z-axis. To proceed,
  // we will compute the rotation that map ez toward n
  // and its axis being cross(ez, n)
  Eigen::Vector3d ez(0, 0, 1);
  Eigen::Vector3d n(w0, w1, w2);

  // check that the plane normal is not the null pointer
  if (n.norm() < std::numeric_limits<float>::epsilon())
  {
    vtkGenericWarningMacro("The plane normal should not be the null vector");
    return;
  }
  n.normalize();

  Eigen::Vector3d u = ez.cross(n);
  // it means that n and ez are colinear
  if (u.norm() < std::numeric_limits<float>::epsilon())
  {
    return;
  }
  u.normalize();

  double angle = SignedAngle(ez, n);
  Eigen::Matrix3d R(Eigen::AngleAxisd(angle, u));
  this->ChangeOfBasis = R;
  this->Projector = this->DiagonalizedProjector * this->ChangeOfBasis.transpose();
}
