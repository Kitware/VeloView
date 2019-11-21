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
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

// VTK
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>

// BOOST
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Dense>

// Implementation of the New function
vtkStandardNewMacro(vtkPointCloudLinearProjector)

//------------------------------------------------------------------------------
int vtkPointCloudLinearProjector::FillInputPortInformation(int port, vtkInformation* info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
int vtkPointCloudLinearProjector::RequestInformation(vtkInformation *vtkNotUsed(request),
                                                vtkInformationVector **vtkNotUsed(inputVector),
                                                vtkInformationVector *outputVector)
{
  if (this->Resolution[0] <= 0 || this->Resolution[1] <= 0)
  {
    vtkWarningMacro("Resolution must be positive, not " << this->Resolution[0] << "x" << this->Resolution[1] << ".")
    return VTK_ERROR;
  }

  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(),
               0, this->Resolution[0] - 1,
               0, this->Resolution[1] - 1,
               0, 0);

  outInfo->Set(vtkDataObject::ORIGIN(), this->Origin, 3);
  outInfo->Set(vtkDataObject::SPACING(), this->Spacing, 3);

  if (this->ExportAsChar)
  {
    vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_UNSIGNED_CHAR, 1);
  }
  else
  {
    vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_DOUBLE, 1);
  }
  return VTK_OK;
}

//------------------------------------------------------------------------------
int vtkPointCloudLinearProjector::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  // Get the input
  vtkPolyData* input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  double point[3];
  Eigen::Vector3d X;
  vtkDataArray* values = this->GetInputArrayToProcess(0, inputVector);

  // Express the data in an other reference frame to align the new
  // Z-axis with a settled direction (typically, the gravity acceleration
  // vector)
  auto transformedPoints = vtkSmartPointer<vtkPoints>::New();
  transformedPoints->SetNumberOfPoints(input->GetNumberOfPoints());
  // Transform the input polydata
  for (vtkIdType pointIndex = 0; pointIndex < input->GetNumberOfPoints(); ++pointIndex)
  {
    input->GetPoint(pointIndex, point);
    X << point[0], point[1], point[2];
    X = this->Projector * X;
    transformedPoints->SetPoint(pointIndex, X.data());
  }

  // Get the point cloud bounding box parameters
  double boundingBox[6];
  transformedPoints->Modified();
  transformedPoints->GetBounds(boundingBox);

  double pointRangeX = boundingBox[1] - boundingBox[0];
  double pointRangeY = boundingBox[3] - boundingBox[2];

  if (pointRangeX <= 0 || pointRangeY <= 0)
  {
    return VTK_ERROR;
  }

  this->Origin[0] = boundingBox[0];
  this->Origin[1] = boundingBox[2];

  this->Spacing[0] = pointRangeX / static_cast<double>(this->Resolution[0]);
  this->Spacing[1] = pointRangeY / static_cast<double>(this->Resolution[1]);

  double scaleX = (this->Resolution[0] - 1) / (pointRangeX);
  double scaleY = (this->Resolution[1] - 1) / (pointRangeY);

  // Get the output image and fill with zeros
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
  image->SetSpacing(this->Spacing);
  // Is it not possible to orient a vtkImageData within a vtkImageAlgorithm
  // filter in ParaView 5.4, which LidarView is currently based on. There are
  // commits to add a "DirectionMatrix" to vtkImageData (commit
  // 55c0d1cbdf1a152eec74d538af9fa4e17617c379) but this is from April 2019.
  //
  // TODO
  // Use DirectionMatrix when it becomes available to align the projection to
  // the point cloud along the projection axis.
  image->SetOrigin(this->Origin);
  image->SetDimensions(this->Resolution[0], this->Resolution[1], 1);
  if (this->ExportAsChar)
  {
    image->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
  }
  else
  {
    image->AllocateScalars(VTK_DOUBLE, 1);
  }
  // Compute distribution about the height of the data lying in a pixel
  std::vector<std::vector<double> > perPixelDistribution(this->Resolution[0] * this->Resolution[1]);
  for (vtkIdType pointIndex = 0; pointIndex < input->GetNumberOfPoints(); ++pointIndex)
  {
    transformedPoints->GetPoint(pointIndex, point);
    int xPixelCoord = std::floor((point[0] - boundingBox[0]) * scaleX);
    int yPixelCoord = std::floor((point[1] - boundingBox[2]) * scaleY);
    perPixelDistribution[xPixelCoord + this->Resolution[0] * yPixelCoord].push_back(
      this->HeightMap ? point[2] : values->GetTuple1(pointIndex)
    );
  }

  // fill the image
  double valueRange[2];
  if (this->HeightMap)
  {
    valueRange[0] = boundingBox[4];
    valueRange[1] = boundingBox[5];
  }
  else
  {
    values->GetRange(valueRange);
  }

  double valueShift = (this->ExportAsChar || this->ShiftToZero) ? valueRange[0] : 0.0;
  double valueScale = valueRange[1] - valueRange[0];
  for (unsigned int y = 0; y < this->Resolution[1]; ++y)
  {
    for (unsigned int x = 0; x < this->Resolution[0]; ++x)
    {
      unsigned int imageIndex = this->Resolution[0] * y + x;
      double value = 0.0;
      // if the pixel is empty, skip it
      if (perPixelDistribution[imageIndex].size() > 0)
      {
        std::sort(perPixelDistribution[imageIndex].begin(), perPixelDistribution[imageIndex].end());
        unsigned int rankIndex = std::floor((perPixelDistribution[imageIndex].size() - 1) * this->RankPercentile);
        value = perPixelDistribution[imageIndex][rankIndex];
      }
      if (this->ExportAsChar)
      {
        value -= valueShift;
        value = std::round((value / valueScale) * 0xff);
      }
      else if (this->ShiftToZero)
      {
        value -= valueShift;
      }
      // Use unscaled values to match input values and ranges.
      image->SetScalarComponentFromDouble(x, y, 0, 0, value);
    }
  }

  int neigh = this->MedianFilterWidth;
  if (this->ShouldMedianFilter)
  {
    vtkSmartPointer<vtkImageData> tempImage = vtkSmartPointer<vtkImageData>::New();
    tempImage->DeepCopy(image);
    int dimX = static_cast<int>(this->Resolution[0]);
    int dimY = static_cast<int>(this->Resolution[1]);
    for (int x = 0; x < dimX; ++x)
    {
      for (int y = 0; y < dimY; ++y)
      {
        if (tempImage->GetScalarComponentAsDouble(x, y, 0, 0) == 0)
        {
          continue;
        }
        std::vector<double> neighborhoodValues;
        unsigned int minU = std::max(0, x - neigh);
        unsigned int maxU = std::min(dimX - 1, x + neigh);
        unsigned int minV = std::max(0, y - neigh);
        unsigned int maxV = std::min(dimY - 1, y + neigh);

        for (unsigned int u = minU; u <= maxU; ++u)
        {
          for (unsigned int v = minV; v <= maxV; ++v)
          {
            neighborhoodValues.push_back(tempImage->GetScalarComponentAsDouble(u, v, 0, 0));
          }
        }
        std::sort(neighborhoodValues.begin(), neighborhoodValues.end());
        double medianValue = neighborhoodValues[neighborhoodValues.size() / 2];
        image->SetScalarComponentFromDouble(x, y, 0, 0, medianValue);
      }
    }
  }

  vtkImageData* outputImage = vtkImageData::GetData(outputVector->GetInformationObject(0));
  outputImage->ShallowCopy(image);
  // Use the input name to share the color scale.
  if (this->HeightMap)
  {
    outputImage->GetPointData()->GetScalars()->SetName("DistanceToPlane");
  }
  else
  {
    outputImage->GetPointData()->GetScalars()->SetName(values->GetName());
  }
  this->Modified();
  return VTK_OK;
}

//------------------------------------------------------------------------------
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
    vtkGenericWarningMacro("The plane normal cannot be the null vector");
    return;
  }
  n.normalize();

  this->Modified();

  double cosAngle = ez.dot(n);
  Eigen::Vector3d rawAxis = ez.cross(n);
  double rawAxisMagnitude = rawAxis.norm();
  // Check if ez and n are colinear.
  if (rawAxisMagnitude < std::numeric_limits<float>::epsilon())
  {
    // Rotate 180 deg around the x axis if the projection is along negative z.
    if (cosAngle < 0)
    {
      this->ChangeOfBasis << 1,0,0,  0,-1,0,  0,0,-1;
    }
    // Reset it to the identity matrix otherwise.
    else
    {
      this->ChangeOfBasis = Eigen::Matrix3d::Identity();
    }
  }
  else
  {
    Eigen::Vector3d unitAxis = rawAxis / rawAxisMagnitude;
    double sinAngle = rawAxis.dot(unitAxis);
    double angle = std::atan2(sinAngle, cosAngle);
    Eigen::Matrix3d R(Eigen::AngleAxisd(angle, unitAxis));
    this->ChangeOfBasis = R;
  }
  this->Projector = this->DiagonalizedProjector * this->ChangeOfBasis.inverse();
}
