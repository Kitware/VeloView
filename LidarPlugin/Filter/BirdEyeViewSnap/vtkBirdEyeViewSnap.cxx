/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkBirdEyeViewSnap.cxx
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// LOCAL
#include "vtkBirdEyeViewSnap.h"

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

// VTK
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTransform.h>
#include <vtkTupleInterpolator.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>
#include <vtkPNGWriter.h>

// BOOST
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Dense>

// Implementation of the New function
vtkStandardNewMacro(vtkBirdEyeViewSnap)

//----------------------------------------------------------------------------
vtkBirdEyeViewSnap::vtkBirdEyeViewSnap()
{
  this->pixelResX = 0.20; // 20cm
  this->pixelResY = 0.20; // 20cm
  this->Count = 0;
  this->RadicalFileName = "NoRadical";
  this->ExtensionFileName = "NoExtension";

  // identity => projection removing z-index
  this->Orientation << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;
}

//----------------------------------------------------------------------------
vtkBirdEyeViewSnap::~vtkBirdEyeViewSnap()
{
}

//-----------------------------------------------------------------------------
void vtkBirdEyeViewSnap::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
int vtkBirdEyeViewSnap::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get the input
  vtkPolyData * input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  // Get the output
  vtkPolyData *output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(input);

  // Check that the provided filename is valid
  if (this->RadicalFileName == "NoRadical" ||
      this->ExtensionFileName == "NoExtension")
  {
    vtkGenericWarningMacro("Filename has not been settled or is invalid");
    return 0;
  }

  // transform the input
  Eigen::Matrix<double, 3, 1> point;
  double vtkpoint[3];
  for (unsigned int k = 0; k < input->GetNumberOfPoints(); ++k)
  {
    input->GetPoint(k, vtkpoint);
    point << vtkpoint[0], vtkpoint[1], vtkpoint[2];
    point = this->Orientation * point;
    vtkpoint[0] = point(0);
    vtkpoint[1] = point(1);
    vtkpoint[2] = point(2);
    output->GetPoints()->SetPoint(k, vtkpoint);
  }

  // Create the bird eye view image
  double bounds[6];
  output->GetBounds(bounds);
  unsigned int x, y;

  unsigned int H = std::ceil((bounds[1] - bounds[0]) / this->pixelResX);
  unsigned int W = std::ceil((bounds[3] - bounds[2]) / this->pixelResY);

  // Image
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
  image->SetDimensions(H, W, 1);
  image->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
  unsigned char* dataPointer = static_cast<unsigned char*>(image->GetScalarPointer());
  std::fill(dataPointer, dataPointer + H * W, 0);

  // Get the reflectivity array
  vtkDataArray* intensity = output->GetPointData()->GetArray("intensity");

  for (unsigned int k = 0; k < output->GetNumberOfPoints(); ++k)
  {
    // Compute pixel coordinate
    output->GetPoint(k, vtkpoint);
    x = std::floor((vtkpoint[0] - bounds[0]) / (bounds[1] - bounds[0]) * (H - 1));
    y = std::floor((vtkpoint[1] - bounds[2]) / (bounds[3] - bounds[2]) * (W - 1));

    // set value
    unsigned char value = intensity->GetVariantValue(k).ToUnsignedChar();
    image->SetScalarComponentFromDouble(x, y, 0, 0, value);
  }

  // Save the image
  std::stringstream ss;
  ss << this->RadicalFileName << this->Count << "." << ExtensionFileName;
  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(ss.str().c_str());
  writer->SetInputData(image);
  writer->Write();

  this->Count++;

  return 1;
}

//-----------------------------------------------------------------------------
void vtkBirdEyeViewSnap::SetPlaneParam(double params[4])
{
  // Here we will construct a new base of R3 using
  // the normal of the plane as the Z-axis. The new
  // x axis will be the projection of the older x-axis
  // on the plane and the new y-axis will be completed
  // so that we obtain a orthonormal direct base
  Eigen::Matrix<double, 3, 1> ex, ey, ez, ex2, ey2, ez2;
  ex << 1, 0, 0;
  ey << 0, 1, 0;
  ez << 0, 0, 1;

  ez2 << params[0], params[1], params[2];
  ez2.normalize();
  ez2 = ez.dot(ez2) / std::abs(ez.dot(ez2)) * ez2;

  ex2 = ex - ex.dot(ez2) * ez2;
  ex2.normalize();

  ey2 = ez2.cross(ex2);
  ey2.normalize();

  this->Orientation.row(0) = ex2;
  this->Orientation.row(1) = ey2;
  this->Orientation.row(2) = ez2;
}

//-----------------------------------------------------------------------------
void vtkBirdEyeViewSnap::SetFolderName(std::string filename)
{
  std::vector<std::string> strs;
  boost::split(strs, filename, boost::is_any_of("."));

  // Check that there is an extension
  if (strs.size() <= 1)
  {
    vtkGenericWarningMacro("filename is not valid: " << filename);
    return;
  }

  // Check that the extension is supported
  std::string extension = strs[strs.size() - 1];
  if ((extension != "png") && (extension != "jpg"))
  {
    vtkGenericWarningMacro("file format not supported: " << extension
                           << " supported file format are: jpg, png");
    return;
  }

  std::stringstream ss;
  for (unsigned int k = 0; k < strs.size() - 1; ++k)
  {
    ss << strs[k];
  }

  this->ExtensionFileName = extension;
  this->RadicalFileName = ss.str();
}

//-----------------------------------------------------------------------------
void vtkBirdEyeViewSnap::SetResolution(double sX, double sY)
{
  this->pixelResX = sX;
  this->pixelResY = sY;
}

//-----------------------------------------------------------------------------
void vtkBirdEyeViewSnap::SetCount(unsigned int count)
{
  this->Count = count;
}