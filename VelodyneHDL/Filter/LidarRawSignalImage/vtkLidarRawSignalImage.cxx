/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLidarRawSignalImage.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// LOCAL
#include "vtkLidarRawSignalImage.h"

#include <vtkObjectFactory.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTable.h>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLidarRawSignalImage)

//----------------------------------------------------------------------------
vtkLidarRawSignalImage::vtkLidarRawSignalImage()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
void vtkLidarRawSignalImage::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  vtkIndent paramIndent = indent.GetNextIndent();
  os << paramIndent << "Image Parameter: " << std::endl;
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->param << std::endl;
  PrintParameter(Width)
  PrintParameter(Spacing[0])
  PrintParameter(Spacing[1])
  PrintParameter(Spacing[2])
  PrintParameter(Origin[0])
  PrintParameter(Origin[1])
  PrintParameter(Origin[2])

}

//-----------------------------------------------------------------------------
int vtkLidarRawSignalImage::FillInputPortInformation(int port, vtkInformation *info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    return 1;
  }
  if (port == 1)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTable" );
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
int vtkLidarRawSignalImage::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get the inputs
  vtkPolyData * input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));
  vtkTable* calibration = vtkTable::GetData(inputVector[1]->GetInformationObject(0));

  // Initialize the filter using the provided sensor calibration table
  if (this->VerticallySortedIndex.size() == 0)
  {
    if (!this->InitializationFromCalibration(calibration))
    {
      return VTK_ERROR;
    }
  }

  // Set the spacing according to the angles FoV.
  // The idea is to have a linear mapping between
  // the angles and the pixels size so that 360 degrees
  // correspond to 100 meters
  // TODO: take into account the fact that the vertical angular
  // resolution is not constant to avoid distortion
  this->Spacing[0] = this->Scale * (this->HorizontalFOV * 100.0) / (360.0 * this->Width);
  this->Spacing[1] = this->Scale * (this->VerticalFOV * 100.0) / (360.0 * this->Height);

  // Get the output image and fill with zeros
  vtkImageData* outputImage = vtkImageData::GetData(outputVector->GetInformationObject(0));
  outputImage->SetDimensions(this->Width, this->Height, 1);
  outputImage->SetSpacing(this->Spacing);
  outputImage->SetOrigin(this->Origin);
  outputImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
  unsigned char* dataPointer = static_cast<unsigned char*>(outputImage->GetScalarPointer());
  std::fill(dataPointer, dataPointer + this->Height * this->Width, 0);

  // Get the required array
  vtkDataArray* arrayToUse = this->GetInputArrayToProcess(0, inputVector);
  if (!arrayToUse)
  {
    vtkErrorMacro("No input array is selected!");
    return 0;
  }
  vtkDataArray* azimuth = input->GetPointData()->GetArray("azimuth");
  vtkDataArray* laserIndex = input->GetPointData()->GetArray("laser_id");
  if (!azimuth || !laserIndex)
  {
    vtkErrorMacro("The input polydata must contain azimuth, laser idx data");
    return 0;
  }

  // Project each point one by one into the image
  for (unsigned int indexPoint = 0; indexPoint < input->GetNumberOfPoints(); ++indexPoint)
  {
    // Get the coordinate of the input point
    double currPoint[3];
    input->GetPoint(indexPoint, currPoint);

    // compute w coordinate of the image based
    // on the azimuth angle
    double azimuthAngle = azimuth->GetTuple1(indexPoint);
    int w = std::floor(azimuthAngle / 36000.0 * this->Width);

    // compute the h coordinate of the image based
    // on the vertically sorted laser index
    // /!\ the spherical grid is then based on the
    // laser index which means that the solid angle
    // represented by a "pixel" will depend on the angular
    // resolution between two consecutives laser. For example,
    // a VLS-128 has a non constant vertical angular resolution
    // resulting in an observed image distorded
    int idx = laserIndex->GetTuple1(indexPoint);
    int h = this->VerticallySortedIndex[idx];
    double value = arrayToUse->GetTuple1(indexPoint);
    outputImage->SetScalarComponentFromDouble(w, h, 0, 0, value);
  }

  return VTK_OK;
}

//-----------------------------------------------------------------------------
int vtkLidarRawSignalImage::RequestInformation(vtkInformation *vtkNotUsed(request),
                                               vtkInformationVector **vtkNotUsed(inputVector),
                                               vtkInformationVector *outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(),
               0, this->Width-1,
               0, this->Height-1,
               0, 0);

  outInfo->Set(vtkDataObject::ORIGIN(),this->Origin,3);
  outInfo->Set(vtkDataObject::SPACING(),this->Spacing,3);

  vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_UNSIGNED_CHAR, 1);
  return VTK_OK;
}

//-----------------------------------------------------------------------------
bool vtkLidarRawSignalImage::InitializationFromCalibration(vtkTable* calibration)
{
  this->VerticallySortedIndex.clear();
  // check that the calibration is provided
  if (!calibration)
  {
    vtkErrorMacro("Calibration is not provided, algorithm can not be initialized");
    return false;
  }

  // The height of the image corresponds to the number of lasers
  this->Height = static_cast<int>(calibration->GetNumberOfRows());

  // Get the vertical angle corrections
  auto verticalCorrectionArray = vtkDataArray::SafeDownCast(
        calibration->GetColumnByName("verticalCorrection"));
  if (!verticalCorrectionArray)
  {
    vtkErrorMacro("Calibration does not provide the vertical angles correction");
    return false;
  }

  // Fill the index mapping
  this->VerticallySortedIndex.resize(this->Height);
  std::vector<std::pair<double, int> > toSortIndex(this->Height);
  for (int idx = 0; idx < this->Height; ++idx)
  {
    toSortIndex[idx] = std::pair<double, int>(verticalCorrectionArray->GetTuple1(idx), idx);
  }
  std::sort(toSortIndex.begin(), toSortIndex.end());
  for (unsigned int idx = 0; idx < toSortIndex.size(); ++idx)
  {
    this->VerticallySortedIndex[toSortIndex[idx].second] = idx;
  }

  // Set the spacing according to the vertical FoV
  // the idea is to have a linear mapping between
  // the angles and the pixels size so that 360 degree
  // correspond to 100 meters
  this->VerticalFOV = std::abs(toSortIndex.front().first - toSortIndex.back().first);

  return true;
}
