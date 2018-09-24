#include "vtkLidarProviderInternal.h"

#include "vtkLidarProvider.h"

vtkLidarProviderInternal::vtkLidarProviderInternal(vtkLidarProvider* obj)
{
  this->Lidar = obj;

  this->NumberOfTrailingFrames = 0;

  this->CalibrationFileName = "";
  this->CalibrationReportedNumLasers = -1;
  this->IsCalibrated = false;

  this->Frequency = 0;
  this->DistanceResolutionM = 0;

  this->IgnoreZeroDistances = true;
  this->IgnoreEmptyFrames = true;

  this->ApplyTransform = false;

  // Cropping
  this->CropMode = vtkLidarProvider::Cartesian;
  this->CropReturns = false;
  this->CropOutside = false;
  this->CropRegion[0] = 0;
  this->CropRegion[1] = 0;
  this->CropRegion[2] = 0;
  this->CropRegion[3] = 0;
  this->CropRegion[4] = 0;
  this->CropRegion[5] = 0;
}

