#include "vtkLidarProviderInternal.h"

#include <boost/filesystem.hpp>
#include <sstream>

#include "vtkLidarProvider.h"
#include "vtkVelodyneTransformInterpolator.h"


vtkLidarProviderInternal::vtkLidarProviderInternal(vtkLidarProvider* obj)
{
  this->Lidar = obj;

  this->CropMode = Cartesian;

  this->NumberOfTrailingFrames = 0;
  this->CalibrationReportedNumLasers = -1;
  this->CalibrationFileName = "";
  this->IsCalibrated = false;

  this->ApplyTransform = false;

  // Cropping
  this->CropReturns = false;
  this->CropOutside = false;
  this->CropRegion[0] = this->CropRegion[1] = 0.0;
  this->CropRegion[2] = this->CropRegion[3] = 0.0;
  this->CropRegion[4] = this->CropRegion[5] = 0.0;

  this->currentRpm;

  this->IgnoreZeroDistances = true;
  this->IgnoreEmptyFrames = true;
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetNumberOfTrailingFrames(const int numberTrailing)
{
  assert(numberTrailing >= 0);
  this->NumberOfTrailingFrames = numberTrailing;
  this->Lidar->Modified();
}

//-----------------------------------------------------------------------------
std::string vtkLidarProviderInternal::GetCalibrationFileName()
{
  return this->CalibrationFileName;
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetCalibrationFileName(const std::string &filename)
{
  if (filename == this->CalibrationFileName)
  {
    return;
  }

  if (!boost::filesystem::exists(filename) ||
    boost::filesystem::is_directory(filename))
  {
    std::ostringstream errorMessage("Invalid sensor configuration file ");
    errorMessage << filename << ": ";
    if (!boost::filesystem::exists(filename))
    {
      errorMessage << "File not found!";
    }
    else
    {
      errorMessage << "It is a directory!";
    }
//    vtkErrorMacro(<< errorMessage.str());
    return;
  }

  this->LoadCalibration(filename);
  this->CalibrationFileName = filename;
  this->IsCalibrated = true;
}

//-----------------------------------------------------------------------------
double vtkLidarProviderInternal::GetCurrentRpm()
{
  return this->currentRpm;
}

//-----------------------------------------------------------------------------
bool vtkLidarProviderInternal::GetIsCalibrated()
{
  return this->IsCalibrated;
}

double vtkLidarProviderInternal::GetDistanceResolutionM()
{
  return this->distanceResolutionM;
}

//-----------------------------------------------------------------------------
int vtkLidarProviderInternal::GetIgnoreZeroDistances() const
{
  return this->IgnoreZeroDistances;
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetIgnoreZeroDistances(const bool value)
{
  if (this->IgnoreZeroDistances != value)
  {
    this->IgnoreZeroDistances = value;
    this->Lidar->Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkLidarProviderInternal::GetIgnoreEmptyFrames() const
{
  return this->IgnoreEmptyFrames;
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetIgnoreEmptyFrames(const bool value)
{
  if (this->IgnoreEmptyFrames != value)
  {
    this->IgnoreEmptyFrames = value;
    this->Lidar->Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkLidarProviderInternal::GetNumberOfChannels()
{
  return this->CalibrationReportedNumLasers;
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetLaserSelection(bool laserSelection[])
{
  for (int i = 0; i < this->CalibrationReportedNumLasers; ++i)
  {
    this->LaserSelection[i] = laserSelection[i];
  }
  this->Lidar->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::GetLaserSelection(bool laserSelection[])
{
  for (int i = 0; i < this->CalibrationReportedNumLasers; ++i)
  {
    laserSelection[i] = this->LaserSelection[i];
  }
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetCropMode(const int mode)
{
  this->CropMode = static_cast<vtkLidarProviderInternal::CropModeEnum>(mode);
  this->Lidar->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetCropReturns(const bool value)
{
  if (!this->CropReturns == value)
  {
    this->CropReturns = value;
    this->Lidar->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetCropOutside(const bool value)
{
  if (!this->CropOutside == value)
  {
    this->CropOutside = value;
    this->Lidar->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetCropRegion(double region[])
{
  std::copy(region, region + 6, this->CropRegion);
  this->Lidar->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetCropRegion(const double v0, const double v1, const double v2, const double v3, const double v4, const double v5)
{
  this->CropRegion[0] = v0;
  this->CropRegion[1] = v1;
  this->CropRegion[2] = v2;
  this->CropRegion[3] = v3;
  this->CropRegion[4] = v4;
  this->CropRegion[5] = v5;
  this->Lidar->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetSensorTransform(vtkTransform *t)
{
  if (t)
  {
    this->SensorTransform->SetMatrix(t->GetMatrix());
  }
  else
  {
    this->SensorTransform->Identity();
  }
  this->Lidar->Modified();
}

//-----------------------------------------------------------------------------
bool vtkLidarProviderInternal::GetApplyTransform()
{
  return this->ApplyTransform;
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetApplyTransform(const bool apply)
{
  if (apply != this->ApplyTransform)
  {
    this->Lidar->Modified();
  }
  this->ApplyTransform = apply;
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator *vtkLidarProviderInternal::GetInterpolator() const
{
  return this->Interp;
}

//-----------------------------------------------------------------------------
void vtkLidarProviderInternal::SetInterpolator(vtkVelodyneTransformInterpolator *interpolator)
{
  this->Interp = interpolator;
  this->Lidar->Modified();
}


