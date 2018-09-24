#include "vtkLidarProvider.h"
#include "vtkLidarProviderInternal.h"
#include "vtkVelodyneTransformInterpolator.h"

#include <boost/filesystem.hpp>
#include <sstream>

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetNumberOfTrailingFrames(const int numberTrailing)
{
  assert(numberTrailing >= 0);
  if (this->Internal->NumberOfTrailingFrames == numberTrailing)
  {
    return;
  }
  this->Internal->NumberOfTrailingFrames = numberTrailing;
  this->Modified();
}

//-----------------------------------------------------------------------------
bool vtkLidarProvider::GetIsCalibrated()
{
  return this->Internal->IsCalibrated;
}

//-----------------------------------------------------------------------------
double vtkLidarProvider::GetFrequency()
{
  return this->Internal->Frequency;
}

//-----------------------------------------------------------------------------
double vtkLidarProvider::GetDistanceResolutionM()
{
  return this->Internal->distanceResolutionM;
}

//-----------------------------------------------------------------------------
std::string vtkLidarProvider::GetCalibrationFileName()
{
  return this->Internal->CalibrationFileName;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCalibrationFileName(const std::string &filename)
{
  if (filename == this->Internal->CalibrationFileName)
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
    vtkErrorMacro(<< errorMessage.str());
    return;
  }

  this->Internal->LoadCalibration(filename);
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetNumberOfChannels()
{
  return this->Internal->CalibrationReportedNumLasers;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetLaserSelection(bool laserSelection[])
{
  for (int i = 0; i < this->Internal->CalibrationReportedNumLasers; ++i)
  {
    this->Internal->LaserSelection[i] = laserSelection[i];
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::GetLaserSelection(bool laserSelection[])
{
  for (int i = 0; i < this->Internal->CalibrationReportedNumLasers; ++i)
  {
    laserSelection[i] = this->Internal->LaserSelection[i];
  }
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropMode(const int mode)
{
  this->Internal->CropMode = static_cast<CropModeEnum>(mode);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropReturns(const bool value)
{
  if (!this->Internal->CropReturns == value)
  {
    this->Internal->CropReturns = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropOutside(const bool value)
{
  if (!this->Internal->CropOutside == value)
  {
    this->Internal->CropOutside = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropRegion(double region[6])
{
  std::copy(region, region + 6, this->Internal->CropRegion);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropRegion(const double v0, const double v1, const double v2, const double v3, const double v4, const double v5)
{
  this->Internal->CropRegion[0] = v0;
  this->Internal->CropRegion[1] = v1;
  this->Internal->CropRegion[2] = v2;
  this->Internal->CropRegion[3] = v3;
  this->Internal->CropRegion[4] = v4;
  this->Internal->CropRegion[5] = v5;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetSensorTransform(vtkTransform * t)
{
  if (t)
  {
    this->Internal->SensorTransform->SetMatrix(t->GetMatrix());
  }
  else
  {
    this->Internal->SensorTransform->Identity();
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
bool vtkLidarProvider::GetApplyTransform()
{
  return this->Internal->ApplyTransform;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetApplyTransform(const bool apply)
{
  if (apply != this->Internal->ApplyTransform)
  {
    return;
  }
  this->Internal->ApplyTransform = apply;
  this->Modified();
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator *vtkLidarProvider::GetInterpolator() const
{
  return this->Internal->Interp;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetInterpolator(vtkVelodyneTransformInterpolator *interpolator)
{
  this->Internal->Interp = interpolator;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetDummyProperty(int)
{
  return this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetIgnoreZeroDistances() const
{
  return this->Internal->IgnoreZeroDistances;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetIgnoreZeroDistances(const bool value)
{
  if (this->Internal->IgnoreZeroDistances != value)
  {
    this->Internal->IgnoreZeroDistances = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetIgnoreEmptyFrames() const
{
  return this->Internal->IgnoreEmptyFrames;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetIgnoreEmptyFrames(const bool value)
{
  if (this->Internal->IgnoreEmptyFrames != value)
  {
    this->Internal->IgnoreEmptyFrames = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
vtkLidarProvider::vtkLidarProvider()
{

}

//-----------------------------------------------------------------------------
vtkLidarProvider::~vtkLidarProvider()
{
  delete this->Internal;
}
