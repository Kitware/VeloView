#include "vtkLidarProvider.h"
#include "vtkLidarProviderInternal.h"

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetNumberOfTrailingFrames(const int numberTrailing)
{
  return this->Internal->SetNumberOfTrailingFrames(numberTrailing);
}

//-----------------------------------------------------------------------------
std::string vtkLidarProvider::GetSensorInformation()
{
  return this->Internal->GetSensorInformation();
}

//-----------------------------------------------------------------------------
bool vtkLidarProvider::GetIsCalibrated()
{
  return this->Internal->GetIsCalibrated();
}

//-----------------------------------------------------------------------------
double vtkLidarProvider::GetCurrentRpm()
{
  return this->Internal->GetCurrentRpm();
}

//-----------------------------------------------------------------------------
double vtkLidarProvider::GetDistanceResolutionM()
{
  return this->Internal->GetDistanceResolutionM();
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetNumberOfFrames()
{
  return this->Internal->GetNumberOfFrames();
}

//-----------------------------------------------------------------------------
std::string vtkLidarProvider::GetCalibrationFileName()
{
  return this->Internal->GetCalibrationFileName();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCalibrationFileName(const std::string &filename)
{
  return this->Internal->SetCalibrationFileName(filename);
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetNumberOfChannels()
{
  return this->Internal->GetNumberOfChannels();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetLaserSelection(bool laserSelection[])
{
  return this->Internal->SetLaserSelection(laserSelection);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::GetLaserSelection(bool laserSelection[])
{
  return this->Internal->GetLaserSelection(laserSelection);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropMode(const int mode)
{
  return this->Internal->SetCropMode(mode);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropReturns(const bool value)
{
  return this->Internal->SetCropReturns(value);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropOutside(const bool value)
{
  return this->Internal->SetCropOutside(value);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropRegion(double region[6])
{
  return this->Internal->SetCropRegion(region);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropRegion(const double v0, const double v1, const double v2, const double v3, const double v4, const double v5)
{
  return this->Internal->SetCropRegion(v0, v1, v2, v3, v4, v5);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkLidarProvider::GetFrame(int frameNumber, int wantedNumberOfTrailingFrame)
{
  return this->Internal->GetFrame(frameNumber, wantedNumberOfTrailingFrame);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetSensorTransform(vtkTransform * t)
{
  return this->Internal->SetSensorTransform(t);
}

//-----------------------------------------------------------------------------
bool vtkLidarProvider::GetApplyTransform()
{
  return this->Internal->GetApplyTransform();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetApplyTransform(const bool apply)
{
  return this->Internal->SetApplyTransform(apply);
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator *vtkLidarProvider::GetInterpolator() const
{
  return this->Internal->GetInterpolator();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetInterpolator(vtkVelodyneTransformInterpolator *interpolator)
{
  return this->Internal->SetInterpolator(interpolator);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetDummyProperty(int)
{
  return this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetIgnoreZeroDistances() const
{
  return this->Internal->GetIgnoreZeroDistances();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetIgnoreZeroDistances(const bool value)
{
  return this->Internal->SetIgnoreZeroDistances(value);
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetIgnoreEmptyFrames() const
{
  return this->Internal->GetIgnoreEmptyFrames();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetIgnoreEmptyFrames(const bool value)
{
  return this->Internal->SetIgnoreEmptyFrames(value);
}

//-----------------------------------------------------------------------------
vtkLidarProvider::vtkLidarProvider()
{

}

//-----------------------------------------------------------------------------
vtkLidarProvider::~vtkLidarProvider()
{

}
