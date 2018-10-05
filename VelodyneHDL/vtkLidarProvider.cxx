#include "vtkLidarProvider.h"
#include "LidarPacketInterpretor.h"
#include "vtkVelodyneTransformInterpolator.h"

#include <boost/filesystem.hpp>
#include <sstream>

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCalibrationFileName(const std::string &filename)
{
  if (filename == this->Interpretor->GetCalibrationFileName())
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

  this->Interpretor->LoadCalibration(filename);
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetNumberOfChannels()
{
  return this->Interpretor->GetCalibrationReportedNumLasers();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetLaserSelection(bool laserSelection[])
{
  this->Interpretor->SetLaserSelection(
        std::vector<bool>(laserSelection, laserSelection + this->Interpretor->GetCalibrationReportedNumLasers()));
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::GetLaserSelection(bool laserSelection[])
{
  // Bool vector is a particular data structure
  // you can't access to the data
  //this->Interpretor->GetLaserSelection().data();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropMode(const int mode)
{
  this->Interpretor->SetCropMode(/*static_cast<CropModeEnum>(*/mode/*)*/);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropRegion(double region[6])
{
  this->Interpretor->SetCropRegion(region);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropRegion(const double v0, const double v1, const double v2, const double v3, const double v4, const double v5)
{
  double region[6];
  region[0] = v0;
  region[1] = v1;
  region[2] = v2;
  region[3] = v3;
  region[4] = v4;
  region[5] = v5;
  this->SetCropRegion(region);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetSensorTransform(vtkTransform * t)
{
  if (t)
  {
    this->Interpretor->SensorTransform->SetMatrix(t->GetMatrix());
  }
  else
  {
    this->Interpretor->SensorTransform->Identity();
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator *vtkLidarProvider::GetInterpolator() const
{
  return this->Interpretor->Interp;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetInterpolator(vtkVelodyneTransformInterpolator *interpolator)
{
  this->Interpretor->Interp = interpolator;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetDummyProperty(int)
{
  return this->Modified();
}

//-----------------------------------------------------------------------------
vtkLidarProvider::vtkLidarProvider()
{

}

//-----------------------------------------------------------------------------
vtkLidarProvider::~vtkLidarProvider()
{
  delete this->Interpretor;
}
