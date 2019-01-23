#include "vtkLidarProvider.h"
#include "LidarPacketInterpreter.h"
#include "vtkVelodyneTransformInterpolator.h"

#include <boost/filesystem.hpp>
#include <sstream>

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCalibrationFileName(const std::string &filename)
{
  if (filename == this->Interpreter->GetCalibrationFileName())
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

  this->Interpreter->LoadCalibration(filename);
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::GetNumberOfChannels()
{
  return this->Interpreter->GetCalibrationReportedNumLasers();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetLaserSelection(bool laserSelection[])
{
  this->Interpreter->SetLaserSelection(
        std::vector<bool>(laserSelection, laserSelection + this->Interpreter->GetCalibrationReportedNumLasers()));
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::GetLaserSelection(bool laserSelection[])
{
  // Bool vector is a particular data structure
  // you can't access to the data
  //this->Interpreter->GetLaserSelection().data();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropMode(const int mode)
{
  this->Interpreter->SetCropMode(/*static_cast<CropModeEnum>(*/mode/*)*/);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCropRegion(double region[6])
{
  this->Interpreter->SetCropRegion(region);
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
    this->Interpreter->SensorTransform->SetMatrix(t->GetMatrix());
  }
  else
  {
    this->Interpreter->SensorTransform->Identity();
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator *vtkLidarProvider::GetInterpolator() const
{
  return this->Interpreter->Interp;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetInterpolator(vtkVelodyneTransformInterpolator *interpolator)
{
  this->Interpreter->Interp = interpolator;
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
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
vtkLidarProvider::~vtkLidarProvider()
{
  if (this->Interpreter)
  {
    delete this->Interpreter;
  }
}
