#include "vtkLidarProvider.h"
#include "LidarPacketInterpreter.h"
#include "vtkVelodyneTransformInterpolator.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
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

}

//-----------------------------------------------------------------------------
vtkLidarProvider::~vtkLidarProvider()
{
  delete this->Interpreter;
}

void vtkLidarProvider::SetGpsTransform(vtkTransform* t)
{
  if (t)
  {
    this->Interpreter->GpsTransform->SetMatrix(t->GetMatrix());
  }
  else
  {
    this->Interpreter->GpsTransform->Identity();
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::AddTransform(double rx, double ry, double rz, double tx, double ty, double tz, double time)
{
  // All the result obtained was with ZXY but it should be ZYX
  // at the end, let's try with ZYX and make some test
  vtkNew<vtkTransform> mappingTransform;
  mappingTransform->PostMultiply();

  // Passage from L(t_current) to L(t_begin) first frame
  // Application of the SLAM result
  mappingTransform->RotateX(rx);
  mappingTransform->RotateY(ry);
  mappingTransform->RotateZ(rz);
  double pos[3] = {tx,ty,tz};
  mappingTransform->Translate(pos);
  this->Interpreter->Interp->AddTransform(time, mappingTransform.GetPointer());
  this->Interpreter->Interp->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::CreateLinearInterpolator()
{
  // Initialize the interpolator
  this->Interpreter->Interp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->Interpreter->Interp->SetInterpolationTypeToLinear();
  this->Interpreter->SetApplyTransform(false);
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::CreateNearestInterpolator()
{
  // Initialize the interpolator
  this->Interpreter->Interp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->Interpreter->Interp->SetInterpolationTypeToNearestLowBounded();
  this->Interpreter->SetApplyTransform(false);
}

