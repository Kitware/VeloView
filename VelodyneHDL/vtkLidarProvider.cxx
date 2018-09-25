#include "vtkLidarProvider.h"
#include "vtkLidarProviderInternal.h"
#include "vtkVelodyneTransformInterpolator.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/preprocessor.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>
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
void vtkLidarProvider::SetGpsTransform(vtkTransform* t)
{
  if (t)
  {
    this->Internal->GpsTransform->SetMatrix(t->GetMatrix());
  }
  else
  {
    this->Internal->GpsTransform->Identity();
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
  if (apply == this->Internal->ApplyTransform)
  {
    return;
  }
  this->Internal->ApplyTransform = apply;
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
  this->Internal->Interp->AddTransform(time, mappingTransform.GetPointer());
  this->Internal->Interp->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::LoadTransforms(const std::string& filename)
{
  std::ifstream file;
  file.open(filename);

  if (!file.is_open())
  {
    vtkGenericWarningMacro("Can't load the specified file");
  }

  // Create a new interpolator
  this->CreateNearestInterpolator();

  std::string line;
  std::string expectedLine = "Time,Rx(Roll),Ry(Pitch),Rz(Yaw),X,Y,Z";
  std::getline(file, line);
  if (line != expectedLine)
  {
    vtkGenericWarningMacro("Header file not expected. Version incompability");
  }

  while (std::getline(file, line))
  {
    std::vector<std::string> values;
    boost::split(values, line, boost::is_any_of(","));

    // time
    double t = std::atof(values[0].c_str());
    // rotation
    double rx = std::atof(values[1].c_str()) * 180.0 / vtkMath::Pi();
    double ry = std::atof(values[2].c_str()) * 180.0 / vtkMath::Pi();
    double rz = std::atof(values[3].c_str()) * 180.0 / vtkMath::Pi();
    // position
    double x = std::atof(values[4].c_str());
    double y = std::atof(values[5].c_str());
    double z = std::atof(values[6].c_str());
    // add the transform
    this->AddTransform(rx, ry, rz, x, y, z, t);
  }
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::ExportTransforms(const std::string& filename)
{
  std::ofstream file;
  file.open(filename);

  if (!file.is_open())
  {
    vtkGenericWarningMacro("Can't write the specified file");
  }

  std::vector<std::vector<double> > transforms = this->Internal->Interp->GetTransformList();
  std::vector<double> T;

  file.precision(12);
  file << "Time,Rx(Roll),Ry(Pitch),Rz(Yaw),X,Y,Z" << std::endl;
  for (unsigned int k = 0; k < transforms.size(); ++k)
  {
    T = transforms[k];
    file << T[0] << "," << T[1] << "," << T[2] << "," << T[3] << ","
         << T[4] << "," << T[5] << "," << T[6] << std::endl;
  }
  file.close();
  return;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::CreateLinearInterpolator()
{
  // Initialize the interpolator
  this->Internal->Interp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->Internal->Interp->SetInterpolationTypeToLinear();
  this->Internal->ApplyTransform = 0;
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::CreateNearestInterpolator()
{
  // Initialize the interpolator
  this->Internal->Interp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->Internal->Interp->SetInterpolationTypeToNearestLowBounded();
  this->Internal->ApplyTransform = 0;
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
