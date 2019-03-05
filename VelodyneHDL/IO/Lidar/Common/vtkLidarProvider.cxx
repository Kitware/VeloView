#include "vtkLidarProvider.h"
#include "vtkLidarPacketInterpreter.h"
#include "vtkVelodyneTransformInterpolator.h"
#include "vtkVelodynePacketInterpreter.h"

#include <vtkInformation.h>

#include <boost/filesystem.hpp>
#include <sstream>

//-----------------------------------------------------------------------------
int vtkLidarProvider::FillOutputPortInformation(int port, vtkInformation* info)
{
  if ( port == 0 )
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    return 1;
  }
  if ( port == 1 )
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTable" );
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
std::string vtkLidarProvider::GetSensorInformation()
{
  return this->Interpreter->GetSensorInformation();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetCalibrationFileName(const std::string &filename)
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
    vtkErrorMacro(<< errorMessage.str());
    return;
  }
  this->CalibrationFileName = filename;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarProvider::SetDummyProperty(int)
{
  return this->Modified();
}

//-----------------------------------------------------------------------------
vtkMTimeType vtkLidarProvider::GetMTime()
{
  if (this->Interpreter)
  {
    return std::max(this->Superclass::GetMTime(), this->Interpreter->GetMTime());
  }
  return this->Superclass::GetMTime();
}

//-----------------------------------------------------------------------------
vtkLidarProvider::vtkLidarProvider()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(2);
}

//-----------------------------------------------------------------------------
int vtkLidarProvider::RequestInformation(vtkInformation *request,
                                         vtkInformationVector **inputVector,
                                         vtkInformationVector *outputVector)
{
  if (!this->Interpreter)
  {
    vtkErrorMacro(<< "Please select an Packet Interpreter" << endl);
  }

  // load the calibration file only now to allow to set it before the interpreter.
  if (this->Interpreter->GetCalibrationFileName() != this->CalibrationFileName)
  {
    this->Interpreter->LoadCalibration(this->CalibrationFileName);
  }
}

//-----------------------------------------------------------------------------
vtkCxxSetObjectMacro(vtkLidarProvider, Interpreter, vtkLidarPacketInterpreter)
