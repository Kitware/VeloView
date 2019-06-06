#include "vtkVelodyneMetaPacketInterpreter.h"

#include <vtkObjectFactory.h>

vtkStandardNewMacro(vtkVelodyneMetaPacketInterpreter)

//-----------------------------------------------------------------------------
vtkVelodyneMetaPacketInterpreter::vtkVelodyneMetaPacketInterpreter()
{
  this->PotentialInterps->AddItem(vtkVelodyneLegacyPacketInterpreter::New());
  this->PotentialInterps->AddItem(vtkVelodyneAdvancedPacketInterpreter::New());
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::LoadCalibration(const std::string &filename)
{
  if (this->SelectedInterp)
  {
    this->SelectedInterp->LoadCalibration(filename);
  }
  else
  {
    this->PotentialInterps->InitTraversal();
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetNextItemAsObject());
      pt->LoadCalibration(filename);
    }
  }
}

//-----------------------------------------------------------------------------
bool vtkVelodyneMetaPacketInterpreter::IsLidarPacket(const unsigned char *data, unsigned int dataLength)
{
  if (this->SelectedInterp)
  {
    return this->SelectedInterp->IsLidarPacket(data, dataLength);
  }
  else
  {
    this->PotentialInterps->InitTraversal();
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetNextItemAsObject());
      if (pt->IsLidarPacket(data, dataLength))
      {
        if (!this->SelectedInterp)
        {
          this->SelectedInterp = pt;
          return true;
        }
        else
        {
          vtkErrorMacro("Multiple Interpreter could interpret this packet! According to the specification, this is impossible!")
          return false;
        }
      }
    }
    return false;
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::ResetCurrentFrame()
{
  if (this->SelectedInterp != nullptr)
  {
    this->SelectedInterp->ResetCurrentFrame();
  }
  else
  {
    this->PotentialInterps->InitTraversal();
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetNextItemAsObject());
      pt->ResetCurrentFrame();
    }
  }
}

//-----------------------------------------------------------------------------
std::string vtkVelodyneMetaPacketInterpreter::GetSensorInformation()
{
  if (!this->SelectedInterp)
  {
    return "Could not determine the interpreter for now";
  }
  else
  {
    return this->SelectedInterp->GetSensorInformation();
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::ResetParserMetaData()
{
  if (this->SelectedInterp != nullptr)
  {
    this->SelectedInterp->ResetParserMetaData();
  }
  else
  {
    this->PotentialInterps->InitTraversal();
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetNextItemAsObject());
      pt->ResetParserMetaData();
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::SetCropRegion(double _arg1, double _arg2, double _arg3, double _arg4, double _arg5, double _arg6)
{
  if (this->SelectedInterp != nullptr)
  {
    this->SelectedInterp->SetCropRegion(_arg1, _arg2, _arg3, _arg4, _arg5, _arg6);
  }
  else
  {
    this->PotentialInterps->InitTraversal();
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetNextItemAsObject());
      pt->SetCropRegion(_arg1, _arg2, _arg3, _arg4, _arg5, _arg6);
    }
  }
}

//-----------------------------------------------------------------------------
vtkMTimeType vtkVelodyneMetaPacketInterpreter::GetMTime()
{
  this->PotentialInterps->InitTraversal();
  vtkMTimeType time = this->Superclass::GetMTime();
  for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
  {
    time = std::max(time, this->PotentialInterps->GetNextItemAsObject()->GetMTime());
  }
  return time;
}
