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
vtkSmartPointer<vtkPolyData> vtkVelodyneMetaPacketInterpreter::CreateNewEmptyFrame(
  vtkIdType vtkNotUsed(numberOfPoints), vtkIdType vtkNotUsed(prereservedNumberOfPoints))
{
  assert(false && "this function should never be called");
  return vtkSmartPointer<vtkPolyData>::New();
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
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
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
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
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
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
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
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
      pt->ResetParserMetaData();
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::SetLaserSelection(const bool *v)
{
  if (this->SelectedInterp != nullptr)
  {
    this->SelectedInterp->SetLaserSelection(v);
  }
  else
  {
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
      pt->SetLaserSelection(v);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::GetLaserSelection(bool *v)
{
  if (this->SelectedInterp != nullptr)
  {
    this->SelectedInterp->GetLaserSelection(v);
  }
  else
  {
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
      pt->GetLaserSelection(v);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::SetLaserSelection(const std::vector<bool> &v)
{
  if (this->SelectedInterp != nullptr)
  {
    this->SelectedInterp->SetLaserSelection(v);
  }
  else
  {
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
      pt->SetLaserSelection(v);
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
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i));
      pt->SetCropRegion(_arg1, _arg2, _arg3, _arg4, _arg5, _arg6);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::GetXMLColorTable(double XMLColorTable[])
{
  if (this->SelectedInterp)
  {
    CallWithDynamiCast(GetXMLColorTable(XMLColorTable), SelectedInterp)
  }
  else
  {
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      CallWithDynamiCast(GetXMLColorTable(XMLColorTable), PotentialInterps->GetItemAsObject(i))
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneMetaPacketInterpreter::GetLaserCorrections(double verticalCorrection[], double rotationalCorrection[], double distanceCorrection[], double distanceCorrectionX[], double distanceCorrectionY[], double verticalOffsetCorrection[], double horizontalOffsetCorrection[], double focalDistance[], double focalSlope[], double minIntensity[], double maxIntensity[])
{
  if (this->SelectedInterp)
  {
    CallWithDynamiCast(GetLaserCorrections(verticalCorrection, rotationalCorrection, distanceCorrection, distanceCorrectionX, distanceCorrectionY, verticalOffsetCorrection, horizontalOffsetCorrection, focalDistance, focalSlope, minIntensity, maxIntensity), SelectedInterp)
  }
  else
  {
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
    {
      CallWithDynamiCast(GetLaserCorrections(verticalCorrection, rotationalCorrection, distanceCorrection, distanceCorrectionX, distanceCorrectionY, verticalOffsetCorrection, horizontalOffsetCorrection, focalDistance, focalSlope, minIntensity, maxIntensity), PotentialInterps->GetItemAsObject(i))
    }
  }
}

//-----------------------------------------------------------------------------
vtkMTimeType vtkVelodyneMetaPacketInterpreter::GetMTime()
{
  vtkMTimeType time = this->Superclass::GetMTime();
  for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i)
  {
    time = std::max(time, this->PotentialInterps->GetItemAsObject(i)->GetMTime());
  }
  return time;
}
