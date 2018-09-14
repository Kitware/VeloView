#include "vtkLidarSource.h"

#include "vtkLidarSourceInternal.h"
#include <vtkTransform.h>

vtkStandardNewMacro(vtkLidarSource);


//-----------------------------------------------------------------------------
vtkLidarSource::vtkLidarSource() /*: vtkLidarSource(new vtkLidarSourceInternal)*/
{

}

//-----------------------------------------------------------------------------
vtkLidarSource::vtkLidarSource(vtkLidarSourceInternal* internal)
{
  this->Internal = internal;
}

//-----------------------------------------------------------------------------
vtkLidarSource::~vtkLidarSource()
{
  // TODO handle the deletion of vtkIn
  //delete this->Internal;
}

//-----------------------------------------------------------------------------
int vtkLidarSource::GetNumberOfChannels()
{
  return this->Internal->CalibrationReportedNumLasers;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkLidarSource::GetFrame(int frameNumber, int wantedNumberOfTrailingFrames)
{
  return this->Internal->GetFrame(frameNumber, wantedNumberOfTrailingFrames);
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetLaserSelection(bool laserSelection[])
{
  for (int i = 0; i < this->Internal->CalibrationReportedNumLasers; ++i)
  {
    this->Internal->LaserSelection[i] = laserSelection[i];
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarSource::GetLaserSelection(bool laserSelection[])
{
  for (int i = 0; i < this->Internal->CalibrationReportedNumLasers; ++i)
  {
    laserSelection[i] = this->Internal->LaserSelection[i];
  }
}

//-----------------------------------------------------------------------------
void vtkLidarSource::PrintSelf( ostream& os, vtkIndent indent )
{
  this->Superclass::PrintSelf( os, indent );
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetNumberOfTrailingFrames(int numTrailing)
{
  assert(numTrailing >= 0);
  this->Internal->NumberOfTrailingFrames = numTrailing;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarSource::ProcessPacket(unsigned char *data, unsigned int bytesReceived)
{
  this->Internal->ProcessPacket(data, bytesReceived);
}

//-----------------------------------------------------------------------------
double vtkLidarSource::GetCurrentRpm()
{
  return this->Internal->currentRpm;
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetCropReturns(int crop)
{
  if (!this->Internal->CropReturns == !!crop)
  {
    this->Internal->CropReturns = !!crop;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetCropOutside(int crop)
{
  if (!this->Internal->CropOutside == !!crop)
  {
    this->Internal->CropOutside = !!crop;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetCropRegion(double region[6])
{
  std::copy(region, region + 6, this->Internal->CropRegion);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetCropMode(int cropMode)
{
  this->Internal->CropMode = static_cast<vtkLidarSourceInternal::CropModeEnum>(cropMode);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetCropRegion(
  double xl, double xu, double yl, double yu, double zl, double zu)
{
  this->Internal->CropRegion[0] = xl;
  this->Internal->CropRegion[1] = xu;
  this->Internal->CropRegion[2] = yl;
  this->Internal->CropRegion[3] = yu;
  this->Internal->CropRegion[4] = zl;
  this->Internal->CropRegion[5] = zu;
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarSource::GetIgnoreZeroDistances() const
{
  return this->Internal->IgnoreZeroDistances;
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetIgnoreZeroDistances(int value)
{
  if (this->Internal->IgnoreZeroDistances != value)
  {
    this->Internal->IgnoreZeroDistances = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkLidarSource::GetIgnoreEmptyFrames() const
{
  return this->Internal->IgnoreEmptyFrames;
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetIgnoreEmptyFrames(int value)
{
  if (this->Internal->IgnoreEmptyFrames != value)
  {
    this->Internal->IgnoreEmptyFrames = value;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
bool vtkLidarSource::getCorrectionsInitialized()
{
  return this->Internal->CorrectionsInitialized;
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetSensorTransform(vtkTransform * transform)
{
  if (transform)
  {
    this->Internal->SensorTransform->SetMatrix(transform->GetMatrix());
  }
  else
  {
    this->Internal->SensorTransform->Identity();
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkLidarSource::GetApplyTransform()
{
  return this->Internal->ApplyTransform;
}

//-----------------------------------------------------------------------------
void vtkLidarSource::SetApplyTransform(int apply)
{
  if (apply != this->Internal->ApplyTransform)
  {
    this->Modified();
  }
  this->Internal->ApplyTransform = apply;
}

//-----------------------------------------------------------------------------
int vtkLidarSource::GetNumberOfFrames()
{
  return this->Internal->FilePositions.size();
}

//-----------------------------------------------------------------------------
int vtkLidarSource::RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
   return 1;
}
