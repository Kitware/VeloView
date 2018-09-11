#include "vtkLidarSource.h"

#include "vtkLidarSourceInternal.h"

vtkStandardNewMacro(vtkLidarSource);


//-----------------------------------------------------------------------------
vtkLidarSource::vtkLidarSource() : vtkLidarSource(new vtkLidarSourceInternal)
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
  delete this->Internal;
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
int vtkLidarSource::RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
   return 1;
}
