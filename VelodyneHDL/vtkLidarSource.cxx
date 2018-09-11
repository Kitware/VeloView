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
int vtkLidarSource::RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
   return 1;
}
