#include "vtkLidarReader.h"

#include "vtkLidarReaderInternal.h"
#include <vtkTransform.h>

//vtkStandardNewMacro(vtkLidarReader);


//-----------------------------------------------------------------------------
vtkLidarReader::vtkLidarReader()
{

}

//-----------------------------------------------------------------------------
vtkLidarReader::vtkLidarReader(vtkLidarReaderInternal* internal)
{
  this->Internal = internal;
  this->SetPimpInternal(internal);
}

//-----------------------------------------------------------------------------
vtkLidarReader::~vtkLidarReader()
{
  // TODO handle the deletion of vtkIn
  //delete this->Internal;
}

void vtkLidarReader::SetPimpInternal(vtkLidarReaderInternal *internal)
{
  vtkLidarProvider::SetPimpInternal(internal);
  this->Internal = internal;
}

//-----------------------------------------------------------------------------
void vtkLidarReader::PrintSelf( ostream& os, vtkIndent indent )
{
  return this->Superclass::PrintSelf( os, indent );
}

//-----------------------------------------------------------------------------
std::string vtkLidarReader::GetFileName()
{
  return this->Internal->GetFileName();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SetFileName(const std::string &filename)
{
  return this->Internal->SetFileName(filename);
}

//-----------------------------------------------------------------------------
void vtkLidarReader::Open()
{
  return this->Internal->Open();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::Close()
{
  return this->Internal->Close();
}

//-----------------------------------------------------------------------------
void vtkLidarReader::ProcessPacket(unsigned char *data, unsigned int bytesReceived)
{
  return this->Internal->ProcessPacket(data, bytesReceived);
}

//-----------------------------------------------------------------------------
int vtkLidarReader::RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
   return 1;
}
