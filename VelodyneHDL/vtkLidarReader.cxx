#include "vtkLidarReader.h"

#include "vtkLidarReaderInternal.h"
#include <vtkTransform.h>
#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

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

//-----------------------------------------------------------------------------
int vtkLidarReader::RequestInformation(vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* info = outputVector->GetInformationObject(0);
  this->SetTimestepInformation(info);
  return 1;
}

//-----------------------------------------------------------------------------
void vtkLidarReader::SetTimestepInformation(vtkInformation* info)
{
  const size_t numberOfTimesteps = this->Internal->FilePositions.size();
  std::vector<double> timesteps;
  for (size_t i = 0; i < numberOfTimesteps; ++i)
  {
    timesteps.push_back(i);
  }

  if (numberOfTimesteps)
  {
    double timeRange[2] = { timesteps.front(), timesteps.back() };
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
  }
  else
  {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
  }
}
