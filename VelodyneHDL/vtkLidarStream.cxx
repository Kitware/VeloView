#include "vtkLidarStream.h"

#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

#include "NetworkSource.h"
#include "PacketConsumer.h"

//-----------------------------------------------------------------------------
vtkLidarStream::vtkLidarStream()
{

}

//-----------------------------------------------------------------------------
void vtkLidarStream::PrintSelf(std::ostream &os, vtkIndent indent)
{

}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetNumberOfFrames()
{
  std::cerr << "this is not implemented yet" << std::endl;
  return 0;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkLidarStream::GetFrame(int frameNumber, int wantedNumberOfTrailingFrames)
{
  std::cerr << "this is not implemented yet" << std::endl;
  return nullptr;
}

//-----------------------------------------------------------------------------
std::string vtkLidarStream::GetOutputFile()
{
  return this->Internal->OutputFileName;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetOutputFile(const std::string &filename)
{
  this->Internal->OutputFileName  = filename;
}

//-----------------------------------------------------------------------------
std::string vtkLidarStream::GetForwardedIpAddress()
{
  return this->Internal->Network->ForwardedIpAddress;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedIpAddress(const std::string &ipAddress)
{
  this->Internal->Network->ForwardedIpAddress = ipAddress;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetLIDARPort()
{
  return this->Internal->Network->LIDARPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetLIDARPort(const int value)
{
  this->Internal->Network->LIDARPort = value;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::GetForwardedLIDARPort()
{
  return this->Internal->Network->ForwardedLIDARPort;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetForwardedLIDARPort(const int value)
{
  this->Internal->Network->ForwardedLIDARPort = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetIsForwarding()
{
  return this->Internal->Network->IsForwarding;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetIsForwarding(const bool value)
{
  this->Internal->Network->IsForwarding = value;
}

//-----------------------------------------------------------------------------
bool vtkLidarStream::GetIsCrashAnalysing()
{
  return this->Internal->Network->IsCrashAnalysing;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetIsCrashAnalysing(const bool value)
{
  this->Internal->Network->IsCrashAnalysing = value;
}

//-----------------------------------------------------------------------------
int vtkLidarStream::RequestInformation(
  vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  std::vector<double> timesteps = this->Internal->Consumer->GetTimesteps();
  const size_t nTimesteps = timesteps.size();
  if (nTimesteps > 0)
  {
    outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(),
      static_cast<int>(nTimesteps));
  }
  else
  {
    outInfo->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
  }

  double timeRange[2] = { 0.0, nTimesteps ? nTimesteps - 1.0 : 0.0 };
  outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);

  return 1;
}

//----------------------------------------------------------------------------
int vtkLidarStream::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkDataSet* output = vtkDataSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  double timeRequest = 0;
  if (outInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    timeRequest = outInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
  }

  double actualTime;
  vtkSmartPointer<vtkPolyData> polyData(NULL);
//  if (this->Internal->Consumer->GetNumberOfTrailingFrames() > 0)
//  {
//    polyData = this->Internal->Consumer->GetDatasetsForTime(
//      timeRequest, actualTime, this->Internal->Consumer->GetNumberOfTrailingFrames());
//  }
//  else
//  {
    polyData = this->Internal->Consumer->GetDatasetForTime(timeRequest, actualTime);
//  }

  if (polyData)
  {
    // printf("request %f, returning %f\n", timeRequest, actualTime);
    output->GetInformation()->Set(vtkDataObject::DATA_TIME_STEP(), actualTime);
    output->ShallowCopy(polyData);
  }

  return 1;
}

//-----------------------------------------------------------------------------
void vtkLidarStream::SetPimpInternal(vtkLidarStreamInternal *internal, LidarPacketInterpretor *interpretor)
{
  vtkLidarProvider::SetPimpInternal(interpretor);
  this->Internal = internal;
}
