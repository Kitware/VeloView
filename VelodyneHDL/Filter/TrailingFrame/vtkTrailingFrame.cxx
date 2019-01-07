#include "vtkTrailingFrame.h"

#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkInformationVector.h>
#include <vtkInformation.h>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkTrailingFrame)

//----------------------------------------------------------------------------
vtkTrailingFrame::vtkTrailingFrame()
  : NumberOfTrailingFrames(0),
    PipelineTime(0),
    LastTimeProcessed(-1),
    FirstFilterIteration(true)
{
  this->CacheTimeRange[0] = -1;
  this->CacheTimeRange[1] = -1;
}

//----------------------------------------------------------------------------
void vtkTrailingFrame::SetNumberOfTrailingFrames(const unsigned int value)
{
  if (this->NumberOfTrailingFrames != value)
  {
    this->NumberOfTrailingFrames = value;
    this->CacheTimeRange[0] = -1;
    this->CacheTimeRange[1] = -1;
    this->Modified();
  }
}

//----------------------------------------------------------------------------
int vtkTrailingFrame::FillOutputPortInformation(int port, vtkInformation *info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMultiBlockDataSet");
    return 1;
  }

  return 0;
}

//----------------------------------------------------------------------------
int vtkTrailingFrame::RequestUpdateExtent(vtkInformation* vtkNotUsed(request),
                                      vtkInformationVector** inputVector,
                                      vtkInformationVector* vtkNotUsed(outputVector))
{
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);

  // first loop
  if (this->FirstFilterIteration)
  {
    // Save current pipeline time step
    this->PipelineTime = static_cast<int>(
      inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()));

    // save old TimeRange and update new one
    int previousCacheTimeRange[2] = {this->CacheTimeRange[0],this->CacheTimeRange[1]};
    this->CacheTimeRange[0] = std::max(static_cast<int>(this->PipelineTime - this->NumberOfTrailingFrames), 0);
    this->CacheTimeRange[1] = this->PipelineTime + 1;

    // handle case when changing NumberOfTrailingFrame
    if (previousCacheTimeRange[1] == -1)
    {
      // move the old  timestep to the right location
      vtkNew<vtkMultiBlockDataSet> oldCache;
      oldCache->ShallowCopy(this->Cache.GetPointer());
      int previousNumberOfTrailingFrame = oldCache->GetNumberOfBlocks() - 1;
      this->Cache->Initialize();
      for (this->LastTimeProcessed = this->CacheTimeRange[1] - 1;
           this->LastTimeProcessed > this->CacheTimeRange[1] - 1 - previousNumberOfTrailingFrame &&
           this->LastTimeProcessed > this->CacheTimeRange[0];
           this->LastTimeProcessed--)
      {
        unsigned int previousIndex = this->LastTimeProcessed % (previousNumberOfTrailingFrame + 1);
        unsigned int newIndex = this->LastTimeProcessed % (this->NumberOfTrailingFrames + 1);
        this->Cache->SetBlock(newIndex, oldCache->GetBlock(previousIndex));
      }
      this->Direction = -1;

    }
    // handle case when jumping backward
    else if (this->CacheTimeRange[1] < previousCacheTimeRange[1])
    {
      this->Direction = -1;
      this->LastTimeProcessed = std::max(0, std::min(this->CacheTimeRange[1], previousCacheTimeRange[0]) - 1);
    }
    // handle case when jumping forward
    else if (this->CacheTimeRange[1] > previousCacheTimeRange[1])
    {
      this->Direction = 1;
      this->LastTimeProcessed = std::max(this->CacheTimeRange[0], previousCacheTimeRange[1]);
    }
    this->FirstFilterIteration = false;
  }
  // not first loop
  else
  {
    this->LastTimeProcessed += this->Direction;
  }

  inInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(),
              this->LastTimeProcessed);
  return 1;
}

//----------------------------------------------------------------------------
int vtkTrailingFrame::RequestData(vtkInformation* request,
                                  vtkInformationVector** inputVector,
                                  vtkInformationVector* outputVector)
{
  vtkPolyData* input = vtkPolyData::GetData(inputVector[0],0);
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::GetData(outputVector);

  if ((this->LastTimeProcessed == this->CacheTimeRange[0] && this->Direction == -1)
      || (this->LastTimeProcessed == this->CacheTimeRange[1]-1 && this->Direction == 1))
  {
    // Stop the pipeline loop
    request->Remove(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING());

    // reset block that should be empty
    for (int i = this->PipelineTime + 1; i < this->NumberOfTrailingFrames + 1; i++)
    {
      int index = i % (this->NumberOfTrailingFrames + 1);
      this->Cache->SetBlock(index, nullptr);
    }

    // reset some variable and pipeline time
    this->FirstFilterIteration = true;
    inInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(),
                this->PipelineTime);
  }
  else
  {
    // force the pipeline loop
    request->Set(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING(), 1);
  }

  // copy the input in the multiblock
  vtkNew<vtkPolyData> currentFrame;
  currentFrame->ShallowCopy(input);
  if(this->NumberOfTrailingFrames)
  {
    unsigned int index = static_cast<unsigned int>(this->LastTimeProcessed) % (this->NumberOfTrailingFrames + 1);
    this->Cache->SetBlock(index, currentFrame.GetPointer());
  }
  // handle case when no trailing frame
  else
  {
    this->Cache->SetBlock(0, currentFrame.GetPointer());
  }
  output->ShallowCopy(this->Cache.GetPointer());
  return 1;
}
