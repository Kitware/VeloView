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
    LastTimeProcessedIndex(-1),
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
  // get the available time steps from source (only once)
  if (this->TimeSteps.size() == 0)
  {
    double *TimeSteps = inInfo->Get(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    double *time_range = inInfo->Get(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
    int iter = 0;
    while (TimeSteps[iter] != time_range[1])
      ++iter;
    this->TimeSteps.assign(TimeSteps, TimeSteps + iter + 1);
  }

  // first loop
  if (this->FirstFilterIteration)
  {
    // Save current pipeline time step
    this->PipelineTime = inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());

    // get the index corresponding to the requested pipeline time
    // find the index of the first time step that is no less than pipeline time
    this->PipelineIndex = std::distance(this->TimeSteps.begin(),
                                        std::lower_bound(this->TimeSteps.begin(),
                                                         this->TimeSteps.end(),
                                                         this->PipelineTime));
    // check if the previous index is closer
    if (this->PipelineIndex > 0)
    {
      if (this->TimeSteps[this->PipelineIndex] - this->PipelineTime > this->PipelineTime - this->TimeSteps[this->PipelineIndex - 1])
        this->PipelineIndex -= 1;
    }
    // save old TimeRange and update new one
    int previousCacheTimeRange[2] = {this->CacheTimeRange[0], this->CacheTimeRange[1]};
    this->CacheTimeRange[0] = std::max(this->PipelineIndex - static_cast<int>(this->NumberOfTrailingFrames), 0);
    this->CacheTimeRange[1] = this->PipelineIndex + 1;

    // handle case when changing NumberOfTrailingFrame
    if (previousCacheTimeRange[1] == -1)
    {
      // move the old  timestep to the right location
      vtkNew<vtkMultiBlockDataSet> oldCache;
      oldCache->ShallowCopy(this->Cache.GetPointer());
      int previousNumberOfTrailingFrame = oldCache->GetNumberOfBlocks() - 1;
      this->Cache->Initialize();
      for (this->LastTimeProcessedIndex = this->CacheTimeRange[1] - 1;
           this->LastTimeProcessedIndex >= this->CacheTimeRange[1] - 1 - previousNumberOfTrailingFrame &&
           this->LastTimeProcessedIndex >= this->CacheTimeRange[0];
           this->LastTimeProcessedIndex--)
      {
        unsigned int previousIndex = this->LastTimeProcessedIndex % (previousNumberOfTrailingFrame + 1);
        unsigned int newIndex = this->LastTimeProcessedIndex % (this->NumberOfTrailingFrames + 1);
        this->Cache->SetBlock(newIndex, oldCache->GetBlock(previousIndex));
      }
      this->Direction = -1;
    }
    // handle case when jumping backward
    else if (this->CacheTimeRange[1] < previousCacheTimeRange[1])
    {
      this->Direction = -1;
      this->LastTimeProcessedIndex = std::max(0, std::min(this->CacheTimeRange[1], previousCacheTimeRange[0]) - 1);
    }
    // handle case when jumping forward
    else if (this->CacheTimeRange[1] > previousCacheTimeRange[1])
    {
      this->Direction = 1;
      this->LastTimeProcessedIndex = std::max(this->CacheTimeRange[0], previousCacheTimeRange[1]);
    }
    this->FirstFilterIteration = false;
  }
  // not first loop
  else
  {
    this->LastTimeProcessedIndex += this->Direction;
  }

  inInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(),
              this->TimeSteps[this->LastTimeProcessedIndex]);

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

  if ((this->LastTimeProcessedIndex <= this->CacheTimeRange[0] && this->Direction == -1)
      || (this->LastTimeProcessedIndex >= this->CacheTimeRange[1]-1 && this->Direction == 1))
  {
    // Stop the pipeline loop
    request->Remove(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING());

    // reset block that should be empty
    for (int i = this->PipelineIndex + 1; i < this->NumberOfTrailingFrames + 1; i++)
    {
      int index = i % (this->NumberOfTrailingFrames + 1);
      this->Cache->SetBlock(index, nullptr);
    }

    // reset some variable and pipeline time
    this->FirstFilterIteration = true;
    inInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(),
                this->TimeSteps[this->PipelineTime]);
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
    unsigned int index = static_cast<unsigned int>(this->LastTimeProcessedIndex) % (this->NumberOfTrailingFrames + 1);
    this->Cache->SetBlock(index, currentFrame.GetPointer());
  }
  // handle case when no trailing frame
  else
  {
    this->Cache->SetBlock(0, currentFrame.GetPointer());
  }
  output->ShallowCopy(this->Cache.GetPointer());

  // re-order output blocks so that:
  //    current frame => 0
  //    current frame - 1 => 1
  //    current frame - 2 => 2
  // ...
  if (output->GetNumberOfBlocks() > 0)
  {
    int index = this->LastTimeProcessedIndex % (this->NumberOfTrailingFrames + 1);
    int a = 0;
    for (int i = index; i >= 0; i--)
    {
      output->SetBlock(a++, this->Cache->GetBlock(i));
    }
    for (int i = output->GetNumberOfBlocks() - 1; i > index; --i)
    {
      output->SetBlock(a++, this->Cache->GetBlock(i));
    }
  }
  return 1;
}
