//=========================================================================
//
// Copyright 2018 Kitware, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//=========================================================================
#include "vtkSlamManager.h"

#include <vtkObjectFactory.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <sstream>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlamManager)

//----------------------------------------------------------------------------
void vtkSlamManager::PrintSelf(std::ostream &os, vtkIndent indent)
{
  os << indent << "Slam Manager: " << std::endl;
  #define PrintParameter(param) os << indent << #param << " " << this->param << std::endl;
  PrintParameter(AllFrame)
  PrintParameter(FirstFrame)
  PrintParameter(LastFrame)
  PrintParameter(StepSize)
  vtkIndent paramIndent = indent.GetNextIndent();
  this->Superclass::PrintSelf(os, paramIndent);
}

//----------------------------------------------------------------------------
vtkSlamManager::vtkSlamManager()
{
  this->SetProgressText("Computing slam");
}

//----------------------------------------------------------------------------
int vtkSlamManager::RequestUpdateExtent(vtkInformation *vtkNotUsed(request),
                                        vtkInformationVector **inputVector,
                                        vtkInformationVector *vtkNotUsed(outputVector))
{
  // Get the time and force it
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  double time = *(inInfo->Get(vtkStreamingDemandDrivenPipeline::TIME_STEPS()) + this->CurrentFrame);
  inInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(), time);
  return 1;
}

//----------------------------------------------------------------------------
int vtkSlamManager::RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Check parameters validity
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  int nb_time_steps = inInfo->Length(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
  if (this->StepSize <= 0)
  {
    vtkErrorMacro("StepSize must be greater than zero!")
    return 0;
  }
  if (this->LastFrame < 0 || this->FirstFrame < 0)
  {
    vtkErrorMacro("FirstFrame and LastFrame must be positive integers!")
    return 0;
  }
  if (this->FirstFrame > nb_time_steps - 1 || this->LastFrame > nb_time_steps -1)
  {
    vtkErrorMacro("The dataset only has " << nb_time_steps << " frames!")
    return 0;
  }
  if (this->LastFrame < this->FirstFrame )
  {
    vtkErrorMacro(<< "The last frame must come after the first frame!")
    return 0;
  }

  // First Iteration
  if (this->FirstIteration)
  {
    // Check if only the timstamp change, in this case we don't need to rerun the slam
    // we just need to copy the cache
    if (this->ParametersModificationTime.GetMTime() <= this->LastModifyTime)
    {
      for (int i = 0; i < this->GetNumberOfOutputPorts(); ++i)
      {
        auto *output = vtkPolyData::GetData(outputVector->GetInformationObject(i));
        output->ShallowCopy(this->Cache[i]);
      }
      return 1;
    }
    this->FirstIteration = false;
    this->Reset();
    this->CurrentFrame = this->AllFrame ? 0 : this->FirstFrame;
  }

  // relaunch the pipeline if necessary
  int firstFrame = this->AllFrame ? 0 : this->FirstFrame;
  int lastFrame = this->AllFrame ? nb_time_steps - 1 : this->LastFrame;
  int candidateNextFrame = this->CurrentFrame + this->StepSize;
  bool lastIteration = candidateNextFrame > lastFrame;
  if (!lastIteration)
  {
    request->Set(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING(), 1);
    this->CurrentFrame = candidateNextFrame;
  }
  // stop the pipeline
  else
  {
    request->Remove(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING());
    this->FirstIteration = true;
    this->LastModifyTime = this->ParametersModificationTime.GetMTime();
  }
  double progress = static_cast<double>(this->CurrentFrame - firstFrame)
      / static_cast<double>(lastFrame - firstFrame);
  this->UpdateProgress(progress);

  // process the frame
  vtkSlam::RequestData(request, inputVector, outputVector);

  // save data to the cache at the end
  if (lastIteration)
  {
    this->Cache.clear();
    for (int i = 0; i < this->GetNumberOfOutputPorts(); ++i)
    {
      auto output = vtkSmartPointer<vtkPolyData>::New();
      output->DeepCopy(vtkPolyData::GetData(outputVector->GetInformationObject(i)));
      this->Cache.push_back(output);
    }
  }

  return 1;
}
