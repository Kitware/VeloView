//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre
// Data: 04-15-2019
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

// LOCAL
#include "vtkOpenCVVideoReader.h"
#include "vtkOpenCVConversions.h"

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

// VTK
#include <vtkDataArray.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPNGWriter.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>

// OPENCV
#include <opencv2/opencv.hpp>

// BOOST
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Dense>

//-----------------------------------------------------------------------------
struct VideoStreamInformation
{
  //! Current position in msec in the stream
  double CurrentPosMsec;
  //! Relative current pose in the stream
  double RelativeCurrentPos;
  //! Size of the image
  unsigned int Width;
  unsigned int Height;
  //! number of FPS
  double Fps;
  //! Total number of frames
  unsigned int NbrFrame;
  //! Current position index in the stream
  unsigned int CurrentPosIndex;
  //! Number of channels in the stream
  unsigned int NChannels;
  //! Update the video informations
  void UpdateInfo(const cv::VideoCapture* video);
};

//-----------------------------------------------------------------------------
struct VideoFramePosition
{
  VideoFramePosition() = default;

  VideoFramePosition(int argIndex, double argTime)
  {
    Index = argIndex;
    Time = argTime;
  }

  //! Index of the frame
  int Index = 0;
  //! Time of the frame
  double Time = 0;
};

//-----------------------------------------------------------------------------
class vtkOpenCVVideoReaderInternal
{
public:
  vtkOpenCVVideoReaderInternal(vtkOpenCVVideoReader* obj);

  /**
   * @brief ReadFrameInformation read the video stream meta information
   */
  int ReadVideoInformation();

  /**
   * @brief SetTimestepInformation indicate to vtk which time step are available
   * @param info
   */
  void SetTimestepInformation(vtkInformation* info);

  /**
   * @brief UpdateVideoInfo updates the meta information of the video
   */
  void UpdateVideoInfo();

  //! Parent OpenCVVideoReader
  vtkOpenCVVideoReader* Parent;

  //! frame index which enable to jump quicky to a given frame
  std::vector<VideoFramePosition> FramesPosition;

  //! Video reader
  cv::VideoCapture Video;

  //! Video meta information
  VideoStreamInformation VideoInfo;

  //! Video filename
  std::string FileName;

  //! Timeshift between the video
  //! and the vehicle internal clock
  double TimeOffset;

  //! Output image dimensions
  int DataExtend[6];
  double Origin[3];
  double Scale[3];
  int NChannels;
};

//-----------------------------------------------------------------------------
vtkOpenCVVideoReaderInternal::vtkOpenCVVideoReaderInternal(vtkOpenCVVideoReader* obj)
{
  this->Parent = obj;
}

//-----------------------------------------------------------------------------
int vtkOpenCVVideoReaderInternal::ReadVideoInformation()
{
  this->FramesPosition.resize(this->VideoInfo.NbrFrame);
  for (unsigned int frameIndex = 0; frameIndex < this->VideoInfo.NbrFrame; frameIndex++)
  {
    this->FramesPosition[frameIndex] = VideoFramePosition(frameIndex,
                                                          static_cast<double>(frameIndex) / this->VideoInfo.Fps + this->TimeOffset);
  }
}

//-----------------------------------------------------------------------------
void vtkOpenCVVideoReaderInternal::SetTimestepInformation(vtkInformation* info)
{
  const size_t numberOfTimesteps = this->FramesPosition.size();
  std::vector<double> timesteps;
  for (size_t i = 0; i < numberOfTimesteps; i++)
  {
    timesteps.push_back( this->FramesPosition[i].Time);
  }

  if (this->FramesPosition.size())
  {
    double timeRange[2] = {timesteps.front(), timesteps.back()};
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
  }
  else
  {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
  }
}

//-----------------------------------------------------------------------------
void vtkOpenCVVideoReaderInternal::UpdateVideoInfo()
{
  this->VideoInfo.UpdateInfo(&this->Video);
  this->DataExtend[0] = 0; this->DataExtend[1] = this->VideoInfo.Width - 1;
  this->DataExtend[2] = 0; this->DataExtend[3] = this->VideoInfo.Height - 1;
  this->DataExtend[4] = 0; this->DataExtend[5] = 0;
  this->Scale[0] = 100.0 / static_cast<double>(this->VideoInfo.Width);
  this->Scale[1] = this->Scale[0]; this->Scale[2] = this->Scale[0];
  this->Origin[0] = -50.0;
  this->Origin[1] = -Scale[0] * static_cast<double>(this->VideoInfo.Height) / 2.0;
  this->Origin[2] = 0.0;
  this->NChannels = this->VideoInfo.NChannels;
}

//-----------------------------------------------------------------------------
void VideoStreamInformation::UpdateInfo(const cv::VideoCapture* video)
{
  this->CurrentPosMsec = video->get(cv::CAP_PROP_POS_MSEC);
  this->RelativeCurrentPos = video->get(cv::CAP_PROP_POS_AVI_RATIO);
  this->Fps = video->get(cv::CAP_PROP_FPS);
  this->Height = static_cast<unsigned int>(video->get(cv::CAP_PROP_FRAME_HEIGHT));
  this->Width = static_cast<unsigned int>(video->get(cv::CAP_PROP_FRAME_WIDTH));
  this->NbrFrame = static_cast<unsigned int>(video->get(cv::CAP_PROP_FRAME_COUNT));
  this->CurrentPosIndex = static_cast<unsigned int>(video->get(cv::CAP_PROP_POS_FRAMES));
}

// Implementation of the New function
vtkStandardNewMacro(vtkOpenCVVideoReader)

//----------------------------------------------------------------------------
vtkOpenCVVideoReader::vtkOpenCVVideoReader()
{
  this->Internal = new vtkOpenCVVideoReaderInternal(this);
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkOpenCVVideoReader::~vtkOpenCVVideoReader()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vtkOpenCVVideoReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  std::stringstream info;
  info << "FileName: " << this->Internal->FileName << "\n";
  info << "CurrentPosMsec: " << this->Internal->VideoInfo.CurrentPosMsec << "\n";
  info << "RelativeCurrentPos: " << this->Internal->VideoInfo.RelativeCurrentPos << "\n";
  info << "Fps: " << this->Internal->VideoInfo.Fps << "\n";
  info << "Height: " << this->Internal->VideoInfo.Height << "\n";
  info << "Width: " << this->Internal->VideoInfo.Width << "\n";
  info << "NbrFrame: " << this->Internal->VideoInfo.NbrFrame << "\n";
  info << "CurrentPosIndex: " << this->Internal->VideoInfo.CurrentPosIndex;
  std::cout << info.str() << std::endl;
}

//-----------------------------------------------------------------------------
int vtkOpenCVVideoReader::RequestInformation(vtkInformation* vtkNotUsed(request),
                                             vtkInformationVector** vtkNotUsed(inputVector),
                                             vtkInformationVector* outputVector)
{
  if (!this->Internal->FileName.empty() && this->Internal->FramesPosition.empty())
  {
    this->Internal->ReadVideoInformation();
  }
  vtkInformation* info = outputVector->GetInformationObject(0);
  this->Internal->SetTimestepInformation(info);

  // Set origin and scale information
  info->Set(vtkDataObject::ORIGIN(), this->Internal->Origin, 3);
  info->Set(vtkDataObject::SPACING(), this->Internal->Scale, 3);

  // set data dimension information
  info->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), this->Internal->DataExtend, 6);
  vtkDataObject::SetPointDataActiveScalarInfo(info, VTK_UNSIGNED_CHAR, 3);

  return 1;
}

//-----------------------------------------------------------------------------
int vtkOpenCVVideoReader::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  vtkImageData* output = vtkImageData::GetData(outputVector);
  vtkInformation* info = outputVector->GetInformationObject(0);

  if (this->Internal->FileName.empty())
  {
    vtkErrorMacro("FileName has not been set.");
    return 0;
  }

  double timestep = 0.0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    timestep = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
  }

  // iterating over all timesteps until finding the first one with a greater time value
  // this is suboptimal
  int frameRequested = 0;
  for (; timestep > this->Internal->FramesPosition[frameRequested].Time; frameRequested++);

  if (frameRequested < 0 || frameRequested >= this->GetNumberOfFrames())
  {
    vtkErrorMacro("Cannot meet timestep request: " << frameRequested << ".  Have "
                                                   << this->GetNumberOfFrames() << " datasets.");
    return 0;
  }

  // Update Video info
  this->Internal->UpdateVideoInfo();
  this->Internal->Video.set(cv::CAP_PROP_POS_FRAMES, frameRequested);

  // Get the image for the current position
  cv::Mat cvImage;
  if (!this->Internal->Video.read(cvImage))
  {
    vtkErrorMacro("Not able to read frame: " << frameRequested);
    return 0;
  }

  // Convert cvMat to vtkImageData
  output->ShallowCopy(CvImageToVtkImage(cvImage));
  output->SetOrigin(this->Internal->Origin);
  output->SetSpacing(this->Internal->Scale);
  output->SetExtent(this->Internal->DataExtend);

  return 1;
}

//-----------------------------------------------------------------------------
int vtkOpenCVVideoReader::GetNumberOfFrames()
{
  return this->Internal->VideoInfo.NbrFrame;
}

//-----------------------------------------------------------------------------
void vtkOpenCVVideoReader::SetFileName(const char* filename)
{
  if (!this->Internal->Video.open(std::string(filename)))
  {
    vtkGenericWarningMacro("Can not load video:" << filename
                           << " check that the file exists and has the correct extension");
  }
  this->Internal->FileName = std::string(filename);
  this->Internal->FramesPosition = std::vector<VideoFramePosition>();
  this->Internal->UpdateVideoInfo();
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkOpenCVVideoReader::SetTimeOffset(double argTs)
{
  this->Internal->TimeOffset = argTs;
}

//-----------------------------------------------------------------------------
double vtkOpenCVVideoReader::GetTimeOffset()
{
  return this->Internal->TimeOffset;
}
