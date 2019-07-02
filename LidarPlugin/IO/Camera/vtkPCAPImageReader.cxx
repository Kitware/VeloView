//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Gabriel Devillers
// Date: 2019-06-26
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

#include "vtkPCAPImageReader.h"

#include <algorithm>
#include <sstream>

#include "vtkPacketFileReader.h"
#include "vtkOpenCVConversions.h"
#include "statistics.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vtkObjectFactory.h>
#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCAPImageReader)

//------------------------------------------------------------------------------
vtkPCAPImageReader::vtkPCAPImageReader()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//------------------------------------------------------------------------------
bool vtkPCAPImageReader::UpdateFrameSize()
{
  if (this->FrameCatalog.empty())
  {
    return false;
  }
  this->Open();
  cv::Mat cvImage = this->GetOpenCVFrame(0);
  this->Close();
  this->Width = cvImage.cols;
  this->Height = cvImage.rows;

  this->Extent[0] = 0;
  this->Extent[1] = this->Width - 1;
  this->Extent[2] = 0;
  this->Extent[3] = this->Height - 1;
  this->Extent[4] = 0;
  this->Extent[5] = 0;
  double scale = 100.0 / static_cast<double>(this->Width);
  this->Scale[0] = scale;
  this->Scale[1] = scale;
  this->Scale[2] = scale;
  this->Origin[0] = -50.0;
  this->Origin[1] = - scale * static_cast<double>(this->Height) / 2.0;
  this->Origin[2] = 0.0;

  return true;
}

//------------------------------------------------------------------------------
void ComputePeriodAndShift(std::vector<cv::Point2d> toFit, double& period, double& shift)
{
  cv::Vec4d params;
  cv::fitLine(toFit, params, cv::DIST_L2, 0.0, 0.0, 0.0);
  period = params[1] / params[0];
  double alpha = - params[2] / params[0];
  shift = params[3] + alpha * params[1]; // ordinate at origin
}

//------------------------------------------------------------------------------
void AnalyzeTimes(std::vector<double> pcapTimes, bool& someFramesSkipped, int& maxFrameSkipped, bool& haveResults, double& period, double& shift)
{
  const size_t testSampleSize = 100;
  std::vector<cv::Point2d> toFit;
  toFit.reserve(pcapTimes.size());

  double currentTime = pcapTimes[0];
  for (size_t i = 1; i < pcapTimes.size(); i++)
  {
    if (currentTime > pcapTimes[i])
    {
      // new time is in the past, skip it
      continue;
    }
    currentTime = pcapTimes[i];
    toFit.push_back(cv::Point2d(static_cast<double>(i), pcapTimes[i]));
  }

  if (toFit.size() < 2 * testSampleSize)
  {
    someFramesSkipped = false; // not enough frames to tell, but this case should be very rare so it is ok to "lie"
  }
  else
  {
    auto startSamples = std::vector<cv::Point2d>(toFit.begin(), toFit.begin() + testSampleSize);
    auto endSamples = std::vector<cv::Point2d>(toFit.end() - testSampleSize, toFit.end());
    double startPeriod, startShift, endPeriod, endShift;
    ComputePeriodAndShift(startSamples, startPeriod, startShift);
    ComputePeriodAndShift(endSamples, endPeriod, endShift);
    double diff = std::abs(startShift - endShift);
    someFramesSkipped = (diff >= 0.5 * startPeriod);
    maxFrameSkipped = std::ceil(diff / startPeriod);
  }

  if (toFit.size() > 3)
  {
    haveResults = true;
    ComputePeriodAndShift(toFit, period, shift);
  }
  else
  {
    haveResults = false;
  }
}

//------------------------------------------------------------------------------
bool isImagePacket(const unsigned char* data, size_t dataLength)
{
  unsigned char identifier[5] = {0x4a, 0x46, 0x49, 0x46, 0x0}; // JFIF followed by null byte
  if (dataLength < 11)
  {
    return false;
  }
  return memcmp(data + 6, identifier, 5) == 0;
}

//------------------------------------------------------------------------------
int vtkPCAPImageReader::ReadFrameInformation()
{
  this->Open();
  const unsigned char* data = nullptr;
  unsigned int dataLength = 0;
  // bool firstIteration = true;

  // reset the frame catalog to build a new one
  this->FrameCatalog.clear();

  // keep track of the file position
  // and the network timestamp of the
  // current udp packet to process
  fpos_t lastFilePosition;
  double lastPacketNetworkTime = 0;
  this->Reader->GetFilePosition(&lastFilePosition);

  bool timeBugDetected = false;

  while (this->Reader->NextPacket(data, dataLength, lastPacketNetworkTime))
  {
    // This command sends a signal that can be observed from outside
    // and that is used to diplay a Qt progress dialog from Python
    // This progress dialog is not displaying a progress percentage,
    // thus it is ok to pass 0.0
    this->UpdateProgress(0.0);

    // We skip any packet wich does not contain an image
    if (!isImagePacket(data, dataLength))
    {
      this->Reader->GetFilePosition(&lastFilePosition);
      continue;
    }

    if (this->FrameCatalog.size() > 0
        && lastPacketNetworkTime < this->FrameCatalog[this->FrameCatalog.size() - 1].FirstPacketNetworkTime)
    {
      // fix a bug we have on the current data
      timeBugDetected = true;
      lastPacketNetworkTime += 1.0;
    }
    struct FrameInformation currentFrameInfo;
    currentFrameInfo.FilePosition = lastFilePosition;
    // "FirstPacket" is not the best name for this use case, because an image
    // is contained in a single packet, so the first is also the only one
    // consituting the frame.
    currentFrameInfo.FirstPacketNetworkTime = lastPacketNetworkTime;
    // "FirstPacketDataTime" makes no sens in this use case, because there is no
    // per-pixel time in the image (there is no time at all actually).
    currentFrameInfo.FirstPacketDataTime = 0.0;
    currentFrameInfo.SpecificInformation = nullptr; // no such data
    this->FrameCatalog.push_back(currentFrameInfo);

    this->Reader->GetFilePosition(&lastFilePosition);
  }

  if (this->FrameCatalog.size() == 0)
  {
    vtkErrorMacro("The reader could not parse the pcap file")
  }

  for (size_t i = 0; i < this->FrameCatalog.size(); i++)
  {
    this->FrameCatalog[i].FirstPacketNetworkTime += this->TimeOffset;
  }

  // Warn if we detect that a packet was dropped, this is important because
  // then the 'corrected' network time will be wrong
  if (timeBugDetected)
  {
    vtkWarningMacro("detected a bug in packet times (time not monotonic)."
                    " Fixing it by adding 1.0 second to packet time")
  }

  return this->GetNumberOfFrames();
}

//------------------------------------------------------------------------------
void vtkPCAPImageReader::SetTimestepInformation(vtkInformation *info)
{
  if (this->FrameCatalog.size() == 0)
  {
    return;
  }
  size_t numberOfTimesteps = this->FrameCatalog.size();
  std::vector<double> timesteps(numberOfTimesteps);
  for (size_t i = 0; i < numberOfTimesteps; ++i)
  {
    timesteps[i] = this->FrameCatalog[i].FirstPacketNetworkTime;
  }
  if (this->FrameCatalog.size() != 0)
  {
    double* firstTimestepPointer = &timesteps.front();
    double* lastTimestepPointer = &timesteps.back();
    double timeRange[2] = { *firstTimestepPointer, *lastTimestepPointer };
    // In order to avoid to display
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), firstTimestepPointer, numberOfTimesteps);
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
  }
  else
  {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
  }
}

//------------------------------------------------------------------------------
void vtkPCAPImageReader::SetFileName(const std::string &filename)
{
  if (filename == this->FileName)
  {
    return;
  }

  this->FileName = filename;
  this->FrameCatalog.clear();
  this->Modified();
}

//------------------------------------------------------------------------------
cv::Mat vtkPCAPImageReader::GetOpenCVFrame(int frameNumber)
{
  if (!this->Reader)
  {
    vtkErrorMacro("GetOpenCVFrame() called but packet file reader is not open.")
    return cv::Mat(0, 0, CV_8UC3);
  }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart;

  // Update the interpreter meta data according to the requested frame
  FrameInformation currInfo= this->FrameCatalog[frameNumber];
  this->Reader->SetFilePosition(&currInfo.FilePosition);

  if (!this->Reader->NextPacket(data, dataLength, timeSinceStart))
  {
    vtkErrorMacro("vtkPacketFileReader::GetOpenCVFrame() failed to access packet")
    return cv::Mat(0, 0, CV_8UC3);
  }

  // Use OpenCV to read the JPEG JFIF image from data.
  // If the memory at data was written to a file you would get a standard image file
  // source: https://stackoverflow.com/questions/14727267/opencv-read-jpeg-image-from-buffer
  // Create a Size(1, nSize) Mat object of 8-bit, single-byte elements
  cv::Mat rawData(1, dataLength, CV_8UC1, (void*)data);
  cv::Mat decodedImage = cv::imdecode(rawData,cv::IMREAD_UNCHANGED|cv::IMREAD_COLOR);
  if (decodedImage.data == nullptr)
  {
    vtkErrorMacro("Error decoding raw image data")
    return cv::Mat(0, 0, CV_8UC3);
  }
  return decodedImage;
}

//------------------------------------------------------------------------------
void vtkPCAPImageReader::Open()
{
  this->Close();
  this->Reader = new vtkPacketFileReader;

  std::string filterPCAP = "udp";
  if (this->NetworkPort != -1)
  {
    filterPCAP += " port " + std::to_string(this->NetworkPort);
  }
  if (!this->Reader->Open(this->FileName, filterPCAP.c_str()))
  {
    vtkErrorMacro(<< "Failed to open packet file: " << this->FileName << "!\n"
                                                 << this->Reader->GetLastError())
    this->Close();
  }
}

//------------------------------------------------------------------------------
void vtkPCAPImageReader::Close()
{
  delete this->Reader;
  this->Reader = 0;
}

//------------------------------------------------------------------------------
int vtkPCAPImageReader::RequestData(vtkInformation *vtkNotUsed(request),
                                vtkInformationVector **vtkNotUsed(inputVector),
                                vtkInformationVector *outputVector)
{
  vtkImageData* output = vtkImageData::GetData(outputVector);
  vtkInformation* info = outputVector->GetInformationObject(0);

  if (this->FileName.empty())
  {
    vtkErrorMacro("FileName has not been set.");
    return 0;
  }

  if (this->FrameCatalog.size() == 0) // This mean that the reader did not manage to parse the pcap file
  {
    return 1;
  }

  double timestep = 0.0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    timestep = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
  }

  // iterating over all timesteps until finding the first one with a greater time value
  auto idx = std::lower_bound(this->FrameCatalog.begin(),
                              this->FrameCatalog.end(),
                              timestep,
                              [](FrameInformation& fp, double d)
                                { return fp.FirstPacketNetworkTime < d; });

  auto frameRequested = std::distance(this->FrameCatalog.begin(), idx);

  if (idx == this->FrameCatalog.end())
  {
    vtkErrorMacro("Cannot meet timestep request: " << frameRequested << ".  Have "
                                                   << this->GetNumberOfFrames() << " datasets.");
    return 0;
  }

  //! @todo we should no open the pcap file everytime a frame is requested !!!
  this->Open();
  cv::Mat cvImage = this->GetOpenCVFrame(frameRequested);
  // Convert cvMat to vtkImageData
  vtkSmartPointer<vtkImageData> vtkImage = CvImageToVtkImage(cvImage);

  output->ShallowCopy(vtkImage);
  output->SetOrigin(this->Origin);
  output->SetSpacing(this->Scale);
  output->SetExtent(this->Extent);

  this->Close();

  return 1;
}

//------------------------------------------------------------------------------
int vtkPCAPImageReader::RequestInformation(vtkInformation* request,
                                       vtkInformationVector** inputVector,
                                       vtkInformationVector* outputVector)
{
  this->Superclass::RequestInformation(request, inputVector, outputVector);
  if (!this->FileName.empty() && this->FrameCatalog.empty())
  {
    this->ReadFrameInformation();
  }
  vtkInformation* info = outputVector->GetInformationObject(0);
  this->SetTimestepInformation(info);

  // early exit with success status, in case the reader was instanciated without FileName
  if (this->FrameCatalog.empty())
  {
    return 1;
  }

  if (!this->UpdateFrameSize())
  {
    return 0;
  }

  // Set origin and scale information
  info->Set(vtkDataObject::ORIGIN(), this->Origin, 3);
  info->Set(vtkDataObject::SPACING(), this->Scale, 3);

  // set data dimension information
  info->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), this->Extent, 6);
  vtkDataObject::SetPointDataActiveScalarInfo(info, VTK_UNSIGNED_CHAR, 3);

  return 1;
}
