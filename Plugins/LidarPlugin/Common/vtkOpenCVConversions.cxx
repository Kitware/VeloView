//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Authors: Gabriel Devillers, Pierre Guilbert
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

// LOCAL
#include "vtkOpenCVConversions.h"
#include <opencv2/imgproc.hpp>

// VTK
#include <vtkDataArray.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>

//----------------------------------------------------------------------------
bool CvImageToVtkImage(const cv::Mat& inImg, vtkSmartPointer<vtkImageData> outImg)
{
  // Because we are going to reinterpret the memory, it is important to check
  // that allocated memory has the shape we expect
  int outImgDimensions[3] = { 0, 0, 0 };
  outImg->GetDimensions(outImgDimensions);
  if (outImgDimensions[0] != inImg.cols
      || outImgDimensions[1] != inImg.rows
      || outImgDimensions[2] != 1)
  {
    return false;
  }

  // We have to things to do: change the color channels from BGR to RGB,
  // and flip horizontally. We want to use OpenCV to avoid loops and because
  // OpenCV should have optimized functions, so we interpret the output
  // vtkImageData as an OpenCV matrix.
  // openCVInterp does not own its memory so outImg data will not be freed
  // when this function returns.
  cv::Mat openCVInterp(inImg.rows, inImg.cols, CV_8UC3, outImg->GetScalarPointer());

  // Flip the image horizontally. This is needed because vtkImages have their
  // origin in the bottom left corner of the image, where as cv::Mat have their
  // origin in the top left corner of the image.
  bool flipHorizontally = true;
  if (flipHorizontally)
  {
    // this makes a call to memcpy redundant
    flip(inImg, openCVInterp, 0);
  }
  else
  {
    size_t N = inImg.rows * inImg.cols * inImg.channels();
    memcpy(outImg->GetScalarPointer(), inImg.data, N);
  }

  // Adapts from OpenCV colorspace (BGR) to VTK colorspace (RGB)
  cv::cvtColor(openCVInterp, openCVInterp, cv::COLOR_BGR2RGB);

  // no image to copy, because openCVInterp data is owned by outImg
  return true;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkImageData> CvImageToVtkImage(const cv::Mat& inImg)
{
  // get input image information
  int H = inImg.rows;
  int W = inImg.cols;
  int channels = inImg.channels();

  // Create output image
  vtkSmartPointer<vtkImageData> outImg = vtkSmartPointer<vtkImageData>::New();
  outImg->SetDimensions(W, H, 1);
  outImg->AllocateScalars(VTK_UNSIGNED_CHAR, channels);

  // Populate the vtkImageData
  CvImageToVtkImage(inImg, outImg);
  return outImg;
}
