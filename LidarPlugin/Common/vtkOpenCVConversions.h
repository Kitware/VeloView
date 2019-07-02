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

#ifndef VTK_OPENCV_CONVERSIONS_H
#define VTK_OPENCV_CONVERSIONS_H

// VTK
#include <vtkImageData.h>
#include <vtkSmartPointer.h>

// OPENCV
#include <opencv2/core.hpp>

/**
   * @brief CvImageToVtkImage converts a OpenCV BGR CV_8UC3 image to a vtkImageData
   *
   * No allocation is made. Uses only OpenCV functions (for speed). Speed up
   * compared to 3 simple loops is over 100x.
   * @param inImg input read-only image which is a valid cv::Mat image of type
   * CV_8UC3, that uses BGR colorspace (the default for OpenCV, we cannot check
   * for error on the colorspace). This is the type of cv::Mat you get when
   * reading a JPEG/PNG image with colors.
   * @param outIm output write-only image which is already allocated to the
   * correct dimensions
   * @return true iff the conversion succeeds
   */
bool CvImageToVtkImage(const cv::Mat& inImg, vtkSmartPointer<vtkImageData> outImg);

/**
   * @brief CvImageToVtkImage converts a OpenCV BGR CV_8UC3 image to a vtkImageData
   *
   * @param inImg input read-only image which is a valid cv::Mat image of type
   * CV_8UC3, that uses BGR colorspace (the default for OpenCV, we cannot check
   * for error on the colorspace). This is the type of cv::Mat you get when
   * reading a JPEG/PNG image with colors.
   */
vtkSmartPointer<vtkImageData> CvImageToVtkImage(const cv::Mat& inImg);

#endif // VTK_OPENCV_CONVERSIONS_H
