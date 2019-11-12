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

#ifndef VTKPCAPIMAGEREADER_H
#define VTKPCAPIMAGEREADER_H

#include <opencv2/core.hpp>

#include <vtkImageAlgorithm.h>

#include "vtkPacketFileReader.h"
#include "FrameInformation.h"


class VTK_EXPORT vtkPCAPImageReader : public vtkImageAlgorithm
{
public:
  static vtkPCAPImageReader* New();
  vtkTypeMacro(vtkPCAPImageReader, vtkImageAlgorithm)

  vtkGetMacro(NetworkPort, int)
  vtkSetMacro(NetworkPort, int)

  int GetNumberOfFrames() { return this->FrameCatalog.size(); }

  void SetFileName(const std::string &filename);
  cv::Mat GetOpenCVFrame(int frameNumber);

  vtkGetMacro(TimeOffset, double)
  vtkSetMacro(TimeOffset, double)

protected:
  vtkPCAPImageReader();
  ~vtkPCAPImageReader() = default;

  //! Name of the pcap file to read
  std::string FileName = "";

  //! Miscellaneous information about a frame that enable:
  //! - Quick jump to a frame index
  //! - ...
  std::vector<FrameInformation> FrameCatalog;

  //! libpcap wrapped reader which enable to get the raw pcap packet from the pcap file
  vtkPacketFileReader* Reader = nullptr;

  //! Filter the packet to only read the packet received on a specify port
  //! To read all packet use -1
  int NetworkPort = -1;

  //! TimeOffset in seconds relative to reception time in the PCAP
  double TimeOffset = 0.0;

  /**
   * @brief Open open the pcap file
   * @todo a decition should be made if the opening/closing of the pcap should be handle by
   * the class itself of the class user. Currently this is not clear
   */
  virtual void Open();

  /**
   * @brief Close close the pcap file
   * @todo a decition should be made if the opening/closing of the pcap should be handle by
   * the class itself of the class user. Currently this is not clear
   */
  virtual void Close();

  int RequestInformation(vtkInformation* request,
                         vtkInformationVector** inputVector,
                         vtkInformationVector* outputVector);

  int RequestData(vtkInformation *vtkNotUsed(request),
                  vtkInformationVector **vtkNotUsed(inputVector),
                  vtkInformationVector *outputVector);

private:
  vtkPCAPImageReader(const vtkPCAPImageReader&) = delete;
  void operator=(const vtkPCAPImageReader&) = delete;

  /**
   * @brief ReadFrameInformation reads the whole pcap and creates a frame index.
   */
  int ReadFrameInformation();

  int Width = 0;
  int Height = 0;
  int Extent[6] = {0, 0, 0, 0, 0, 0};
  double Origin[3] = {0.0, 0.0, 0.0};
  double Scale[3] = {0.0, 0.0, 0.0};

  /**
   * @brief Read the size of the first frame (if available) and related information
   * (Width, Height, Extent, Origin, Scale)
   *
   */
  bool UpdateFrameSize();

  /**
   * @brief SetTimestepInformation Set the timeStep available
   * @param info
   */
  void SetTimestepInformation(vtkInformation *info);
};

#endif // VTKPCAPIMAGEREADER_H
