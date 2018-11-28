// Copyright 2013 Velodyne Acoustics, Inc.
// Copyright 2018 Kitware SAS.
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

#ifndef VTKLIDARREADER_H
#define VTKLIDARREADER_H

#include "vtkLidarProvider.h"

//! @todo a decition should be made if the opening/closing of the pcap should be handle by
//! the class itself of the class user. Currently this is not clear

class vtkLidarReaderInternal;

class VTK_EXPORT vtkLidarReader : public vtkLidarProvider
{
public:
  static vtkLidarReader* New();
  vtkTypeMacro(vtkLidarReader, vtkLidarProvider)
  void PrintSelf(ostream& os, vtkIndent indent);

  /**
   * @copydoc vtkLidarReaderInternal::FileName
   */
  std::string GetFileName();
  virtual void SetFileName(const std::string& filename);

  int GetNumberOfFrames() override;


  /**
   * @brief GetFrame returns the requested frame
   * @param frameNumber beteween 0 and vtkLidarReader::GetNumberOfFrames()
   */
  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber);

  /**
   * @copydoc vtkLidarReaderInternal::Open()
   * @todo a decition should be made if the opening/closing of the pcap should be handle by
   * the class itself of the class user. Currently this is not clear
   */
  virtual void Open();

  /**
   * @copydoc vtkLidarReaderInternal::Close()
   * @todo a decition should be made if the opening/closing of the pcap should be handle by
   * the class itself of the class user. Currently this is not clear
   */
  virtual void Close();

  /**
   * @brief SaveFrame save the packet corresponding to the desired frames in a pcap file.
   * Because we are saving network packet, part of previous and/or next frames could be included in generated the pcap
   * @param startFrame first frame to record
   * @param endFrame last frame to record, this frame is included
   * @param filename where to save the generate pcap file
   */
  virtual void SaveFrame(int startFrame, int endFrame, const std::string& filename);

protected:
  vtkLidarReader();
  ~vtkLidarReader();

  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector) override;

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;


private:
  vtkLidarReaderInternal* Internal;
  vtkLidarReader(const vtkLidarReader&) = delete;
  void operator=(const vtkLidarReader&) = delete;
};

#endif // VTKLIDARREADER_H
