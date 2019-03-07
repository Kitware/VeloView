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

class vtkPacketFileReader;
struct FramePosition;
//! @todo a decition should be made if the opening/closing of the pcap should be handle by
//! the class itself of the class user. Currently this is not clear

class vtkLidarReaderInternal;

class VTK_EXPORT vtkLidarReader : public vtkLidarProvider
{
public:
  static vtkLidarReader* New();
  vtkTypeMacro(vtkLidarReader, vtkLidarProvider)

  int GetNumberOfFrames() override { return this->FilePositions.size(); }

  /**
   * @copydoc FileName
   */
  vtkGetMacro(FileName, std::string)
  virtual void SetFileName(const std::string& filename);


  /**
   * @brief GetFrame returns the requested frame
   * @param frameNumber beteween 0 and vtkLidarReader::GetNumberOfFrames()
   */
  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber);

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

  /**
   * @brief SaveFrame save the packet corresponding to the desired frames in a pcap file.
   * Because we are saving network packet, part of previous and/or next frames could be included in generated the pcap
   * @param startFrame first frame to record
   * @param endFrame last frame to record, this frame is included
   * @param filename where to save the generate pcap file
   */
  virtual void SaveFrame(int startFrame, int endFrame, const std::string& filename);

  vtkGetMacro(ShowFirstAndLastFrame, bool)
  vtkSetMacro(ShowFirstAndLastFrame, bool)

protected:
  vtkLidarReader() = default;
  ~vtkLidarReader() = default;

  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector) override;

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  //! Name of the pcap file to read
  std::string FileName = "";

  //! frame index which enable to jump quickly to a given frame
  std::vector<FramePosition> FilePositions;

  //! Show/Hide the first and last frame that most of the time are partial frames
  bool ShowFirstAndLastFrame = false;

  //! libpcap wrapped reader which enable to get the raw pcap packet from the pcap file
  vtkPacketFileReader* Reader = nullptr;

private:
  /**
   * @brief ReadFrameInformation read the whole pcap and create a frame index.
   * In case the calibration is contained in the pcap file, this will also read it
   */
  int ReadFrameInformation();
  /**
   * @brief SetTimestepInformation Set the timestep available
   * @param info
   */
  void SetTimestepInformation(vtkInformation *info);

  vtkLidarReader(const vtkLidarReader&) = delete;
  void operator=(const vtkLidarReader&) = delete;
};

//-----------------------------------------------------------------------------
// Internal structure
//-----------------------------------------------------------------------------
typedef struct FramePosition
{
  FramePosition(const fpos_t pos, const int skip, const double time)
    : Position(pos), Skip(skip), Time(time) {}

  //! position of the first packet of the given frame
  fpos_t Position;
  //! Offset specific to the lidar data format
  //! Used as some frame start at the middle of a packet
  int Skip;
  //! To be agnostic to the underlying data, we rely on the first packet timestep to determine
  //! the Time of frame. The packet timestep has no relation with the timesteps that are in the
  //! payload of the packet. It's contained in the header, and indicate when a packet has been
  //! received
  double Time;
} FramePosition;

#endif // VTKLIDARREADER_H
