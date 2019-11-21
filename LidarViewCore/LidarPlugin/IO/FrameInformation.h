// Copyright 2019 Kitware SAS.
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

#ifndef FRAMEINFORMATION_H
#define FRAMEINFORMATION_H

#include <memory>

/**
 * @brief SpecificFrameInformation placeholder for
 *        specific sensor implementation
 */
struct SpecificFrameInformation {
  virtual void reset() = 0;
  virtual std::unique_ptr<SpecificFrameInformation> clone() = 0;
};

/**
 * @brief FrameInformation contains information about a frame
 */
struct FrameInformation
{
  //! position of the first packet of the given frame
  fpos_t FilePosition;

  //! To be agnostic to the underlying data, we rely on the first packet timestep to determine
  //! the Time of frame. The packet timestep has no relation with the timesteps that are in the
  //! payload of the packet. It's contained in the header, and indicate when a packet has been
  //! received
  double FirstPacketNetworkTime = 0;

  //! timestamp of data contained in the first packet
  double FirstPacketDataTime = 0;

  //! Packet information that are specific to a sensor
  std::shared_ptr<SpecificFrameInformation> SpecificInformation = nullptr;

  void Reset() {
    this->FirstPacketDataTime = 0;
    this->FirstPacketNetworkTime = 0;
    this->SpecificInformation->reset();
  }

  FrameInformation() = default;

  FrameInformation(const FrameInformation& arg) { *this = arg; }

  void operator=(const FrameInformation& arg) {
    this->FilePosition = arg.FilePosition;
    this->FirstPacketNetworkTime = arg.FirstPacketNetworkTime;
    this->FirstPacketDataTime = arg.FirstPacketDataTime;
    if(arg.SpecificInformation != nullptr)
    {
      this->SpecificInformation = arg.SpecificInformation->clone();
    }
  }

};

#endif
