// Copyright 2013 Velodyne Acoustics, Inc.
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

#include "vvPacketSender.h"
#include "vtkPacketFileReader.h"

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

const int HDL_NUM_ROT_ANGLES = 36001;
const int HDL_LASER_PER_FIRING = 32;
const int HDL_MAX_NUM_LASERS = 64;
const int HDL_FIRING_PER_PKT = 12;

enum HDLBlock
{
  BLOCK_0_TO_31 = 0xeeff,
  BLOCK_32_TO_63 = 0xddff
};

#pragma pack(push, 1)
typedef struct HDLLaserReturn
{
  unsigned short distance;
  unsigned char intensity;
} HDLLaserReturn;

struct HDLFiringData
{
  unsigned short blockIdentifier;
  unsigned short rotationalPosition;
  HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};

struct HDLDataPacket
{
  HDLFiringData firingData[HDL_FIRING_PER_PKT];
  unsigned int gpsTimestamp;
  unsigned char blank1;
  unsigned char blank2;
};

struct HDLLaserCorrection
{
  double azimuthCorrection;
  double verticalCorrection;
  double distanceCorrection;
  double verticalOffsetCorrection;
  double horizontalOffsetCorrection;
  double sinVertCorrection;
  double cosVertCorrection;
  double sinVertOffsetCorrection;
  double cosVertOffsetCorrection;
};

struct HDLRGB
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
#pragma pack(pop)


//-----------------------------------------------------------------------------
class vvPacketSender::vvInternal
{
public:
  vvInternal(std::string destinationIp,
             int lidarPort,
             int positionPort) :
    LIDARSocket(0),
    PositionSocket(0),
    PacketReader(0),
    Done(false),
    PacketCount(0),
    lastTimestamp(0),
    LIDAREndpoint(boost::asio::ip::address_v4::from_string(destinationIp), lidarPort),
    PositionEndpoint(boost::asio::ip::address_v4::from_string(destinationIp), positionPort)
  {
  }

  boost::asio::ip::udp::socket* LIDARSocket;
  boost::asio::ip::udp::socket* PositionSocket;

  vtkPacketFileReader* PacketReader;
  bool Done;
  unsigned int lastTimestamp;
  size_t PacketCount;
  boost::asio::ip::udp::endpoint LIDAREndpoint;
  boost::asio::ip::udp::endpoint PositionEndpoint;
  boost::asio::io_service IOService;
  };

//-----------------------------------------------------------------------------
vvPacketSender::vvPacketSender(std::string pcapfile,
                               std::string destinationIp,
                               int lidarPort,
                               int positionPort) :
  Internal(new vvPacketSender::vvInternal(destinationIp, lidarPort, positionPort))
{
  this->Internal->PacketReader = new vtkPacketFileReader;
  this->Internal->PacketReader->Open(pcapfile);
  if(!this->Internal->PacketReader->IsOpen())
    {
    throw std::runtime_error("Unable to open packet file");
    }

  this->Internal->LIDARSocket = new boost::asio::ip::udp::socket(this->Internal->IOService);
  this->Internal->LIDARSocket->open(this->Internal->LIDAREndpoint.protocol());

  this->Internal->PositionSocket = new boost::asio::ip::udp::socket(this->Internal->IOService);
  this->Internal->PositionSocket->open(this->Internal->PositionEndpoint.protocol());
}

//-----------------------------------------------------------------------------
vvPacketSender::~vvPacketSender()
{
  delete this->Internal->LIDARSocket;
  delete this->Internal->PositionSocket;
  delete this->Internal;
}

//-----------------------------------------------------------------------------
int vvPacketSender::pumpPacket()
{
  if(this->Internal->Done)
    {
    return std::numeric_limits<double>::max();
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;
  int timeDiff = 0;
  if (!this->Internal->PacketReader->NextPacket(data, dataLength, timeSinceStart))
    {
    this->Internal->Done = true;
    return timeSinceStart;
    }

  if( (dataLength == 512) )
    {
    size_t bytesSent = this->Internal->PositionSocket->send_to(boost::asio::buffer(data, dataLength),
                                                               this->Internal->PositionEndpoint);
    }

  // Recurse until we get to the right kind of packet
  else
  //if (dataLength == 1206)
    {
    const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data);
    timeDiff = static_cast<int>(dataPacket->gpsTimestamp) - static_cast<int>(this->Internal->lastTimestamp);
    this->Internal->lastTimestamp = dataPacket->gpsTimestamp;
    ++this->Internal->PacketCount;
    size_t bytesSent = this->Internal->LIDARSocket->send_to(boost::asio::buffer(data, dataLength),
                                                            this->Internal->LIDAREndpoint);
    }


  return timeDiff;
}

//-----------------------------------------------------------------------------
size_t vvPacketSender::packetCount() const
{
  return this->Internal->PacketCount;
}

//-----------------------------------------------------------------------------
bool vvPacketSender::done() const
{
  return this->Internal->Done;
}
