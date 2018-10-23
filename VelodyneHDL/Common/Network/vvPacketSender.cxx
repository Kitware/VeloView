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
#include "vtkDataPacket.h"
#include "vtkPacketFileReader.h"

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

using namespace DataPacketFixedLength;

//-----------------------------------------------------------------------------
class vvPacketSender::vvInternal
{
public:
  vvInternal(std::string destinationIp, int lidarPort, int positionPort)
    : LIDARSocket(0)
    , PositionSocket(0)
    , PacketReader(0)
    , Done(false)
    , lastTimestamp(0)
    , PacketCount(0)
    , LIDAREndpoint(boost::asio::ip::address_v4::from_string(destinationIp), lidarPort)
    , PositionEndpoint(boost::asio::ip::address_v4::from_string(destinationIp), positionPort)
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
vvPacketSender::vvPacketSender(
  std::string pcapfile, std::string destinationIp, int lidarPort, int positionPort)
  : Internal(new vvPacketSender::vvInternal(destinationIp, lidarPort, positionPort))
{
  this->Internal->PacketReader = new vtkPacketFileReader;
  this->Internal->PacketReader->Open(pcapfile);
  if (!this->Internal->PacketReader->IsOpen())
  {
    throw std::runtime_error("Unable to open packet file");
  }

  this->Internal->LIDARSocket = new boost::asio::ip::udp::socket(this->Internal->IOService);
  this->Internal->LIDARSocket->open(this->Internal->LIDAREndpoint.protocol());
  this->Internal->LIDARSocket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
  // Allow to send the packet on the same machine
  this->Internal->LIDARSocket->set_option(boost::asio::ip::multicast::enable_loopback(true));

  this->Internal->PositionSocket = new boost::asio::ip::udp::socket(this->Internal->IOService);
  this->Internal->PositionSocket->open(this->Internal->PositionEndpoint.protocol());
  this->Internal->PositionSocket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
  // Allow to send the packet on the same machine
  this->Internal->PositionSocket->set_option(boost::asio::ip::multicast::enable_loopback(true));
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
  if (this->Internal->Done)
  {
    return std::numeric_limits<int>::max();
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

  // Position packet
  if ((dataLength == 512))
  {
    this->Internal->PositionSocket->send_to(
      boost::asio::buffer(data, dataLength), this->Internal->PositionEndpoint);
  }

  // Recurse until we get to the right kind of packet
  else
  // if (dataLength == 1206)
  {
    const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket*>(data);
    timeDiff =
      static_cast<int>(dataPacket->gpsTimestamp) - static_cast<int>(this->Internal->lastTimestamp);
    this->Internal->lastTimestamp = dataPacket->gpsTimestamp;
    ++this->Internal->PacketCount;

    this->Internal->LIDARSocket->send_to(
      boost::asio::buffer(data, dataLength), this->Internal->LIDAREndpoint);
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
