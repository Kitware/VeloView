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

//-----------------------------------------------------------------------------
vvPacketSender::vvPacketSender(
  std::string pcapfile, std::string destinationIp, int lidarPort, int positionPort)
  : LIDARSocket(0)
  , LIDAREndpoint(boost::asio::ip::address_v4::from_string(destinationIp), lidarPort)
  , PositionSocket(0)
  , PositionEndpoint(boost::asio::ip::address_v4::from_string(destinationIp), positionPort)
  , PacketReader(0)
  , Done(false)
  , PacketCount(0)
{
  this->PacketReader = new vtkPacketFileReader;
  this->PacketReader->Open(pcapfile);
  if (!this->PacketReader->IsOpen())
  {
    throw std::runtime_error("Unable to open packet file");
  }

  this->LIDARSocket = new boost::asio::ip::udp::socket(this->IOService);
  this->LIDARSocket->open(this->LIDAREndpoint.protocol());
  this->LIDARSocket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
  // Allow to send the packet on the same machine
  this->LIDARSocket->set_option(boost::asio::ip::multicast::enable_loopback(true));

  this->PositionSocket = new boost::asio::ip::udp::socket(this->IOService);
  this->PositionSocket->open(this->PositionEndpoint.protocol());
  this->PositionSocket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
  // Allow to send the packet on the same machine
  this->PositionSocket->set_option(boost::asio::ip::multicast::enable_loopback(true));
}

//-----------------------------------------------------------------------------
vvPacketSender::~vvPacketSender()
{
  delete this->LIDARSocket;
  delete this->PositionSocket;
}

//-----------------------------------------------------------------------------
double vvPacketSender::pumpPacket()
{
  if (this->Done)
  {
    return std::numeric_limits<int>::max();
  }

  // some return value
  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  // Get the next packet
  if (!this->PacketReader->NextPacket(data, dataLength, timeSinceStart))
  {
    this->Done = true;
    return timeSinceStart;
  }

  // Position packet
  if ((dataLength == 512))
  {
    this->PositionSocket->send_to(
      boost::asio::buffer(data, dataLength), this->PositionEndpoint);
  }
  else // Lidar packet
  {
    this->LIDARSocket->send_to(
      boost::asio::buffer(data, dataLength), this->LIDAREndpoint);
  }
  this->PacketCount++;
  return timeSinceStart;
}

//-----------------------------------------------------------------------------
size_t vvPacketSender::GetPacketCount() const
{
  return this->PacketCount;
}

//-----------------------------------------------------------------------------
bool vvPacketSender::IsDone() const
{
  return this->Done;
}
