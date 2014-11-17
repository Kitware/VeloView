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


//-----------------------------------------------------------------------------
class vvPacketSender::vvInternal
{
public:
  vvInternal(std::string destinationIp,
             int port) : Socket(0),
                         PacketReader(0),
                         Done(false),
                         PacketCount(0),
                         Endpoint(boost::asio::ip::address_v4::from_string(destinationIp), port)
  {
  }

  boost::asio::ip::udp::socket* Socket;
  vtkPacketFileReader* PacketReader;
  bool Done;
  size_t PacketCount;
  boost::asio::ip::udp::endpoint Endpoint;

  };

//-----------------------------------------------------------------------------
vvPacketSender::vvPacketSender(std::string pcapfile,
                               std::string destinationIp,
                               int port) : Internal(new vvPacketSender::vvInternal(destinationIp, port))
{
  this->Internal->PacketReader = new vtkPacketFileReader;
  this->Internal->PacketReader->Open(pcapfile);
  if(!this->Internal->PacketReader->IsOpen())
    {
    throw std::runtime_error("Unable to open packet file");
    }

  boost::asio::io_service ioService;
  this->Internal->Socket = new boost::asio::ip::udp::socket(ioService);
  this->Internal->Socket->open(this->Internal->Endpoint.protocol());
}

//-----------------------------------------------------------------------------
vvPacketSender::~vvPacketSender()
{

  delete this->Internal->Socket;
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vvPacketSender::pumpPacket()
{
  if(this->Internal->Done)
    {
    return;
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;
  if (!this->Internal->PacketReader->NextPacket(data, dataLength, timeSinceStart))
    {
    this->Internal->Done = true;
    return;
    }

  // Recurse until we get to the right kind of packet
  if (dataLength != 1206)
    {
    return;
    }

  ++this->Internal->PacketCount;
  size_t bytesSent = this->Internal->Socket->send_to(boost::asio::buffer(data, dataLength),
                                                     this->Internal->Endpoint);
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
