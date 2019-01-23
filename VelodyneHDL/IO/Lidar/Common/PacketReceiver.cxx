//=========================================================================
//
// Copyright 2018 Kitware, Inc.
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
#include "PacketReceiver.h"
#include "NetworkSource.h"

#include <vtkMath.h>


//-----------------------------------------------------------------------------
PacketReceiver::PacketReceiver(boost::asio::io_service &io, int port, int forwardport, std::string forwarddestinationIp, bool isforwarding, NetworkSource *parent)
  : isForwarding(isforwarding)
  , Port(port)
  , PacketCounter(0)
  , Socket(io)
  , ForwardedSocket(io)
  , Parent(parent)
  , IsReceiving(true)
  , ShouldStop(false)
{
  this->Socket.open(boost::asio::ip::udp::v4()); // Opening the socket with an UDP v4 protocol
  this->Socket.set_option(boost::asio::ip::udp::socket::reuse_address(
                      true)); // Tell the OS we accept to re-use the port address for an other app
  this->Socket.bind(boost::asio::ip::udp::endpoint(
                boost::asio::ip::udp::v4(), port)); // Bind the socket to the right address

  // Check that the provided ipadress is valid
  boost::system::error_code errCode;
  boost::asio::ip::address ipAddressForwarding = boost::asio::ip::address_v4::from_string(forwarddestinationIp, errCode);

  // If the ip address is not valid we replace it by a generic
  // 0.0.0.0 ip address. This is due to the application
  // crashes on windows if a not valid ip address is provided
  // with error message:
  // Qt has caught an exception from an event handler. This
  // is not supported in Qt.
  if (errCode)
  {
    ipAddressForwarding = boost::asio::ip::address_v4::from_string("0.0.0.0");
    if (this->isForwarding)
    {
      vtkGenericWarningMacro("Forward ip address not valid, packets won't be forwarded");
      this->isForwarding = false;
    }
  }

  this->ForwardEndpoint = boost::asio::ip::udp::endpoint(ipAddressForwarding, forwardport);
  this->ForwardedSocket.open(ForwardEndpoint.protocol()); // Opening the socket with an UDP v4 protocol
  // toward the forwarded ip address and port
  this->ForwardedSocket.set_option(boost::asio::ip::multicast::enable_loopback(
                                true)); // Allow to send the packet on the same machine
}

//-----------------------------------------------------------------------------
PacketReceiver::~PacketReceiver()
{
  this->Socket.cancel();
  this->ForwardedSocket.cancel();
  {
    boost::unique_lock<boost::mutex> guard(this->IsReceivingMtx);
    this->ShouldStop = true;
    while (this->IsReceiving)
    {
      this->IsReceivingCond.wait(guard);
    }
  }

  // Close and delete the logs files. So that,
  // if a log file is present in the next session
  // it means that the software has been closed
  // properly (potentially a crash)
  this->CrashAnalysis.CloseAnalyzer();
  this->CrashAnalysis.DeleteLogFiles();
}

//-----------------------------------------------------------------------------
void PacketReceiver::StartReceive()
{
  {
    boost::lock_guard<boost::mutex> guard(this->IsReceivingMtx);
    this->IsReceiving = true;
  }

  // expecting exactly 1206 bytes, using a larger buffer so that if a
  // larger packet arrives unexpectedly we'll notice it.
  this->Socket.async_receive(boost::asio::buffer(this->RXBuffer, BUFFER_SIZE),
                             boost::bind(&PacketReceiver::SocketCallback, this, boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

//-----------------------------------------------------------------------------
void PacketReceiver::EnableCrashAnalysing(std::string filenameCrashAnalysis_, unsigned int nbrPacketToStore_, bool isCrashAnalysing_)
{
  this->IsCrashAnalysing = isCrashAnalysing_;

  // Opening crash analysis file
  if (this->IsCrashAnalysing)
  {
    this->CrashAnalysis.SetNbrPacketsToStore(nbrPacketToStore_);
    this->CrashAnalysis.SetFilename(filenameCrashAnalysis_);
    this->CrashAnalysis.ArchivePreviousLogIfExist();
  }
}

//-----------------------------------------------------------------------------
void PacketReceiver::SocketCallback(
  const boost::system::error_code& error, std::size_t numberOfBytes)
{
  if (error || this->ShouldStop)
  {
    // This is called on cancel
    // TODO: Check other error codes
    {
      boost::lock_guard<boost::mutex> guard(this->IsReceivingMtx);
      this->IsReceiving = false;
    }
    this->IsReceivingCond.notify_one();

    return;
  }
  std::string* packet = new std::string(this->RXBuffer, numberOfBytes);

  if (this->isForwarding)
  {
    ForwardedSocket.send_to(boost::asio::buffer(packet->c_str(), numberOfBytes), ForwardEndpoint);
  }

  if (this->IsCrashAnalysing)
  {
    this->CrashAnalysis.AddPacket(*packet);
  }

  this->Parent->QueuePackets(packet);

  this->StartReceive();

  if ((++this->PacketCounter % 5000) == 0)
  {
    std::cout << "RECV packets: " << this->PacketCounter << " on " << this->Port << std::endl;
    ;
  }
}

