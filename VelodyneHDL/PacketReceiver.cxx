#include "PacketReceiver.h"
#include "NetworkSource.h"

#include <vtkMath.h>


//-----------------------------------------------------------------------------
PacketReceiver::PacketReceiver(boost::asio::io_service &io, int port, int forwardport, std::string forwarddestinationIp, bool isforwarding, NetworkSource *parent)
  : Port(port)
  , PacketCounter(0)
  , isForwarding(isforwarding)
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
  if (this->fileCrashAnalysis.is_open())
  {
    this->fileCrashAnalysis.close();
  }
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
void PacketReceiver::EnableCrashAnalysing(std::string filenameCrashAnalysis_, int bytesPerPacket_, bool isCrashAnalysing_)
{
  this->filenameCrashAnalysis = filenameCrashAnalysis_;
  this->bytesPerPacket = bytesPerPacket_;
  this->isCrashAnalysing = isCrashAnalysing_;
  // Opening crash analysis file
  if (isCrashAnalysing)
  {
    this->fileCrashAnalysis.open(filenameCrashAnalysis.c_str(), std::ios::out | std::ios::binary);
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
    size_t bytesSent =
      ForwardedSocket.send_to(boost::asio::buffer(packet->c_str(), numberOfBytes), ForwardEndpoint);
  }
  if (this->fileCrashAnalysis.is_open())
  {
    boost::unique_lock<boost::mutex> scoped_lock(this->IsWriting);
    this->fileCrashAnalysis.write(packet->c_str(), this->bytesPerPacket);
    this->fileCrashAnalysis.flush();
    std::streampos pos =
      static_cast<std::streampos>((this->PacketCounter % NBR_PACKETS_SAVED) * this->bytesPerPacket);
    this->fileCrashAnalysis.seekp(pos);
  }

  this->Parent->QueuePackets(packet);

  this->StartReceive();

  if ((++this->PacketCounter % 5000) == 0)
  {
    std::cout << "RECV packets: " << this->PacketCounter << " on " << this->Port << std::endl;
    ;
  }
}

