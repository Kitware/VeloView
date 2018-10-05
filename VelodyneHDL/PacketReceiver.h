#ifndef PACKETRECEIVER_H
#define PACKETRECEIVER_H

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>

class NetworkSource;

/*!< Size of the buffer used to store the data received */
#define BUFFER_SIZE 1500

/*!< Number of packed save when the option CrashAnalysing is set */
#define NBR_PACKETS_SAVED  1500

/**
 * \class PacketReceiver
 * \brief This classs is reponsbale for listening on a socket and each time a packet is received,
 * it will enqueue the packet on a specific Queue. Here it is used to setup an UDP multicast protocol.
 * The packet receiver can forward the received packets and/or store them in a output bin file
*/
class PacketReceiver
{
public:
  /**
   * @brief PacketReceiver
   * @param io The in/out service used to handle the reception of the packets
   * @param port The port address which will receive the packet
   * @param forwardport The port adress which will receive the forwarded packets
   * @param forwarddestinationIp The IP adress of the computer which will receive the forwarded packets
   * @param isforwarding Allow or not the forwarding of the packets
   * @param parent @todo to replace by a synchronizedQueue
   */
  PacketReceiver(boost::asio::io_service& io, int port, int forwardport,
    std::string forwarddestinationIp, bool isforwarding, NetworkSource* parent);

  ~PacketReceiver();

  /**
   * @brief StartReceive bind the callback function to the socket and specify the buffer used
   * to receive the data
   */
  void StartReceive();

  /**
   * @brief EnableCrashAnalysing
   * @param filenameCrashAnalysis_ the name of the output file
   * @param bytesPerPacket_ the number of bytes per packets
   * @param isCrashAnalysing_ Enable the crash analysing
   */
  void EnableCrashAnalysing(std::string filenameCrashAnalysis_, int bytesPerPacket_, bool isCrashAnalysing_);

  void SocketCallback(const boost::system::error_code& error, std::size_t numberOfBytes);

private:
  /*!< Allow or not the forwarding of the packets */
  bool isForwarding;

  /*!< EndPoint to forward to, contain information such as forward ip, forward port */
  boost::asio::ip::udp::endpoint ForwardEndpoint;
  
  /*!< Port address which will receive the packet */
  int Port;                
  
  /*!< Number of packets received */
  int PacketCounter;

  /*!< Socket : determines the protocol used and the address used for the reception of the packets */
  boost::asio::ip::udp::socket Socket;

  /*!< Socket : determines the protocol used and the address used for the reception of the forwarded packets */
  boost::asio::ip::udp::socket ForwardedSocket;

  /*!< Network Shouce where the packet will be enqueue */
  NetworkSource* Parent;

  /*!< Buffer which will saved the data. Expecting exactly 1206 bytes, using a larger buffer
   *  so that if a larger packet arrives unexpectedly we'll notice it. */
  char RXBuffer[BUFFER_SIZE];

  bool IsReceiving; /*!< Flag indicating if the socket is receiving packets */
  bool ShouldStop;  /*!< Flag indicating if we should stop the listening */
  boost::mutex IsReceivingMtx; /*!< Mutex : Block the access of IsReceiving when a thread is seting the flag */
  boost::condition_variable IsReceivingCond;
  std::string filenameCrashAnalysis; /*!< Flag indicating if we should stop the listening */
  std::ofstream fileCrashAnalysis;
  boost::mutex IsWriting;
  int bytesPerPacket;
  bool isCrashAnalysing;
};

#endif // PACKETRECEIVER_H
