#include "NetworkPacket.h"
#include <cstring>
#include <iostream>

#ifdef _MSC_VER
#include <windows.h>

namespace
{
//------------------------------------------------------------------------------
int gettimeofday(struct timeval* tp, void*)
{
  FILETIME ft;
  ::GetSystemTimeAsFileTime(&ft);
  long long t = (static_cast<long long>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
  t -= 116444736000000000LL;
  t /= 10; // microseconds
  tp->tv_sec = static_cast<long>(t / 1000000UL);
  tp->tv_usec = static_cast<long>(t % 1000000UL);
  return 0;
}
}

#endif

// in network (big endian) order
const unsigned char NetworkPacket::EthIP4UDPHeaderDefault[42] = {
  // 14 bytes ethernet header
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // dst MAC addr
  0x60, 0x76, 0x88, 0x00, 0x00, 0x00, // src MAC addr
  0x08, 0x00, // packet type (IP)
  // 20 bytes IPv4 header
  0x45, 0x00, // version+IHL, services
  0x04, 0xd2, // totlen
  0x00, 0x00, // iden
  0x40, 0x00, // flag (40=do not fragment), frag offset
  0xff, 0x11, // TTL, protocol (UDP)
  0xb4, 0xaa, // checksum
  0xc0, 0xa8, 0x01, 0xc8, // src ip: 192.168.1.200
  0xff, 0xff, 0xff, 0xff, // dest ip: broadcast
  // 8 bytes UDP header
  // src port, dst port , len, chksum
  0x09, 0x40, // src port: 2368
  0x09, 0x40, // dst port: 2368
  0x04, 0xbe, // len
  0x00, 0x00 // checksum
};

//------------------------------------------------------------------------------
NetworkPacket* NetworkPacket::BuildEthernetIP4UDP(const unsigned char* payload,
                                                 unsigned int payloadSize,
                                                 const unsigned char* sourceIPv4BigEndian,
                                                 unsigned short sourcePort,
                                                 unsigned short destinationPort)
{
  NetworkPacket* packet = new NetworkPacket();
  gettimeofday(&packet->ReceptionTime, nullptr);
  packet->PacketSize = sizeof(NetworkPacket::EthIP4UDPHeaderDefault) + payloadSize;
  packet->PacketData.resize(packet->PacketSize);

  packet->PayloadStart = sizeof(NetworkPacket::EthIP4UDPHeaderDefault);

  std::copy(NetworkPacket::EthIP4UDPHeaderDefault,
            NetworkPacket::EthIP4UDPHeaderDefault + sizeof(NetworkPacket::EthIP4UDPHeaderDefault),
            packet->PacketData.begin());
  std::copy(payload,
            payload + payloadSize,
            packet->PacketData.begin() + sizeof(NetworkPacket::EthIP4UDPHeaderDefault));

  // Set IP-frame length (which is payloadSize + 28), in network order: big endian
  packet->PacketData[EthIPUDPHeader_IPFRAMELEN] = ((payloadSize + 28) & 0xFF00) >> 8;
  packet->PacketData[EthIPUDPHeader_IPFRAMELEN + 1] = ((payloadSize + 28) & 0x00FF) >> 0;

  // Set UDP-frame length (which is payloadSize + 8), in network order: big endian
  packet->PacketData[EthIPUDPHeader_UDPFRAMELEN] = ((payloadSize + 8) & 0xFF00) >> 8;
  packet->PacketData[EthIPUDPHeader_UDPFRAMELEN + 1] = ((payloadSize + 8) & 0x00FF) >> 0;

  // Set IPv4 of source, from big endian to big endian
  std::copy(sourceIPv4BigEndian,
            sourceIPv4BigEndian + 4,
            packet->PacketData.begin() + EthIPUDPHeader_SOURCEIP4);

  // Set source port, in network order: big endian
  packet->PacketData[EthIPUDPHeader_SOURCEPORT] = (sourcePort & 0xFF00) >> 8;
  packet->PacketData[EthIPUDPHeader_SOURCEPORT + 1] = (sourcePort & 0x00FF) >> 0;

  // Set destination port, in network order: big endian
  packet->PacketData[EthIPUDPHeader_DESTPORT] = (destinationPort & 0xFF00) >> 8;
  packet->PacketData[EthIPUDPHeader_DESTPORT + 1] = (destinationPort & 0x00FF) >> 0;

  // We could compute and set the UDP checksum, then the IP checksum but this
  // was not done in the past.

  return packet;
}

//------------------------------------------------------------------------------
const unsigned char* NetworkPacket::GetPacketData() const
{
  return this->PacketData.data();
}

//------------------------------------------------------------------------------
unsigned int NetworkPacket::GetPacketSize() const
{
  return this->PacketSize;
}

//------------------------------------------------------------------------------
const unsigned char* NetworkPacket::GetPayloadData() const
{
  return this->PacketData.data() + this->PayloadStart;
}

//------------------------------------------------------------------------------
unsigned int NetworkPacket::GetPayloadSize() const
{
  // last - first + 1
  return (this->PacketSize - 1) - this->PayloadStart + 1;
}
