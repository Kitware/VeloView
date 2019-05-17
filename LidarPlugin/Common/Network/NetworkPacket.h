//=========================================================================
// Copyright 2019 Kitware, Inc.
// Author: Gabriel Devillers
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

#ifndef NETWORKPACKET_H
#define NETWORKPACKET_H

#ifdef _MSC_VER
#include <winsock2.h>
#else
#include <sys/time.h>
#endif
#include <vector>

class NetworkPacket
{
public:
  static NetworkPacket* BuildEthernetIP4UDP(const unsigned char* payload,
                                           unsigned int payloadSize,
                                           const unsigned char* sourceIPv4BigEndian,
                                           unsigned short sourcePort,
                                           unsigned short destinationPort);

  // Note that the memory zone returned by P->GetPayloadData() has a lifetime equal
  // to the instance P of NetworkPacket (no copy is done), and must not be freed
  // by the user of NetworkPacket
  const unsigned char* GetPacketData() const;
  unsigned int GetPacketSize() const;
  const unsigned char* GetPayloadData() const;
  unsigned int GetPayloadSize() const;
  struct timeval ReceptionTime;

  // useful offsets in packet header:
  static const unsigned int EthIPUDPHeader_IPFRAMELEN = 16;
  static const unsigned int EthIPUDPHeader_SOURCEIP4 = 26;
  static const unsigned int EthIPUDPHeader_SOURCEPORT = 34;
  static const unsigned int EthIPUDPHeader_UDPFRAMELEN = 38;
  static const unsigned int EthIPUDPHeader_DESTPORT = 36;
private:
  NetworkPacket() = default; // prevent construction without using a static constructor
  unsigned int PacketSize = 0;
  unsigned int PayloadStart = 0;
  // should you want to switch PacketData to a C array,
  // ensure care that the copy constructor copies it.
  std::vector<unsigned char> PacketData;
  // default header used only by BuildEthernetIP4UDP, inside of which we
  // override some fields when they are available
  static const unsigned char EthIP4UDPHeaderDefault[42];
};


#endif // NETWORKPACKET_H
