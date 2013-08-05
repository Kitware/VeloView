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
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPacketFileWriter.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPacketFileWriter -
// .SECTION Description
//

#ifndef __vtkPacketFileWriter_h
#define __vtkPacketFileWriter_h

#include <pcap.h>
#include <string>
#ifdef _MSC_VER
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else
# include <stdint.h>
#endif


#ifdef _MSC_VER

#include <windows.h>

namespace {
int gettimeofday(struct timeval * tp, void *)
{
  FILETIME ft;
  ::GetSystemTimeAsFileTime( &ft );
  long long t = (static_cast<long long>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
  t -= 116444736000000000LL;
  t /= 10;  // microseconds
  tp->tv_sec = static_cast<long>( t / 1000000UL);
  tp->tv_usec = static_cast<long>( t % 1000000UL);
  return 0;
}
}

#endif

class vtkPacketFileWriter
{
public:

  vtkPacketFileWriter()
  {
    this->PCAPFile = 0;
    this->PCAPDump = 0;

    this->PacketHeader.caplen = 1248;
    this->PacketHeader.len = 1248;

    // note these values are little endian, pcap wants the packet header and
    //data to be in the platform's native byte order, so assuming little endian.
    unsigned short packetHeader[21] = {
      0xffff, 0xffff, 0xffff, 0x7660,
      0x0088, 0x0000, 0x0008, 0x0045,
      0xd204, 0x0000, 0x0040, 0x11ff,
      0xaab4, 0xa8c0, 0xc801, 0xffff, // checksum 0xa9b4 //source ip 0xa8c0, 0xc801 is 192.168.1.200
      0xffff, 0x4009, 0x4009, 0xbe04, 0x0000};

    memcpy(this->PacketBuffer, packetHeader, 42);
  }

  ~vtkPacketFileWriter()
  {
    this->Close();
  }

  bool Open(const std::string& filename)
  {
    this->PCAPFile = pcap_open_dead(DLT_EN10MB, 65535);
    this->PCAPDump = pcap_dump_open(this->PCAPFile, filename.c_str());

    if (!this->PCAPDump)
      {
      this->LastError = pcap_geterr(this->PCAPFile);
      pcap_close(this->PCAPFile);
      this->PCAPFile = 0;
      return false;
      }

    this->FileName = filename;
    return true;
  }

  bool IsOpen()
  {
    return (this->PCAPFile != 0);
  }

  void Close()
  {
    if (this->PCAPFile)
      {
      pcap_dump_close(this->PCAPDump);
      pcap_close(this->PCAPFile);
      this->PCAPFile = 0;
      this->PCAPDump = 0;
      this->FileName.clear();
      }
  }

  const std::string& GetLastError()
  {
    return this->LastError;
  }

  const std::string& GetFileName()
  {
    return this->FileName;
  }

  bool WritePacket(const unsigned char* data, unsigned int dataLength)
  {
    if (!this->PCAPFile)
      {
      return false;
      }

    if (dataLength != 1206)
      {
      return false;
      }

    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    this->PacketHeader.ts = currentTime;

    memcpy(this->PacketBuffer + 42, data, dataLength);

    pcap_dump((u_char *)this->PCAPDump, &this->PacketHeader, this->PacketBuffer);
    return true;
  }

  bool WritePacket(pcap_pkthdr* packetHeader, unsigned char* packetData)
  {
    pcap_dump((u_char *)this->PCAPDump, packetHeader, packetData);
    return true;
  }

protected:



  pcap_t* PCAPFile;
  pcap_dumper_t* PCAPDump;
  struct pcap_pkthdr PacketHeader;
  unsigned char PacketBuffer[1248];

  std::string FileName;
  std::string LastError;
};

#endif
