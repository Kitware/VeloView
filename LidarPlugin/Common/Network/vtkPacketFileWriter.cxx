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

#include "vtkPacketFileWriter.h"

#include <cstring>

//--------------------------------------------------------------------------------
vtkPacketFileWriter::vtkPacketFileWriter()
{
  this->PCAPFile = 0;
  this->PCAPDump = 0;
}

//--------------------------------------------------------------------------------
vtkPacketFileWriter::~vtkPacketFileWriter()
{
  this->Close();
}

//--------------------------------------------------------------------------------
bool vtkPacketFileWriter::Open(const std::string& filename)
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

//--------------------------------------------------------------------------------
bool vtkPacketFileWriter::IsOpen()
{
  return (this->PCAPFile != 0);
}

void vtkPacketFileWriter::Close()
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

//--------------------------------------------------------------------------------
const std::string& vtkPacketFileWriter::GetLastError()
{
  return this->LastError;
}

//--------------------------------------------------------------------------------
const std::string& vtkPacketFileWriter::GetFileName()
{
  return this->FileName;
}

//--------------------------------------------------------------------------------
// Write an UDP packet from the data (without providing a header, so we construct it)
bool vtkPacketFileWriter::WritePacket(const NetworkPacket& packet)
{
  if (!this->PCAPFile)
  {
    return false;
  }

  struct pcap_pkthdr header;
  header.caplen = packet.GetPacketSize();
  header.len = packet.GetPacketSize();
  header.ts = packet.ReceptionTime;

  pcap_dump((u_char*)this->PCAPDump, &header, packet.GetPacketData());
  return true;
}

//--------------------------------------------------------------------------------
// Write an packet from packetHeader and packetData (which includes the packet header)
bool vtkPacketFileWriter::WritePacket(pcap_pkthdr* packetHeader, unsigned char* packetData)
{
  pcap_dump((u_char*)this->PCAPDump, packetHeader, packetData);
  return true;
}
