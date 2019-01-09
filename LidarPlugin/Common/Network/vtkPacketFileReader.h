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
  Module:    vtkPacketFileReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPacketFileReader -
// .SECTION Description
//

#ifndef __vtkPacketFileReader_h
#define __vtkPacketFileReader_h

#include <pcap.h>
#include <string>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <unordered_map>

// Some versions of libpcap do not have PCAP_NETMASK_UNKNOWN
#if !defined(PCAP_NETMASK_UNKNOWN)
#define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif

// Packet fragment offsets are given in steps of 8 bytes.
constexpr unsigned int FRAGMENT_OFFSET_STEP = 8;

//! @brief Track fragment reassembly to confirm that all data has been
//!        reassembled before passing it on.
struct FragmentTracker
{
  // The incrementally reassembled data.
  std::vector<unsigned char> Data;
  // Set when the final fragment is encountered.
  unsigned int ExpectedSize = 0;
  // Incremented as fragments are added to the data.
  unsigned int CurrentSize = 0;
};



class vtkPacketFileReader
{
public:
  vtkPacketFileReader()
  {
    this->PCAPFile = 0;
  }

  ~vtkPacketFileReader()
  {
    this->Close();
  }

  // This function is called to read a savefile .pcap
  // 1-Open a savefile in the tcpdump/libcap format to read packet
  // 2-A packet filter is then compile to convert an high level filtering
  //  expression in a program that can be interpreted by the kernel-level filtering engine
  // 3- The compiled filter is then associate to the capture
  bool Open(const std::string& filename, std::string filter_arg="udp")
  {
    char errbuff[PCAP_ERRBUF_SIZE];
    pcap_t* pcapFile = pcap_open_offline(filename.c_str(), errbuff);
    if (!pcapFile)
    {
      this->LastError = errbuff;
      return false;
    }

    bpf_program filter;

    if (pcap_compile(pcapFile, &filter, filter_arg.c_str(), 0, PCAP_NETMASK_UNKNOWN) == -1)
    {
      this->LastError = pcap_geterr(pcapFile);
      return false;
    }

    if (pcap_setfilter(pcapFile, &filter) == -1)
    {
      this->LastError = pcap_geterr(pcapFile);
      return false;
    }

    const unsigned int loopback_header_size = 4;
    const unsigned int ethernet_header_size = 14;
    auto linktype = pcap_datalink(pcapFile);
    switch (linktype)
    {
      case DLT_EN10MB:
        this->FrameHeaderLength = ethernet_header_size;
        break;
      case DLT_NULL:
        this->FrameHeaderLength = loopback_header_size;
        break;
      default:
        this->LastError = "Unknown link type in pcap file. Cannot tell where the payload is.";
        return false;
    }

    this->FileName = filename;
    this->PCAPFile = pcapFile;
    this->StartTime.tv_sec = this->StartTime.tv_usec = 0;
    return true;
  }
  bool IsOpen() { return (this->PCAPFile != 0); }

  void Close()
  {
    if (this->PCAPFile)
    {
      pcap_close(this->PCAPFile);
      this->PCAPFile = 0;
      this->FileName.clear();
    }
  }

  const std::string& GetLastError() { return this->LastError; }

  const std::string& GetFileName() { return this->FileName; }

  void GetFilePosition(fpos_t* position)
  {
#ifdef _MSC_VER
    pcap_fgetpos(this->PCAPFile, position);
#else
    FILE* f = pcap_file(this->PCAPFile);
    fgetpos(f, position);
#endif
  }

  void SetFilePosition(fpos_t* position)
{
#ifdef _MSC_VER
    pcap_fsetpos(this->PCAPFile, position);
#else
    FILE* f = pcap_file(this->PCAPFile);
    fsetpos(f, position);
#endif
  }

  bool NextPacket(const unsigned char*& data, unsigned int& dataLength, double& timeSinceStart,
    pcap_pkthdr** headerReference = NULL, unsigned int* dataHeaderLength = NULL)
  {
    if (!this->PCAPFile)
    {
      return false;
    }

    // Delete reassembled fragment data from the previous call, if any. This
    // cannot be done earlier because it would invalidate the data before it is
    // returned.
    if (this->RemoveAssembled)
    {
      auto it = this->Fragments.find(this->AssembledId);
      this->Fragments.erase(it);
      this->AssembledId = 0;
      this->RemoveAssembled = false;
    }

    dataLength = 0;
    bool moreFragments = true;
    uint16_t fragmentOffset = 0;
    pcap_pkthdr* header;

    while (moreFragments)
    {
      unsigned char const * tmpData = nullptr;
      unsigned int tmpDataLength;

      int returnValue = pcap_next_ex(this->PCAPFile, &header, &tmpData);
      if (returnValue < 0)
      {
        this->Close();
        return false;
      }

      // Collect header values before they are removed.
      uint16_t identification = 0x100 * tmpData[0x12] + tmpData[0x13];
      unsigned char const flags = tmpData[0x14] >> 4;
      moreFragments = flags & 0x2;
      fragmentOffset = (tmpData[0x14] & 0x1F) * 0x100 + tmpData[0x15];

      // Only return the payload.
      // We read the actual IP header length (v4 & v6) + assumes UDP
      const unsigned int ipHeaderLength = (tmpData[this->FrameHeaderLength + 0] & 0xf) * 4;
      const unsigned int udpHeaderLength = 8;
      const unsigned int bytesToSkip = this->FrameHeaderLength + ipHeaderLength + udpHeaderLength;

      tmpDataLength = header->len - bytesToSkip;
      if (header->len > header->caplen)
        tmpDataLength = header->caplen - bytesToSkip;
      tmpData = tmpData + bytesToSkip;
      timeSinceStart = GetElapsedTime(header->ts, this->StartTime);

      if (headerReference != NULL && dataHeaderLength != NULL)
      {
        *headerReference = header;
        *dataHeaderLength = bytesToSkip;
      }

      // pcap_next_ex may reallocate the buffers it returns so the data must be
      // copied between each call.
      if (dataLength > 0 || moreFragments || fragmentOffset > 0)
      {
        decltype(tmpDataLength) offset = fragmentOffset * FRAGMENT_OFFSET_STEP;
        unsigned requiredSize = offset + tmpDataLength;

        auto & fragmentTracker = this->Fragments[identification];
        auto & reassembledData = fragmentTracker.Data;

        if (requiredSize > reassembledData.size())
        {
          reassembledData.resize(requiredSize);
        }
        std::copy(tmpData, tmpData + tmpDataLength, reassembledData.begin() + offset);

        // Update current size, which represents the total number of bytes
        // collected in the reassembled packet.
        fragmentTracker.CurrentSize += tmpDataLength;

        // There may be gaps of size FRAGMENT_OFFSET_STEP between fragments. Add
        // this to the current size.
        //
        // Note that this assumes that <number of fragments - 1> *
        // FRAGMENT_OFFSET_STEP is never larger than any given fragment so that
        // it will not be omitted accidentally.
        if (moreFragments)
        {
          fragmentTracker.CurrentSize += FRAGMENT_OFFSET_STEP;
        }
        // Set the expected size if this is the final fragment.
        else
        {
          fragmentTracker.ExpectedSize = requiredSize;
        }


        // Return the packet if it's complete.
        if (fragmentTracker.ExpectedSize > 0 && fragmentTracker.CurrentSize >= fragmentTracker.ExpectedSize)
        {
          data = reassembledData.data();
          dataLength = reassembledData.size();
          // Delete the associated data on the next iteration.
          this->AssembledId = identification;
          this->RemoveAssembled = true;
          return true;
        }
      }
      else
      {
        data = tmpData;
        dataLength = tmpDataLength;
        return true;
      }
    }

    return true;
  }

protected:
  double GetElapsedTime(const timeval& end, const timeval& start)
  {
    return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.00;
  }

  pcap_t* PCAPFile;
  std::string FileName;
  std::string LastError;
  timeval StartTime;
  unsigned int FrameHeaderLength;


private:
  //! @brief A map of fragmented packet IDs to the collected array of fragments.
  std::unordered_map<uint16_t, FragmentTracker> Fragments;

  //! @brief The ID of the last completed fragment.
  uint16_t AssembledId = 0;

  //! @brief True if there is a reassembled packet to remove.
  bool RemoveAssembled = false;
};

#endif
