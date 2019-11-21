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

/*
 * Useful links to get started with IPv4 and IPv6 headers:
 * - https://en.wikipedia.org/wiki/IPv4#Packet_structure
 * - https://en.wikipedia.org/wiki/IPv6_packet
 */

// Some versions of libpcap do not have PCAP_NETMASK_UNKNOWN
#if !defined(PCAP_NETMASK_UNKNOWN)
#define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif

//------------------------------------------------------------------------------
// IPv6 fragment IDs are 4 bytes, IPv4 are only 2.
typedef uint32_t FragmentIdentificationT;

//------------------------------------------------------------------------------
// Packet fragment offsets are given in steps of 8 bytes.
constexpr unsigned int FRAGMENT_OFFSET_STEP = 8;

//------------------------------------------------------------------------------
/*!
 * @brief Track fragment reassembly to confirm that all data has been
 *        reassembled before passing it on.
 */
struct FragmentTracker
{
  // The incrementally reassembled data.
  std::vector<unsigned char> Data;
  // Set when the final fragment is encountered.
  unsigned int ExpectedSize = 0;
  // Incremented as fragments are added to the data.
  unsigned int CurrentSize = 0;
};

//------------------------------------------------------------------------------
/*!
 * @brief Fragment info required for reconstructing fragmented IP packets.
 */
struct FragmentInfo
{
  FragmentIdentificationT Identification = 0;
  uint16_t Offset = 0;
  bool MoreFragments = false;

  void Reset()
  {
    this->Identification = 0;
    this->Offset = 0;
    this->MoreFragments = false;
  }
};


namespace IPHeaderFunctions
{
  //----------------------------------------------------------------------------
  /*!
   * @brief      Inspect the IP header to get fragment information.
   * @param[in]  data         A pointer to the bytes of the IP header.
   * @param[out] fragmentInfo The collected fragment info.
   * @return     True if the information could be retrieved, false otherwise.
   */
  bool getFragmentInfo(unsigned char const * data, FragmentInfo & fragmentInfo);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Determine the IP header length by inspection.
   * @param[in] data A pointer to the first byte of the IP header.
   * @return    The number of bytes in the IP header, or 0 if this could not be
   *            determined.
   */
  unsigned int getIPHeaderLength(unsigned char const* data);
}



//------------------------------------------------------------------------------
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
    pcap_pkthdr* header;
    FragmentInfo fragmentInfo;
    fragmentInfo.MoreFragments = true;

    while (fragmentInfo.MoreFragments)
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
      IPHeaderFunctions::getFragmentInfo(tmpData + this->FrameHeaderLength, fragmentInfo);

      // Only return the payload.
      // We read the actual IP header length (v4 & v6) + assume UDP
      const unsigned int ipHeaderLength = IPHeaderFunctions::getIPHeaderLength(tmpData + this->FrameHeaderLength);
      if (ipHeaderLength == 0)
      {
        continue;
      }
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
      if (dataLength > 0 || fragmentInfo.MoreFragments || fragmentInfo.Offset > 0)
      {
        decltype(tmpDataLength) offset = fragmentInfo.Offset * FRAGMENT_OFFSET_STEP;
        decltype(tmpDataLength) requiredSize = offset + tmpDataLength;

        auto & fragmentTracker = this->Fragments[fragmentInfo.Identification];
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
        if (fragmentInfo.MoreFragments)
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
          this->AssembledId = fragmentInfo.Identification;
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
  std::unordered_map<FragmentIdentificationT, FragmentTracker> Fragments;

  //! @brief The ID of the last completed fragment.
  FragmentIdentificationT AssembledId = 0;

  //! @brief True if there is a reassembled packet to remove.
  bool RemoveAssembled = false;
};

#endif
