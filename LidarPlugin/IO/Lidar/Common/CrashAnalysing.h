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

#ifndef CRASH_ANALYSING_H
#define CRASH_ANALYSING_H

// LOCAL
#include "vtkPacketFileWriter.h"

// STD
#include <string>

/**
 * \class CrashAnalysisWriter
 * \brief This class is responsible to save as a .pcap file
 *        the last N packets received and remove the older ones.
 *        The idea is to generate a small .pcap to analyze when
 *        the software crashes in streaming mode
*/
class CrashAnalysisWriter
{
public:
  // Default constructor
  CrashAnalysisWriter() {this->PacketCount = 0;}

  // Setters
  void SetNbrPacketsToStore(unsigned int arg) {this->NbrPacketsToStore = arg;}
  void SetFilename(const std::string& arg) {this->Filename = arg;}

  // Add a packet to the crash analyzer
  void AddPacket(const std::string& packet);

  // Close the pcap writer if it is opened
  void CloseAnalyzer();

  // Delete the logs files
  void DeleteLogFiles();

  // If a previous log exists it means that the
  // stream has been quit unproperly and has potentialy
  // crashed. We will archieve the previous log in this
  // case
  void ArchivePreviousLogIfExist();

private:
  // Export file information
  unsigned int NbrPacketsToStore = 5000;
  std::string Filename = "";

  // Writer
  vtkPacketFileWriter Writer;

  // Internal parameters
  unsigned int PacketCount = 0;
  unsigned int FileToStore = 0;

  void WriteLastPacket(const std::string& packet);
};

#endif // CRASH_ANALYSING_H