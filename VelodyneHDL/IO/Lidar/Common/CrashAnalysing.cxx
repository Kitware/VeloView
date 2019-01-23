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
#include "CrashAnalysing.h"

// STD
#include <ctime>
#include <fstream>
#include <sstream>
#include <stdio.h>

// VTK
#include <vtkInformation.h>

//-----------------------------------------------------------------------------
void CrashAnalysisWriter::AddPacket(const std::string& packet)
{
  // The idea is to store 2 .pcap files. One corresponding
  // to the last N packets received and one corresponding to
  // the N older ones. Switching from one file to the other
  // insure that we always have at least the N last packets
  // (except for the very beginning).
  // When switching to file 0 or 1, the existing content of the
  // file is discarded resulting in a storage of a number of packets
  // between N and 2*N
  if (this->PacketCount % this->NbrPacketsToStore == 0)
  {
    // Close the writer if it is already opened
    if (this->Writer.IsOpen())
    {
      this->Writer.Close();
    }

    // Switch the filename and open the writer
    this->FileToStore++;
    unsigned int nbrFile = this->FileToStore % 2; // file name: 0 or 1
    std::stringstream ss;
    ss << this->Filename << nbrFile << ".bin";
    if (!this->Writer.Open(ss.str()))
    {
      vtkGenericWarningMacro("Crash analysis failed to open the log file.");
      return;
    }
  }

  this->WriteLastPacket(packet);
  this->PacketCount++;
}

//-----------------------------------------------------------------------------
void CrashAnalysisWriter::WriteLastPacket(const std::string& packet)
{
  // check that the writer is opened
  if (!this->Writer.IsOpen())
  {
    vtkGenericWarningMacro("Crash analysis couldn't save last packet received since the writer is not opened");
    return;
  }

  this->Writer.WritePacket(
          reinterpret_cast<const unsigned char*>(packet.c_str()), packet.length());
}

//-----------------------------------------------------------------------------
void CrashAnalysisWriter::CloseAnalyzer()
{
  if (this->Writer.IsOpen())
  {
    this->Writer.Close();
    return;
  }
}

//-----------------------------------------------------------------------------
void CrashAnalysisWriter::DeleteLogFiles()
{
  std::remove((this->Filename + "0.bin").c_str());
  std::remove((this->Filename + "1.bin").c_str());
}

//-----------------------------------------------------------------------------
void CrashAnalysisWriter::ArchivePreviousLogIfExist()
{
  // check if the file exists
  std::ifstream file1((this->Filename + "0.bin").c_str());
  std::ifstream file2((this->Filename + "1.bin").c_str());

  // Get the date and time
  std::time_t rawtime;
  struct std::tm * timeinfo;
  char buffer[80];
  std::time(&rawtime);
  timeinfo = std::localtime(&rawtime);
  std::strftime(buffer, sizeof(buffer), "%d_%m_%Y_%H_%M_%S", timeinfo);
  std::string timeStr(buffer);

  bool fileHasBeenRenamed = false;
  // The file exists, rename it
  if (file1.good())
  {
    file1.close();
    std::rename((this->Filename + "0.bin").c_str(), (this->Filename + timeStr + "_0.bin").c_str());
    fileHasBeenRenamed = true;
  }
  if (file2.good())
  {
    file2.close();
    std::rename((this->Filename + "1.bin").c_str(), (this->Filename + timeStr + "_1.bin").c_str());
    fileHasBeenRenamed = true;
  }

  if (fileHasBeenRenamed)
  {
    vtkGenericWarningMacro("We found log files in folder: " << this->Filename
                           << " which may be due to " << SOFTWARE_NAME << " previous crash. "
                           << "The log files have been renamed using timestamp: " << timeStr);
  }
}