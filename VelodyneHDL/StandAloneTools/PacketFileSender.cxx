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
  Module:    PacketFileSender.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME PacketFileSender -
// .SECTION Description
// This program reads a pcap file and sends the packets using UDP.

#include "vtkPacketFileReader.h"
#include "vvPacketSender.h"

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>

#include <boost/thread/thread.hpp>

int main(int argc, char* argv[])
{

  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0]
              << " <packet file> [loop] [ip] [dataPort] [position Port] [PlaybackSpeedMultiplier]"
              << std::endl;
    return 1;
  }
  std::string filename(argv[1]);

  int loop = 0;
  double speed = 1;
  std::string destinationIp = "127.0.0.1";
  int dataPort = 2368;
  int positionPort = 8308;
  if (argc > 2)
  {
    loop = atoi(argv[2]);
  }
  if (argc > 3)
  {
    destinationIp = argv[3];
  }
  if (argc > 5)
  {
    dataPort = atoi(argv[4]);
    positionPort = atoi(argv[5]);
  }
  if (argc > 6)
  {
    speed = static_cast<double>(atof(argv[6]));
  }

  // Default time to wait -> it is the elapsed
  // time between two firing of a HDL-32 sensor
  const int defaultTimeToWait = 553;

  // The sensor send one packet every hundreds microseconds
  // thus, the first initialization of timeToWait is defaultTimeToWait
  int timeToWaitPerPacket = static_cast<int>(1.0 / speed * defaultTimeToWait);

  // The timer's resolution is only 1000 microsecond
  // But we need to send packets every X microseconds
  // Thus, we send packet by group of NumberPacketsByPool
  // and then wait the necessary time
  int NumberPacketsByPool = 40;

  // The minimalTimeToWait resolution so that
  // timeToWait * NumberPacketsByPool > 1000
  double minimalTimeToWait = 1000.0 / NumberPacketsByPool;

  // Measurement of the sleep
  clock_t T1, T2;
  T1 = std::clock();
  double elapsedTimeMeasured;
  double elapsedTimeBetweenPackets = 0;

  std::cout << "Start sending" << std::endl;
  try
  {
    do
    {
      vvPacketSender sender(filename, destinationIp, dataPort, positionPort);
      // socket.connect(destinationEndpoint);

      sender.pumpPacket();
      while (!sender.done())
      {
        // Get the timestamp of the packet to compute
        // the next timeToWait
        int currentTimeDiff = sender.pumpPacket();

        // Elapsed time measured from the timestamp of the packets
        // This timestamp will be used to compute the next time to wait
        elapsedTimeBetweenPackets += currentTimeDiff;

        // Every 1000 packets sent, give some information
        // about the number of packet sent, the elapsed time
        // and the next time to wait for the 1000 next packets
        if ((sender.packetCount() % 1000) == 0)
        {
          // Elapsed time measured from the clock of the computer
          // This timestamp will be used to inform the user
          // This timestamp should be greater than elapsedTime2
          T2 = std::clock();
          elapsedTimeMeasured = T2 - T1;
          std::cout << "total sent packets : " << sender.packetCount() << std::endl
                    << " Elapsed time per packets asked    : " << timeToWaitPerPacket
                    << " microseconds" << std::endl
                    << " Elapsed time per packets measured : " << elapsedTimeMeasured
                    << " microseconds" << std::endl
                    << std::endl;

          timeToWaitPerPacket = static_cast<int>(1.0 / speed * elapsedTimeBetweenPackets / 1000);

          // If the computed timeToWait is too high we assume that
          // there is a corruption into the packets timestamp
          // then we set the timeToWait to the HDL-32 firing rate
          if (timeToWaitPerPacket > 2500)
          {
            timeToWaitPerPacket = static_cast<int>(1.0 / speed * defaultTimeToWait);
          }

          // If the computed timeToWait is under the minimal resolution
          // we set the timeToWait to this minimal resolution
          if (timeToWaitPerPacket < minimalTimeToWait)
          {
            timeToWaitPerPacket = static_cast<int>(minimalTimeToWait);
          }

          // Refresh the measured timestamps
          T1 = std::clock();
          elapsedTimeBetweenPackets = 0;
        }

        // boost::this_thread::sleep(boost::posix_time::microseconds(553));
        // Every NumberPacketsByPool packets we wait to be consistent with
        // the real recording of the data. The number of packets should not be
        // too small -> the clock resolution is 1000 microsecond. Hence,
        // NumberPacketsByPool * timeToWait should be greater than 1000
        if ((sender.packetCount() % NumberPacketsByPool) == 0)
        {
          boost::this_thread::sleep(
            boost::posix_time::microseconds(NumberPacketsByPool * timeToWaitPerPacket));
        }
      }
    } while (loop);
  }
  catch (std::exception& e)
  {
    std::cout << "Caught Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
