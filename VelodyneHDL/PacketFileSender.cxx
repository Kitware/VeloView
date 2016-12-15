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

#include <string>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <ctime>
#include <thread>
#include <boost/thread/thread.hpp>

int main(int argc, char* argv[])
{

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <packet file> [loop] [ip] [dataPort] [position Port] [PlaybackSpeedMultiplier]" << std::endl;
    return 1;
  }
  std::string filename(argv[1]);

  int loop = 0;
  double speed = 1;
  std::string destinationIp = "127.0.0.1";
  int dataPort=2368;
  int positionPort=8308;
  if(argc > 2)
    {
    loop = atoi(argv[2]);
    }
  if(argc > 3)
    {
    destinationIp = argv[3];
    }
  if(argc>5)
    {
      dataPort=atoi(argv[4]);
      positionPort=atoi(argv[5]);
    }
  if(argc>6)
    {
      speed = static_cast<double>(atof(argv[6]));
    }

  //The sensor send one packet every 553 microseconds
  int timeToWait = static_cast<int>(1 / speed * 553);

  //The timers resolution is only 1000 microsecond
  //But we need to send packets every 553 microseconds
  //Thus, we send packet by group of 100 and then wait
  //55 300 microseconds
  int NumberPacketsByPool = 20;

  //Calibration of the sleep
  std::chrono::time_point<std::chrono::system_clock> T1, T2;
  std::chrono::duration<double> elapsedTime;
  double timeMicroSecond = 0;

  std::cout << "Start sending" << std::endl;
  try
    {
    do
      {
      vvPacketSender sender(filename, destinationIp, dataPort, positionPort);
      //socket.connect(destinationEndpoint);
      bool isFirstPacket = true;
      double currentTimeStamp = 0;
      double previousTimeStamp = 0;
      while (!sender.done())
        {
        sender.pumpPacket();

        if ((sender.packetCount() % 1000) == 0)
          {
          T2 = std::chrono::high_resolution_clock::now();
          elapsedTime = T2 - T1;
          std::cout << "total sent packets : " << sender.packetCount();
          std::cout <<" Elapsed time per packets : " << elapsedTime.count() * 1e6 / 1000 << " microsecond" << std::endl;
          T1 = std::chrono::high_resolution_clock::now();
          //printf("total sent packets: %lu\t, time between packets : %f\n", sender.packetCount(),timeToWait/1e6);
          }
        //boost::this_thread::sleep(boost::posix_time::microseconds(553));
        if ((sender.packetCount() % NumberPacketsByPool) == 0)
          {
            std::this_thread::sleep_for(std::chrono::microseconds(NumberPacketsByPool * timeToWait));
          }
        }
      } while(loop);
    }
  catch( std::exception & e )
    {
    std::cout << "Caught Exception: " << e.what() << std::endl;
    return 1;
    }

  return 0;
}