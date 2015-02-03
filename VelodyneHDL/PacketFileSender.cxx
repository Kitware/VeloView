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

#include <boost/thread/thread.hpp>

int main(int argc, char* argv[])
{

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <packet file> [loop] [ip]" << std::endl;
    return 1;
  }
  std::string filename(argv[1]);

  int loop = 0;
  std::string destinationIp = "127.0.0.1";
  if(argc > 2)
    {
    loop = atoi(argv[2]);
    }
  if(argc > 3)
    {
    destinationIp = argv[3];
    }

  try
    {
    int dataPort = 2368;
    int positionPort = 8308;
    do
      {
      vvPacketSender sender(filename, destinationIp, dataPort, positionPort);
      //socket.connect(destinationEndpoint);

      while (!sender.done())
        {
        sender.pumpPacket();
        if ((sender.packetCount() % 500) == 0)
          {
          printf("total sent packets: %lu\n", sender.packetCount());
          }

        boost::this_thread::sleep(boost::posix_time::microseconds(200));
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
