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
// The default playback speed is based on the timestamps specified in the pcap file

#include "vtkPacketFileReader.h"
#include "vvPacketSender.h"

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <chrono>

#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

const int OUTPUT_WIDTH = 15; // width of the column (#packet, duration, ...) in the output stream

int main(int argc, char* argv[])
{
  bool loop = false;  // run the capture 1 time or in loop

  // parse the command line options
  po::options_description visible("Allowed options");
  visible.add_options()
      ("help", "produce help message")
      ("ip", po::value<std::string>()->default_value("127.0.0.1"), "destination ip adress")
      ("loop", po::bool_switch(&loop), "run the capture in loop")
      ("lidarPort", po::value<unsigned int>()->default_value(2368), "destination port for lidar packets")
      ("GPSPort", po::value<unsigned int>()->default_value(8308), "destination port for GPS packets")
      ("speed", po::value<double>()->default_value(1), "playback speed")
      ("display-frequency", po::value<unsigned int>()->default_value(1000), "print information after every interval of X sent packets")
      ;

  po::options_description hidden("Hidden options");
  hidden.add_options()
      ("input-file", po::value<std::string>(), "input file")
      ;

  po::positional_options_description p;
  p.add("input-file", -1);

  po::options_description cmdline_options;
  cmdline_options.add(visible).add(hidden);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(cmdline_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || argc < 2) {
      cout << "Usage: PacketFileSender <pcap_file> [options]\n";
      cout << visible << "\n";
      return 1;
  }

  // convert to the right type
  std::string filename = vm["input-file"].as<std::string>();
  double speed = vm["speed"].as<double>();
  std::string destinationIp = vm["ip"].as<std::string>();
  unsigned int lidarPort =  vm["lidarPort"].as<unsigned int>();
  unsigned int GPSPort = vm["GPSPort"].as<unsigned int>();
  unsigned int display_frequency = vm["display-frequency"].as<unsigned int>();

  const int microSecondsPerSecond = 1e6;

  std::cout << "Start sending" << std::endl;

  try
  {
    do
    {
      auto replayStartTime = std::chrono::steady_clock::now();

      // output the column header for the displayed values
      std::cout << "----------------------------------------------------------------------------" << std::endl
                << std::right << std::setw(OUTPUT_WIDTH) << "# packets"
                << std::right << std::setw(OUTPUT_WIDTH) << "duration (s)"
                << std::right << std::setw(OUTPUT_WIDTH) << "f (Hz)"
                << std::right << std::setw(OUTPUT_WIDTH) << "delay (us)"
                << std::endl
                << "----------------------------------------------------------------------------" << std::endl;

      // Create a Packet Sender
      vvPacketSender sender(filename, destinationIp, lidarPort, GPSPort);

      // Case starting time
      double pcapStartTime = sender.pumpPacket();

      while (!sender.IsDone())
      {
        // time from the pcap file
        double pcapCurrentTime = sender.pumpPacket();
        double pcapmicroSecondsSinceStart = (pcapCurrentTime - pcapStartTime) * microSecondsPerSecond;

        // time from the wall clock
        auto replayCurrentTime = std::chrono::steady_clock::now();
        int replaymicroSecondsSinceStart =
            std::chrono::duration_cast<std::chrono::microseconds>(replayCurrentTime - replayStartTime).count();

        // check if the replay is to much in advance compared to the pcap time step
        // if this is the case, we sleep the thread until the pcap catch up the replay
        double time_delay = static_cast<double>(pcapmicroSecondsSinceStart) / speed  - replaymicroSecondsSinceStart;

        if (time_delay > 0)
        {
        boost::this_thread::sleep(
          boost::posix_time::microseconds(static_cast<int>(time_delay)));
        }

        // Display the user some information
        if ((sender.GetPacketCount() % display_frequency) == 0)
        {
          // Compute nb of packets sended including the one from previous loops
          int nbPacketSended = sender.GetPacketCount();

          // Compute time since the replay began
          double secondSinceStart = static_cast<double>(replaymicroSecondsSinceStart) / microSecondsPerSecond;

          // Nice output
          std::cout << std::fixed
                    << std::right << std::setw(OUTPUT_WIDTH) << nbPacketSended
                    << std::fixed << std::right << std::setw(OUTPUT_WIDTH) << secondSinceStart
                    << std::right << std::setw(OUTPUT_WIDTH)
                    << static_cast<double>(nbPacketSended) /  secondSinceStart
                    << std::right << std::setw(OUTPUT_WIDTH) << time_delay
                    << std::endl;
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
