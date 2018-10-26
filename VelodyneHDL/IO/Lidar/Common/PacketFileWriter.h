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
  Module:    vtkVelodyneHDLReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef PACKETWRITER_H
#define PACKETWRITER_H

#include <string>
#include <queue>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

#include "vtkPacketFileWriter.h"
#include "SynchronizedQueue.h"

class PacketFileWriter
{
public:
  void ThreadLoop();

  void Start(const std::string& filename);

  void Stop();

  void Enqueue(std::string* packet);

  bool IsOpen() { return this->PacketWriter.IsOpen(); }

  void Close() { this->PacketWriter.Close(); }

private:
  vtkPacketFileWriter PacketWriter;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;
};


#endif // PACKETWRITER_H
