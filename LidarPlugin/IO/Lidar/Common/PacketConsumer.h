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

#ifndef PACKETCONSUMER_H
#define PACKETCONSUMER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <deque>

#include "vtkSmartPointer.h"
#include "vtkLidarPacketInterpreter.h"
#include "NetworkPacket.h"


template<typename T>
class SynchronizedQueue;

class PacketConsumer
{
public:
  PacketConsumer();

  void HandleSensorData(const unsigned char* data, unsigned int length);

  vtkSmartPointer<vtkPolyData> GetLastAvailableFrame();

  int CheckForNewData();

  void ClearAllFrames() { this->Frames.clear();}

  void Start();

  void Stop();

  void Enqueue(NetworkPacket* packet);

  void SetInterpreter(vtkLidarPacketInterpreter* inter) { this->Interpreter = inter;}

  // Hold this when modifying internals of reader
  boost::mutex ConsumerMutex;

protected:
  void ThreadLoop();

  void HandleNewData(vtkSmartPointer<vtkPolyData> polyData);

  std::deque<vtkSmartPointer<vtkPolyData> > Frames;
  vtkLidarPacketInterpreter* Interpreter;

  boost::shared_ptr<SynchronizedQueue<NetworkPacket*>> Packets;

  boost::shared_ptr<boost::thread> Thread;
};

#endif // PACKETCONSUMER_H
