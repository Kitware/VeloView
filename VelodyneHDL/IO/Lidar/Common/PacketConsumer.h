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

#ifndef PACKETCONSUMER_H
#define PACKETCONSUMER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <vtkNew.h>
#include <deque>

#include "vtkSmartPointer.h"
#include "vtkLidarPacketInterpreter.h"


template<typename T>
class SynchronizedQueue;

class PacketConsumer
{
public:
  PacketConsumer();

  void HandleSensorData(const unsigned char* data, unsigned int length);

  // You must lock PacketConsumer.ConsumerMutex while calling this function
  vtkSmartPointer<vtkPolyData> GetFrameForTime(double timeRequest, double& actualTime, int numberOfTrailingFrame = 0);

  std::vector<double> GetTimesteps();

  int GetMaxNumberOfFrames() { return this->MaxNumberOfFrames; }

  void SetMaxNumberOfFrames(int nFrames);

  bool CheckForNewData();

  void ThreadLoop();

  void Start();

  void Stop();

  void Enqueue(std::string* packet);

  void SetInterpreter(vtkLidarPacketInterpreter* inter) { this->Interpreter = inter;}

  void UnloadData();

  // Hold this when running reader code code or modifying its internals
  boost::mutex ReaderMutex;

  // Hold this when modifying internals of reader
  boost::mutex ConsumerMutex;

protected:
  void UpdateDequeSize();

  size_t GetIndexForTime(double time);

  void HandleNewData(vtkSmartPointer<vtkPolyData> polyData);

  bool ShouldCheckSensor;
  bool NewData;
  int MaxNumberOfFrames;
  double LastTime;

  std::deque<vtkSmartPointer<vtkPolyData> > Frames;
  std::deque<double> Timesteps;
  vtkLidarPacketInterpreter* Interpreter;

  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;

  boost::shared_ptr<boost::thread> Thread;
};

#endif // PACKETCONSUMER_H
