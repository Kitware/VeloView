#ifndef PACKETCONSUMER_H
#define PACKETCONSUMER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <vtkNew.h>
#include <deque>

#include "vtkSmartPointer.h"
#include "LidarPacketInterpreter.h"


template<typename T>
class SynchronizedQueue;

class PacketConsumer
{
public:
  PacketConsumer();

  void HandleSensorData(const unsigned char* data, unsigned int length);

  vtkSmartPointer<vtkPolyData> GetDatasetForTime(double timeRequest, double& actualTime, int numberOfTrailingFrame = 0);

  std::vector<double> GetTimesteps();

  int GetMaxNumberOfDatasets() { return this->MaxNumberOfDatasets; }

  void SetMaxNumberOfDatasets(int nDatasets);

  bool CheckForNewData();

  void ThreadLoop();

  void Start();

  void Stop();

  void Enqueue(std::string* packet);

  void SetInterpreter(LidarPacketInterpreter* inter) { this->Interpreter = inter;}

  void UnloadData();

  // Hold this when running reader code code or modifying its internals
  boost::mutex ReaderMutex;

protected:
  void UpdateDequeSize();

  size_t GetIndexForTime(double time);

  void HandleNewData(vtkSmartPointer<vtkPolyData> polyData);

  bool ShouldCheckSensor;
  bool NewData;
  int MaxNumberOfDatasets;
  double LastTime;

  // Hold this when modifying internals of reader
  boost::mutex ConsumerMutex;

  std::deque<vtkSmartPointer<vtkPolyData> > Datasets;
  std::deque<double> Timesteps;
  LidarPacketInterpreter* Interpreter;

  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;

  boost::shared_ptr<boost::thread> Thread;
};

#endif // PACKETCONSUMER_H
