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
  Module:    vtkVelodyneHDLSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkVelodyneHDLSource.h"
#include "vtkVelodyneHDLReader.h"
#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"
#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

#include <queue>
#include <deque>

//----------------------------------------------------------------------------
namespace
{

  template<typename T>
  class SynchronizedQueue
  {
    public:

      SynchronizedQueue () :
        queue_(), mutex_(), cond_(), request_to_end_(false), enqueue_data_(true) { }

      void
      enqueue (const T& data)
      {
        boost::unique_lock<boost::mutex> lock (mutex_);

        if (enqueue_data_)
        {
          queue_.push (data);
          cond_.notify_one ();
        }
      }

      bool
      dequeue (T& result)
      {
        boost::unique_lock<boost::mutex> lock (mutex_);

        while (queue_.empty () && (!request_to_end_))
        {
          cond_.wait (lock);
        }

        if (request_to_end_)
        {
          doEndActions ();
          return false;
        }

        result = queue_.front ();
        queue_.pop ();

        return true;
      }

      void
      stopQueue ()
      {
        boost::unique_lock<boost::mutex> lock (mutex_);
        request_to_end_ = true;
        cond_.notify_one ();
      }

      unsigned int
      size ()
      {
        boost::unique_lock<boost::mutex> lock (mutex_);
        return static_cast<unsigned int> (queue_.size ());
      }

      bool
      isEmpty () const
      {
        boost::unique_lock<boost::mutex> lock (mutex_);
        return (queue_.empty ());
      }

    private:
      void
      doEndActions ()
      {
        enqueue_data_ = false;

        while (!queue_.empty ())
        {
          queue_.pop ();
        }
      }

      std::queue<T> queue_;              // Use STL queue to store data
      mutable boost::mutex mutex_;       // The mutex to synchronise on
      boost::condition_variable cond_;   // The condition to wait for

      bool request_to_end_;
      bool enqueue_data_;
  };

//----------------------------------------------------------------------------
class PacketConsumer
{
public:

  PacketConsumer()
  {
    this->NewData = false;
    this->MaxNumberOfDatasets = 1000;
    this->LastTime = 0.0;
  }

  void HandleSensorData(const unsigned char* data, unsigned int length)
  {
    this->HDLReader->ProcessHDLPacket(const_cast<unsigned char*>(data), length);
    if (this->HDLReader->GetDatasets().size())
      {
      this->HandleNewData(this->HDLReader->GetDatasets().back());
      this->HDLReader->GetDatasets().clear();
      }
  }

  vtkSmartPointer<vtkPolyData> GetDatasetForTime(double timeRequest, double& actualTime)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);

    size_t stepIndex = this->GetIndexForTime(timeRequest);
    if (stepIndex < this->Timesteps.size())
      {
      actualTime = this->Timesteps[stepIndex];
      return this->Datasets[stepIndex];
      }
    actualTime = 0;
    return 0;
  }

  std::vector<double> GetTimesteps()
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    const size_t nTimesteps = this->Timesteps.size();
    std::vector<double> timesteps(nTimesteps, 0);
    for (size_t i = 0; i < nTimesteps; ++i)
      {
      timesteps[i] = this->Timesteps[i];
      }
    return timesteps;
  }


  int GetMaxNumberOfDatasets()
  {
    return this->MaxNumberOfDatasets;
  }

  void SetMaxNumberOfDatasets(int nDatasets)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    this->MaxNumberOfDatasets = nDatasets;
    this->UpdateDequeSize();
  }

  bool CheckForNewData()
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    bool newData = this->NewData;
    this->NewData = false;
    return newData;
  }


  void ThreadLoop()
  {
    std::string* packet = 0;
    while (this->Packets->dequeue(packet))
      {
      this->HandleSensorData(reinterpret_cast<const unsigned char*>(packet->c_str()), packet->length());
      delete packet;
      }
  }

  void Start()
  {
    if (this->Thread)
      {
      return;
      }

    this->Packets.reset(new SynchronizedQueue<std::string*>);
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketConsumer::ThreadLoop, this)));
  }

  void Stop()
  {
    if (this->Thread)
      {
      this->Packets->stopQueue();
      this->Thread->join();
      this->Thread.reset();
      this->Packets.reset();
      }
  }

  void Enqueue(std::string* packet)
  {
    this->Packets->enqueue(packet);
  }

  vtkVelodyneHDLReader* GetReader()
  {
    return this->HDLReader.GetPointer();
  }

protected:

  void UpdateDequeSize()
  {
    if (this->MaxNumberOfDatasets <= 0)
      {
      return;
      }
    while (this->Datasets.size() >= this->MaxNumberOfDatasets)
      {
      this->Datasets.pop_front();
      this->Timesteps.pop_front();
      }
  }

  size_t GetIndexForTime(double time)
  {
    size_t index = 0;
    double minDifference = VTK_DOUBLE_MAX;
    const size_t nTimesteps = this->Timesteps.size();
    for (size_t i = 0; i < nTimesteps; ++i)
      {
      double difference = std::abs(this->Timesteps[i] - time);
        if (difference < minDifference)
          {
          minDifference = difference;
          index = i;
          }
      }
    return index;
  }

  void HandleNewData(vtkSmartPointer<vtkPolyData> polyData)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);

    this->UpdateDequeSize();
    this->Timesteps.push_back(this->LastTime);
    this->Datasets.push_back(polyData);
    this->NewData = true;
    this->LastTime += 1.0;
  }

  bool NewData;
  int MaxNumberOfDatasets;
  double LastTime;
  boost::mutex Mutex;
  boost::mutex PacketMutex;
  std::deque<vtkSmartPointer<vtkPolyData> > Datasets;
  std::deque<double> Timesteps;
  vtkNew<vtkVelodyneHDLReader> HDLReader;

  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;

  boost::shared_ptr<boost::thread> Thread;
};


//----------------------------------------------------------------------------
class PacketFileWriter
{
public:

  void ThreadLoop()
  {
    std::string* packet = 0;
    while (this->Packets->dequeue(packet))
      {
      this->PacketWriter.WritePacket(reinterpret_cast<const unsigned char*>(packet->c_str()), packet->length());

      delete packet;
      }
  }

  void Start(const std::string& filename)
  {
    if (this->Thread)
      {
      return;
      }

    if (this->PacketWriter.GetFileName() != filename)
      {
      this->PacketWriter.Close();
      }

    if (!this->PacketWriter.IsOpen())
      {
      if (!this->PacketWriter.Open(filename))
        {
        vtkGenericWarningMacro("Failed to open packet file: " << filename);
        return;
        }
      }

    this->Packets.reset(new SynchronizedQueue<std::string*>);
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketFileWriter::ThreadLoop, this)));
  }

  void Stop()
  {
    if (this->Thread)
      {
      this->Packets->stopQueue();
      this->Thread->join();
      this->Thread.reset();
      this->Packets.reset();
      }
  }

  void Enqueue(std::string* packet)
  {
    this->Packets->enqueue(packet);
  }

  bool IsOpen()
  {
    return this->PacketWriter.IsOpen();
  }

  void Close()
  {
    this->PacketWriter.Close();
  }

private:
  vtkPacketFileWriter PacketWriter;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;
};


//----------------------------------------------------------------------------
class PacketNetworkSource
{
public:

  void StartReceive()
  {
    // expecting exactly 1206 bytes, using a larger buffer so that if a
    // larger packet arrives unexpectedly we'll notice it.
    this->Socket->async_receive(boost::asio::buffer(this->RXBuffer, 1500),
      boost::bind(&PacketNetworkSource::SocketCallback, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  void SocketCallback(const boost::system::error_code& error, std::size_t numberOfBytes)
  {
    if (this->ShouldStop)
      {
      return;
      }

    //this->Consumer->HandleSensorData(this->RXBuffer, numberOfBytes);

    if( this->Consumer )
      {
      std::string* packet = new std::string(this->RXBuffer, numberOfBytes);
      this->Consumer->Enqueue(packet);
      }

    if (this->Writer)
      {
      std::string* packet = new std::string(this->RXBuffer, numberOfBytes);
      this->Writer->Enqueue(packet);
      }

    this->StartReceive();

    if ((++this->PacketCounter % 500) == 0)
      {
      std::cout << "RECV packets: " << this->PacketCounter << " on "
                << this->PortNumber << "[";
      if(this->Consumer)
        {
        std::cout << "C";
        }
      if(this->Writer)
        {
        std::cout << "W";
        }
      std::cout << "]" << std::endl;
      }
  }

  void ThreadLoop()
  {
    this->StartReceive();
    this->IOService.reset();
    this->IOService.run();
  }

  void Start()
  {
    if (this->Thread)
      {
      return;
      }

    std::string destinationIp = "192.168.3.255";

    this->Socket.reset();

    // destinationEndpoint is the socket on this machine where the data packet are received
    boost::asio::ip::udp::endpoint destinationEndpoint(boost::asio::ip::address_v4::any(),
                                                       this->PortNumber);

    try
      {
      this->Socket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(this->IOService));
      this->Socket->open(destinationEndpoint.protocol());
      this->Socket->bind(destinationEndpoint);
      }
    catch( std::exception & e )
      {
      vtkGenericWarningMacro("Caught exception while binding to " << destinationIp << ": " << e.what());

      try
        {
        destinationEndpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(),
                                                             this->PortNumber);

        this->Socket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(this->IOService));
        this->Socket->open(destinationEndpoint.protocol());
        this->Socket->bind(destinationEndpoint);
        }
      catch( std::exception & e )
        {
        vtkGenericWarningMacro("Caught Exception: " << e.what());
        return;
        }
      }

    this->ShouldStop = false;
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketNetworkSource::ThreadLoop, this)));
  }

  void Stop()
  {
    this->ShouldStop = true;
    if (this->Thread)
      {
      this->Socket->close();
      this->IOService.stop();
      this->Thread->join();
      this->Thread.reset();
      }
  }

  PacketNetworkSource(int _PortNumber) : PortNumber(_PortNumber),
                                         ShouldStop(false),
                                         PacketCounter(0)
  {
  }

  int PortNumber;
  bool ShouldStop;
  char RXBuffer[1500];
  vtkIdType PacketCounter;

  boost::asio::io_service IOService;
  boost::shared_ptr<boost::asio::ip::udp::socket> Socket;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<PacketConsumer> Consumer;
  boost::shared_ptr<PacketFileWriter> Writer;
};


//----------------------------------------------------------------------------
class PacketFileSource
{
public:

  void ThreadLoop()
  {
    while (!this->ShouldStop)
      {
      if (!this->ReadNextPacket())
        {
        break;
        }

        //boost::this_thread::sleep(boost::posix_time::microseconds(100));
      }
  }

  bool Open(const std::string& filename)
  {
    if (this->PacketReader.GetFileName() != filename)
      {
      this->PacketReader.Close();
      }

    if (!this->PacketReader.IsOpen())
      {
      if (!this->PacketReader.Open(filename))
        {
        vtkGenericWarningMacro("Failed to open packet file: " << filename);
        return false;
        }
      }

    return true;
  }

  bool ReadNextPacket()
  {
    const unsigned char* data = 0;
    unsigned int dataLength = 0;
    double timeSinceStart = 0;
    if (!this->PacketReader.NextPacket(data, dataLength, timeSinceStart))
      {
      return false;
      }

    this->Consumer->HandleSensorData(data, dataLength);
    return true;
  }

  bool ReadNextFrame()
  {
    if (this->Thread)
      {
      vtkGenericWarningMacro("ReadNextFrame() called while thread is active.");
      return false;
      }

    // todo - handle end of file packets that create a partial frame
    while (this->ReadNextPacket())
      {
      if (this->Consumer->CheckForNewData())
        {
        return true;
        }
      }
    return false;
  }

  void Start(const std::string& filename)
  {
    if (this->Thread)
      {
      return;
      }

    this->ShouldStop = false;
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketFileSource::ThreadLoop, this)));
  }

  void Stop()
  {
    this->ShouldStop = true;
    if (this->Thread)
      {
      this->Thread->join();
      this->Thread.reset();
      }
  }

  bool ShouldStop;
  vtkPacketFileReader PacketReader;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<PacketConsumer> Consumer;
};

} // end namespace

//----------------------------------------------------------------------------
class vtkVelodyneHDLSource::vtkInternal
{
public:

  vtkInternal() : NetworkSource(2368), PositionNetworkSource(8308)
  {
    this->Consumer = boost::shared_ptr<PacketConsumer>(new PacketConsumer);
    this->Writer = boost::shared_ptr<PacketFileWriter>(new PacketFileWriter);
    this->NetworkSource.Consumer = this->Consumer;
    // Position source does not need a consumer
    this->FileSource.Consumer = this->Consumer;
  }

  ~vtkInternal()
  {
  }

  boost::shared_ptr<PacketConsumer> Consumer;
  boost::shared_ptr<PacketFileWriter> Writer;
  PacketNetworkSource NetworkSource;
  PacketNetworkSource PositionNetworkSource;
  PacketFileSource FileSource;
};

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLSource);

//----------------------------------------------------------------------------
vtkVelodyneHDLSource::vtkVelodyneHDLSource()
{
  this->Internal = new vtkInternal;
  this->SensorPort = 2368;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkVelodyneHDLSource::~vtkVelodyneHDLSource()
{
  this->Stop();
  delete this->Internal;
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLSource::GetPacketFile()
{
  return this->PacketFile;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetPacketFile(const std::string& filename)
{
  if (filename == this->GetPacketFile())
    {
    return;
    }

  this->PacketFile = filename;
  this->Modified();
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLSource::GetOutputFile()
{
  return this->OutputFile;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetOutputFile(const std::string& filename)
{
  if (filename == this->GetOutputFile())
    {
    return;
    }

  this->Internal->Writer->Close();
  this->OutputFile = filename;
  this->Modified();
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLSource::GetCorrectionsFile()
{
  return this->Internal->Consumer->GetReader()->GetCorrectionsFile();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetCorrectionsFile(const std::string& filename)
{
  if (filename == this->GetCorrectionsFile())
    {
    return;
    }

  this->Internal->Consumer->GetReader()->SetCorrectionsFile(filename);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetLaserMask(int LaserMask[64])
{
  this->Internal->Consumer->GetReader()->SetLaserMask(LaserMask);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::GetLaserMask(int LaserMask[64])
{
  this->Internal->Consumer->GetReader()->GetLaserMask(LaserMask);
}


//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::GetVerticalCorrections(double VerticalCorrections[64])
{
  this->Internal->Consumer->GetReader()->GetVerticalCorrections(VerticalCorrections);
}


//-----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetDummyProperty(int vtkNotUsed(dummy))
{
  this->Modified();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Start()
{
  if (this->PacketFile.length())
    {
    this->Internal->FileSource.Start(this->PacketFile);
    }
  else
    {
    if (this->OutputFile.length())
      {
      this->Internal->Writer->Start(this->OutputFile);
      }

    this->Internal->NetworkSource.Writer.reset();
    this->Internal->PositionNetworkSource.Writer.reset();

    if (this->Internal->Writer->IsOpen())
      {
      this->Internal->NetworkSource.Writer = this->Internal->Writer;
      this->Internal->PositionNetworkSource.Writer = this->Internal->Writer;
      }

    this->Internal->Consumer->Start();

    this->Internal->NetworkSource.Start();
    this->Internal->PositionNetworkSource.Start();
    }
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Stop()
{
  this->Internal->FileSource.Stop();
  this->Internal->NetworkSource.Stop();
  this->Internal->PositionNetworkSource.Stop();
  this->Internal->Consumer->Stop();
  this->Internal->Writer->Stop();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::ReadNextFrame()
{
  if (this->PacketFile.length())
    {
    if (!this->Internal->FileSource.Open(this->PacketFile))
      {
      return;
      }

    if (this->Internal->FileSource.ReadNextFrame())
      {
      this->Modified();
      }
    }
}


//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Poll()
{
  if (this->Internal->Consumer->CheckForNewData())
    {
    this->Modified();
    }
}

//----------------------------------------------------------------------------
int vtkVelodyneHDLSource::GetCacheSize()
{
  return this->Internal->Consumer->GetMaxNumberOfDatasets();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::SetCacheSize(int cacheSize)
{
  if (cacheSize == this->GetCacheSize())
    {
    return;
    }

  this->Internal->Consumer->SetMaxNumberOfDatasets(cacheSize);
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLSource::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  std::vector<double> timesteps = this->Internal->Consumer->GetTimesteps();
  const size_t nTimesteps = timesteps.size();
  if (nTimesteps > 0)
    {
    outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), static_cast<int>(nTimesteps));
    }
  else
    {
    outInfo->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    }

  double timeRange[2] = {0, nTimesteps ? nTimesteps - 1 : 0};
  outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);

  return 1;
}

//----------------------------------------------------------------------------
int vtkVelodyneHDLSource::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkDataSet *output = vtkDataSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  double timeRequest = 0;
  if (outInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
    {
    timeRequest = outInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
    }


  double actualTime;
  vtkSmartPointer<vtkPolyData> polyData = this->Internal->Consumer->GetDatasetForTime(timeRequest, actualTime);
  if (polyData)
    {
    //printf("request %f, returning %f\n", timeRequest, actualTime);
    output->GetInformation()->Set(vtkDataObject::DATA_TIME_STEP(), actualTime);
    output->ShallowCopy(polyData);
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
