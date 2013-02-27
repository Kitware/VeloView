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
#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

//----------------------------------------------------------------------------
namespace
{

//----------------------------------------------------------------------------
class PacketConsumer
{
public:

  PacketConsumer()
  {
    this->NewData = false;
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

  void HandleNewData(vtkSmartPointer<vtkPolyData> polyData)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    this->PolyData = polyData;
    this->NewData = true;
  }

  vtkSmartPointer<vtkPolyData> GetLatestPolyData()
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    vtkSmartPointer<vtkPolyData> polyData = this->PolyData;
    this->PolyData = NULL;
    this->NewData = false;
    return polyData;
  }

  bool HasNewData()
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    return this->NewData;
  }

  bool NewData;
  boost::mutex Mutex;
  vtkSmartPointer<vtkPolyData> PolyData;
  vtkNew<vtkVelodyneHDLReader> HDLReader;
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

    this->Consumer->HandleSensorData(this->RXBuffer, numberOfBytes);
    this->StartReceive();

    //static long packetCounter = 0;
    //if ((++packetCounter % 170) == 0)
    //  {
    //  printf("received frame %ld,  total packets %ld\n", packetCounter/170, packetCounter);
    //  }
  }

  void ThreadLoop()
  {
    this->StartReceive();
    this->IOService.reset();
    this->IOService.run();
  }

  void Start(int sensorPort)
  {
    if (this->Thread)
      {
      return;
      }

    this->Socket.reset();
    //std::string sensorIp = "127.0.0.1";
    //boost::asio::ip::udp::endpoint sensorEndpoint(boost::asio::ip::address_v4::from_string(sensorIp), dataPort);
    boost::asio::ip::udp::endpoint genericEndpoint(boost::asio::ip::udp::v4(), sensorPort);

    try
      {
      this->Socket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(this->IOService));
      this->Socket->open(genericEndpoint.protocol());
      this->Socket->bind(genericEndpoint);
      }
    catch( std::exception & e )
      {
      vtkGenericWarningMacro("Caught Exception: " << e.what());
      return;
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

  bool ShouldStop;
  unsigned char RXBuffer[1500];

  boost::asio::io_service IOService;
  boost::shared_ptr<boost::asio::ip::udp::socket> Socket;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<PacketConsumer> Consumer;
};


//----------------------------------------------------------------------------
class PacketFileSource
{
public:

  void ThreadLoop()
  {
    while (!this->ShouldStop)
      {
        const unsigned char* data = 0;
        unsigned int dataLength = 0;
        double timeSinceStart = 0;
        if (!this->PacketReader.NextPacket(data, dataLength, timeSinceStart))
          {
          printf("end of packet file\n");
          break;
          }

        this->Consumer->HandleSensorData(data, dataLength);
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
      }
  }

  void Start(const std::string& filename)
  {
    if (this->Thread)
      {
      return;
      }

    if (this->PacketReader.FileName != filename)
      {
      this->PacketReader.Close();
      }

    if (!this->PacketReader.IsOpen())
      {
      printf("opening packet file: %s\n", filename.c_str());
      if (!this->PacketReader.Open(filename))
        {
        vtkGenericWarningMacro("Failed to open packet file: " << filename);
        return;
        }
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

  vtkInternal()
  {
    this->Consumer = boost::shared_ptr<PacketConsumer>(new PacketConsumer);
    this->NetworkSource.Consumer = this->Consumer;
    this->FileSource.Consumer = this->Consumer;
  }

  ~vtkInternal()
  {
  }

  boost::shared_ptr<PacketConsumer> Consumer;
  PacketNetworkSource NetworkSource;
  PacketFileSource FileSource;
};

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLSource);

//----------------------------------------------------------------------------
vtkVelodyneHDLSource::vtkVelodyneHDLSource()
{
  this->Internal = new vtkInternal;
  this->PacketFile = 0;
  this->SensorPort = 2368;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkVelodyneHDLSource::~vtkVelodyneHDLSource()
{
  this->Stop();
  this->SetPacketFile(0);
  delete this->Internal;
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Start()
{
  if (this->PacketFile)
    {
    this->Internal->FileSource.Start(this->PacketFile);
    }
  else
    {
    this->Internal->NetworkSource.Start(this->SensorPort);
    }
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Stop()
{
  this->Internal->NetworkSource.Stop();
  this->Internal->FileSource.Stop();
}

//----------------------------------------------------------------------------
bool vtkVelodyneHDLSource::HasNewData()
{
  return this->Internal->Consumer->HasNewData();
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::Poll()
{
  if (this->HasNewData())
    {
    this->Modified();
    }
}

//----------------------------------------------------------------------------
int vtkVelodyneHDLSource::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkDataSet *output = vtkDataSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  if (!this->HasNewData())
    {
    return 1;
    }

  output->ShallowCopy(this->Internal->Consumer->GetLatestPolyData());
  return 1;
}

//----------------------------------------------------------------------------
void vtkVelodyneHDLSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
