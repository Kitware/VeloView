#include "PacketConsumer.h"

#include "SynchronizedQueue.h"

//----------------------------------------------------------------------------
PacketConsumer::PacketConsumer()
{
  this->Packets.reset(new SynchronizedQueue<NetworkPacket*>);
}

//----------------------------------------------------------------------------
void PacketConsumer::HandleSensorData(const unsigned char *data, unsigned int length)
{
  this->Interpreter->ProcessPacket(data, length);
  if (this->Interpreter->IsNewFrameReady())
  {
    {
      boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
      this->Frames.push_back(this->Interpreter->GetLastFrameAvailable());
    }
    this->Interpreter->ClearAllFramesAvailable();
  }
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> PacketConsumer::GetLastAvailableFrame()
{
  return this->Frames.back();
}

//----------------------------------------------------------------------------
int PacketConsumer::CheckForNewData()
{
  return this->Frames.size();
}

//----------------------------------------------------------------------------
void PacketConsumer::ThreadLoop()
{
  NetworkPacket* packet = nullptr;
  this->Interpreter->ResetCurrentFrame();
  while (this->Packets->dequeue(packet))
  {
    this->HandleSensorData(packet->GetPayloadData(), packet->GetPayloadSize());
    delete packet;
  }
}

//----------------------------------------------------------------------------
void PacketConsumer::Start()
{
  if (this->Thread)
  {
    return;
  }

  this->Packets.reset(new SynchronizedQueue<NetworkPacket*>);
  this->Thread = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&PacketConsumer::ThreadLoop, this)));
}

//----------------------------------------------------------------------------
void PacketConsumer::Stop()
{
  if (this->Thread)
  {
    this->Packets->stopQueue();
    this->Thread->join();
    this->Thread.reset();
    this->Packets.reset();
  }
}

//----------------------------------------------------------------------------
void PacketConsumer::Enqueue(NetworkPacket* packet)
{
  this->Packets->enqueue(packet);
}
