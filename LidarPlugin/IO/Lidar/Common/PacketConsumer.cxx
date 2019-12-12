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
  if (!this->Interpreter->IsLidarPacket(data, length))
    return;
  this->Interpreter->ProcessPacket(data, length);
  if (this->Interpreter->IsNewFrameReady())
  {
    {
      boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
      this->Frames.push_back(this->Interpreter->GetLastFrameAvailable());
      size_t size = this->Frames.size();
      // This prevents accumulating frames forever when "Pause" is toggled
      // There is little reason to use a std::deque to cache the frames, so
      // while waiting for a better fix, lets set a maximum size to the queue.
      // If this maximum size is too big (>= 100) this seems to cause a
      // memory leak. TODO: investigate (not needed if a refactor removes the
      // queue)
      if (this->Frames.size() > 2) {
          this->Frames.pop_back();
      }
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

  // In order to prevent memory usage to grow unbounded, we limit the growth of
  // the packet cache. Above an arbitrary limit it seems safe to assume that
  // the lateness of the consuming (decoding) thread has zero chance of being
  // catched up and that packets can be dropped.
  // To test: look at memory usage while running PacketFileSender --speed 100
  if (this->Packets->size() > this->PacketCacheSize)
  {
      NetworkPacket* packet = nullptr;
      this->Packets->dequeue(packet);
      delete packet;
  }
}
