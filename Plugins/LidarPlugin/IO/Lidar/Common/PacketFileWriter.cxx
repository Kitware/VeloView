#include "PacketFileWriter.h"

//! @todo this include is only for vtkGenericWarningMacro which is strange
#include <vtkMath.h>

//-----------------------------------------------------------------------------
void PacketFileWriter::ThreadLoop()
{
  NetworkPacket* packet = nullptr;
  while (this->Packets->dequeue(packet))
  {
    this->PacketWriter.WritePacket(*packet);
    delete packet;
  }
}

//-----------------------------------------------------------------------------
void PacketFileWriter::Start(const std::string &filename)
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

  this->Packets.reset(new SynchronizedQueue<NetworkPacket*>);
  this->Thread = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&PacketFileWriter::ThreadLoop, this)));
}

//-----------------------------------------------------------------------------
void PacketFileWriter::Stop()
{
  if (this->Thread)
  {
    this->Packets->stopQueue();
    this->Thread->join();
    this->Thread.reset();
    this->Packets.reset();
  }
}

//-----------------------------------------------------------------------------
void PacketFileWriter::Enqueue(NetworkPacket* packet)
{
  // TODO
  // After capturing a stream and stoping the recording, Packets is NULL
  // and this loop continues until a new reader or stream is selected.
  if (this->Packets != NULL)
  {
    this->Packets->enqueue(packet);
  }
  else
  {
    this->Stop();
  }
}
