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

  void Enqueue(std::string* packet) { this->Packets->enqueue(packet); }

  bool IsOpen() { return this->PacketWriter.IsOpen(); }

  void Close() { this->PacketWriter.Close(); }

private:
  vtkPacketFileWriter PacketWriter;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;
};


#endif // PACKETWRITER_H
