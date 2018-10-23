#include "NetworkSource.h"
#include "vtkPacketFileWriter.h"

#include "PacketReceiver.h"
#include "PacketFileWriter.h"
#include "PacketConsumer.h"

//-----------------------------------------------------------------------------
NetworkSource::~NetworkSource()
{
  this->Stop();

  delete this->DummyWork;

  if (this->Thread)
  {
    this->Thread->join();
  }
}

//-----------------------------------------------------------------------------
void NetworkSource::QueuePackets(std::string *packet)
{
  std::string* packet2 = 0;
  if (this->Writer)
  {
    packet2 = new std::string(*packet);
  }

  if (this->Consumer)
  {
    this->Consumer->Enqueue(packet);
  }

  if (this->Writer)
  {
    this->Writer->Enqueue(packet2);
  }
}

//-----------------------------------------------------------------------------
void NetworkSource::Start()
{
  if (!this->Thread)
  {
    std::cout << "Start listen" << std::endl;
    this->Thread.reset(
      new boost::thread(boost::bind(&boost::asio::io_service::run, &this->IOService)));
  }

  // Create work
  this->LIDARPortReceiver = boost::shared_ptr<PacketReceiver>(new PacketReceiver(
    this->IOService, LIDARPort, ForwardedLIDARPort, ForwardedIpAddress, IsForwarding, this));

  if (IsCrashAnalysing)
  {
    std::string appDir;

    // the home directory path is contained in the HOME environment variable on UNIX systems
    if (getenv("HOME"))
    {
      appDir = getenv("HOME");
      appDir += "/";
      appDir += SOFTWARE_NAME;
      appDir += "/";
    }
    else
    {
      // On Windows, it's a concatanation of 2 environment variables
      appDir = getenv("HOMEDRIVE");
      appDir += getenv("HOMEPATH");
      appDir += "\\";
      appDir += SOFTWARE_NAME;
      appDir += "\\";
    }

    // Checking if the application directory exists in the home directory and create it otherwise
    boost::filesystem::path appDirPath(appDir.c_str());

    if (!boost::filesystem::is_directory(appDirPath))
    {
      boost::filesystem::create_directory(appDirPath);
    }

    this->LIDARPortReceiver->EnableCrashAnalysing(
      appDir + "LidarLastData.bin", 1206, IsCrashAnalysing);
  }

  this->LIDARPortReceiver->StartReceive();
}

void NetworkSource::Stop()
{
  // Kill the receivers
  this->LIDARPortReceiver.reset();
}
