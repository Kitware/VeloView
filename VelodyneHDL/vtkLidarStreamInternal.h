#ifndef VTKLIDARSTREAMINTERNAL_H
#define VTKLIDARSTREAMINTERNAL_H

#include "NetworkSource.h"
#include <memory>


class PacketConsumer;
class PacketFileWriter;
class NetworkSource;

class vtkLidarStreamInternal
{
public:
  vtkLidarStreamInternal(int argLIDARPort, int ForwardedLIDARPort,
                         std::string ForwardedIpAddress, bool isForwarding, bool isCrashAnalysing);


  //! where to save a live record of the sensor
  std::string OutputFileName;


  std::shared_ptr<PacketConsumer> Consumer;
  std::shared_ptr<PacketFileWriter> Writer;
  std::unique_ptr<NetworkSource> Network;
};

#endif // VTKLIDARSTREAMINTERNAL_H
