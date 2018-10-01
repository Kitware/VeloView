#include "vtkLidarStreamInternal.h"

#include "NetworkSource.h"
#include "PacketConsumer.h"
#include "PacketFileWriter.h"

vtkLidarStreamInternal::vtkLidarStreamInternal(int argLIDARPort, int ForwardedLIDARPort,
                                               std::string ForwardedIpAddress, bool isForwarding, bool isCrashAnalysing)
  : Consumer(new PacketConsumer)
  , Writer(new PacketFileWriter)
  , Network(std::unique_ptr<NetworkSource>(new NetworkSource(this->Consumer, argLIDARPort, ForwardedLIDARPort,
      ForwardedIpAddress, isForwarding, isCrashAnalysing)))
{

}
