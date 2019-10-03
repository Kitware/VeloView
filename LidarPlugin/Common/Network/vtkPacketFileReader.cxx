#include "vtkPacketFileReader.h"

//------------------------------------------------------------------------------
bool IPHeaderFunctions::getFragmentInfo(unsigned char const * data, FragmentInfo & fragmentInfo)
{
  fragmentInfo.Reset();
  unsigned char const ipVersion = data[0] >> 4;
  if (ipVersion == 0x4)
  {
    fragmentInfo.Identification = 0x100 * data[4] + data[5];
    fragmentInfo.MoreFragments = (data[6] & 0x20) == 0x20;
    fragmentInfo.Offset = (data[6] & 0x1F) * 0x100 + data[7];
    return true;
  }
  if (ipVersion == 0x6)
  {
    // Scan for the fragment header.
    unsigned char nextHeader = data[6];
    data += 40;
    while (true)
    {
      switch (nextHeader)
      {
        // hop-by-hop and destination options
        case 0:
        case 60:
        // routing
        case 43:
          nextHeader = data[0];
          data += 8 + data[1];
          break;

        // fragment
        case 44:
          fragmentInfo.Offset = (data[2] * 0x100 + data[3]) >> 3;
          fragmentInfo.MoreFragments = (data[3] & 0x1) == 0x1;
          fragmentInfo.Identification = data[4] * 0x1000000 + data[5] * 0x10000 + data[6] * 0x100 + data[7];
          return true;

        // authentication header
        case 51:
          nextHeader = data[0];
          data += 2 + data[1];
          break;

        // encapsulating security payload
        case 50:
        // mobility
        case 135:
        // host identity protocol
        case 139:
        // shim6 protocol
        case 140:
        // reserved
        case 253:
        case 254:
          // None of these are currently handled.
          return false;

        default:
          return false;

      }
    }
  }
  return false;
}

//------------------------------------------------------------------------------
unsigned int IPHeaderFunctions::getIPHeaderLength(unsigned char const* data)
{
  unsigned char const ipVersion = data[0] >> 4;
  // IPv4 declares its length in the lower 4 bits of the first byte.
  if (ipVersion == 0x4)
  {
    return (data[0] & 0xf) * 4;
  }
  // IPv6 includes optional fragment headers that must be walked to determine
  // the full size.
  if (ipVersion == 0x6)
  {
    // Get the next header type.
    unsigned char nextHeader = data[6];
    // Add the length of the IPv4 header.
    unsigned char const* header = data + 40;
    // Collect the lengths of all optional headers.
    bool notPayload = true;
    while (notPayload)
    {
      switch (nextHeader)
      {
        // hop-by-hop and destination options
        case 0:
        case 60:
        // routing
        case 43:
          nextHeader = header[0];
          header += 8 + header[1];
          break;

        // fragment
        case 44:
          nextHeader = header[0];
          header += 8;
          break;

        // authentication header
        case 51:
          nextHeader = header[0];
          header += 2 + header[1];
          break;

        // encapsulating security payload
        case 50:
        // mobility
        case 135:
        // host identity protocol
        case 139:
        // shim6 protocol
        case 140:
        // reserved
        case 253:
        case 254:
          // None of these are currently handled.
          return 0;

        default:
          notPayload = false;
      }
    }
    return (header - data);
  }
  return 0;
}

