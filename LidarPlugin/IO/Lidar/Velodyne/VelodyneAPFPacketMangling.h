#include "VelodyneAPFCommon.h"

uint8_t * stripIntensities(uint8_t const * data, size_t dataLength, size_t & strippedDataLength)
{
  decltype(dataLength) index = 0;
  size_t requiredSize = 0;
  size_t payloadAndExtensionHeadersSize = 0;

  // This checks that PayloadHeader's IsValid function, which in turn checks
  // that the version is 1 and that expected lengths are consistent.
  PayloadHeader const * payloadHeader = reinterpretCastWithChecks<PayloadHeader>(data, dataLength, index);
  if ((payloadHeader == nullptr) || (payloadHeader->GetHlen() > dataLength))
  {
    return nullptr;
  }
  ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, payloadHeader, nullptr);

  auto nxhdr = payloadHeader->GetNxhdr();
  while (nxhdr != 0)
  {
    ExtensionHeader const * extensionHeader =
      reinterpretCastWithChecks<ExtensionHeader>(data, dataLength, index);
    if (extensionHeader == nullptr)
    {
      return nullptr;
    }
    ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, extensionHeader, nullptr);
    nxhdr = extensionHeader->GetNxhdr();
  }

  size_t const dcount = payloadHeader->GetDistanceCount();
  // Nothing to do if there are no distances. Copy the data anyway to avoid
  // stripping const qualifiers and to ensure that memory can be managed as
  // expected.
  if (dcount == 0)
  {
    strippedDataLength = dataLength;
    uint8_t * strippedData = new uint8_t[dataLength];
    std::memcpy(strippedData, data, dataLength);
    return strippedData;
  }
  size_t const bytesPerDistance = payloadHeader->GetDistanceSizeInBytes();
  size_t const numberOfDataBytesPerFiring = payloadHeader->GetNumberOfDataBytesPerFiring();
  size_t const numberOfBytesPerFiringHeader = payloadHeader->GetFlen();
  size_t const numberOfBytesPerFiring = payloadHeader->GetNumberOfBytesPerFiring();
  size_t const numberOfBytesPerFiringGroupHeader = payloadHeader->GetGlen();
  size_t const numberOfBytesPerStrippedFiring = numberOfBytesPerFiringHeader + dcount * bytesPerDistance;

  requiredSize = index;
  payloadAndExtensionHeadersSize = index;

  // Check for empty distance counts, which mean there are no firings.
  if (payloadHeader->GetDistanceCount() != 0)
  {

    while (index < dataLength)
    {
      FiringGroupHeader const * firingGroupHeader = reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);
      if (firingGroupHeader == nullptr)
      {
        return nullptr;
      }
      // TODO
      // Add firing header checks if necessary here. See ProcessPacket for an
      // example of how to loop over each firing and advance the index.
      index += (numberOfBytesPerFiring * firingGroupHeader->GetFcnt()) +
        numberOfBytesPerFiringGroupHeader;

      requiredSize += numberOfBytesPerFiringGroupHeader + (numberOfBytesPerStrippedFiring * firingGroupHeader->GetFcnt());
    }
  }

  if (index != dataLength)
  {
    return nullptr;
  }


  uint8_t * strippedData = new uint8_t[requiredSize];
  if (strippedData == nullptr)
  {
    return nullptr;
  }

  std::memcpy(strippedData, data, payloadAndExtensionHeadersSize);
  size_t strippedIndex = payloadAndExtensionHeadersSize;

  // nullify ISET
  strippedData[6] = 0;
  strippedData[7] = 0;


  if (payloadHeader->GetDistanceCount() != 0)
  {
    index = payloadAndExtensionHeadersSize;

    while (index < dataLength)
    {
      FiringGroupHeader const * firingGroupHeader = reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);

      // Copy the firing group header.
      std::memcpy(strippedData+strippedIndex, data+index, numberOfBytesPerFiringGroupHeader);
      strippedIndex += numberOfBytesPerFiringGroupHeader;

      for (
        size_t firingIndex = 0, firingOffset = index + numberOfBytesPerFiringGroupHeader; 
        firingIndex < firingGroupHeader->GetFcnt();
        ++firingIndex, firingOffset += numberOfBytesPerFiring
      )
      {
        // Copy the firing header.
        std::memcpy(strippedData + strippedIndex, data + firingOffset, numberOfBytesPerFiringHeader);
        strippedIndex += numberOfBytesPerFiringHeader;

        for (
          size_t returnIndex = 0, returnOffset = firingOffset + numberOfBytesPerFiringHeader; 
          returnIndex < dcount; 
          ++returnIndex, returnOffset += numberOfDataBytesPerFiring
        )
        {
          std::memcpy(strippedData + strippedIndex, data + returnOffset, bytesPerDistance);
          strippedIndex += bytesPerDistance;
        }
      }

      // Add firing header checks if necessary here. See ProcessPacket for an
      // example of how to loop over each firing and advance the index.
      index += (numberOfBytesPerFiring * firingGroupHeader->GetFcnt()) +
        numberOfBytesPerFiringGroupHeader;
    }
  }

  strippedDataLength = strippedIndex;
  return strippedData;
}

