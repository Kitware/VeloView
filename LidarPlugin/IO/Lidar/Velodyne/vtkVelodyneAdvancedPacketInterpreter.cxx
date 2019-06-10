#include "vtkVelodyneAdvancedPacketInterpreter.h"
#include "VelodyneAPFCommon.h"

#include <vtkDoubleArray.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>

#include <type_traits>

#include <cstring>

#include <iostream>

// include only to go the structure VelodyneSpecificFrameInformation before merging code
#include <vtkVelodyneLegacyPacketInterpreter.h>

//------------------------------------------------------------------------------
// FrameTracker
//------------------------------------------------------------------------------
/*!
 * @brief Object to track information related to frame changes. All frame change
 *        logic should be kept here.
 *
 * The current logic was taken from the VelArray branch.
 */
class FrameTracker
{
private:
  FramingLogic FrameLogic { FramingLogic::FL_DEFAULT };
  bool HasLastState;
  HorizontalDirection LastHorDir;
  VerticalDirection LastVertDir;
  int LastSlope;
  decltype(std::declval<FiringGroupHeader>().GetAzm()) LastAzimuth;
  decltype(std::declval<FiringGroupHeader>().GetVdfl()) LastVertDeflection;

public:
  FrameTracker()
  {
    this->Reset();
  }

  void SetFramingLogic(FramingLogic fl)
  {
    this->FrameLogic = fl;
  }

  //! @brief Reset frame detection.
  void
  Reset()
  {
    this->HasLastState = false;
    this->LastSlope      = 0;
    this->LastAzimuth    = static_cast<decltype(this->LastAzimuth)>(-1);
  }

  /*!
   * @brief     Update the frame tracker.
   * @param[in] payloadHeader     The payload header, used to determine sensor
   *                              model.
   * @param[in] firingGroupHeader The firing group header to inspect for frame
   *                              changes.
   * @return    True if a new frame is detected, false otherwise.
   */
  bool
  Update(
    PayloadHeader const * const vtkNotUsed(payloadHeader),
    FiringGroupHeader const * const firingGroupHeader)
  {
    // Get and update all member values here to avoid doing so in various blocks
    // below just before returning.

    decltype(this->LastAzimuth) azimuth     = firingGroupHeader->GetAzm(),
                                lastAzimuth = this->LastAzimuth;

    decltype(this->LastVertDeflection) vertDeflection = firingGroupHeader->GetVdfl(); //,
                                       // lastVertDeflection = this->LastVertDeflection;
    // Follow the logic of the legacy interpreter.
    int slope = static_cast<int>(azimuth) - static_cast<int>(lastAzimuth),
        lastSlope = this->LastSlope;

    HorizontalDirection horDir = firingGroupHeader->GetHdir(); //,
                        // lastHorDir = this->LastHorDir;

    VerticalDirection vertDir = firingGroupHeader->GetVdir(),
                      lastVertDir = this->LastVertDir;

    bool hasLastState = this->HasLastState;

    this->LastAzimuth = azimuth; // static_cast<decltype(this->LastAzimuth)>(-1);
    this->LastVertDeflection = vertDeflection;
    // this->LastSlope   = slope;
    this->LastHorDir = horDir;
    this->LastVertDir = vertDir;
    this->HasLastState = true;

    if (! hasLastState)
    {
      return false;
    }

    if (this->FrameLogic != FramingLogic::FL_AZIMUTH_CROSSING)
    {
      return (vertDir != lastVertDir);
    }

    // // VelArray
    // ModelIdentificationCode mic = payloadHeader->GetMic();
    // if (mic == ModelIdentificationCode::MIC_VELARRAY || firingGroupHeader->GetVdfl() != 0)
    // {
    //   // The frame split if either the vertical direction changes (vertical
    //   // scanning) OR the horizontal direction changes without a change in the
    //   // vertical deflection (pure horizontal scanning).
    //   return (
    //       (vertDir != lastVertDir) ||
    //       (horDir != lastHorDir && vertDeflection == lastVertDeflection)
    //     );
    // }

    // Not VelArray
    else
    {
      // // New frame when the azimuth rolls over.
      // if (azimuth == 0 and lastAzimuth != 0)
      // {
      //   return true;
      // }
      // else
      // {
      //   return this->HasLastHorDir ? (horDir != lastHorDir) : false;
      // }

      // Old logic from VelArray FramingState. This doesn't seem to work for
      // other sensors.
      if (slope == 0)
      {
        return false;
      }

      int isSameSlopeDir = slope * lastSlope;
      if (isSameSlopeDir > 0)
      {
        return false;
      }

      else if (isSameSlopeDir < 0)
      {
        this->LastSlope = 0;
        return true;
      }

      if (this->LastSlope == 0 && slope != 0)
      {
        this->LastSlope = slope;
        return false;
      }
      vtkGenericWarningMacro("Unhandled sequence in framing state.");
      return false;
    }

    return false;
  }
};

//------------------------------------------------------------------------------
// vtkVelodyneAdvancedPacketInterpreter methods.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneAdvancedPacketInterpreter)

//----------------------------------------------------------------------------
vtkVelodyneAdvancedPacketInterpreter::vtkVelodyneAdvancedPacketInterpreter()
{
  this->CurrentFrameTracker          = new FrameTracker();
  this->MaxFrameSize                 = MEM_STEP_SIZE;
  this->CurrentArraySize             = 0;
  this->NumberOfPointsInCurrentFrame = 0;
  this->ResetCurrentFrame();

  this->ParserMetaData.SpecificInformation =
    std::make_shared<VelodyneSpecificFrameInformation>();
}

//------------------------------------------------------------------------------
vtkVelodyneAdvancedPacketInterpreter::~vtkVelodyneAdvancedPacketInterpreter()
{
  delete this->CurrentFrameTracker;
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::ProcessPacket(
  const unsigned char * data,
  unsigned int dataLength)
{
  decltype(dataLength) index = 0;
  PayloadHeader const * payloadHeader =
    reinterpretCastWithChecks<PayloadHeader>(data, dataLength, index);
  if (payloadHeader == nullptr)
  {
    return;
  }
  ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, payloadHeader, void())

  uint8_t const distanceCount = payloadHeader->GetDistanceCount();

  // A valid distance count may be 0, in which case there are no returns
  // included in the firing and thus there is nothing to do.
  if (distanceCount == 0)
  {
    return;
  }

  uint8_t const distanceSize = payloadHeader->GetDistanceSizeInBytes();
  uint8_t distanceIndex;

  auto const pseq       = payloadHeader->GetPseq();
  auto const iset       = payloadHeader->GetIset();
  auto const dsetMask   = payloadHeader->GetDsetMask();
  auto const isDsetMask = payloadHeader->IsDsetMask();

  // 64-bit PTP truncated format.
  // auto const timeRef = payloadHeader->GetTref();
  auto const packetTimeReferenceInNanoseconds = payloadHeader->GetTrefInNanoseconds();

  size_t const numberOfBytesPerFiringGroupHeader = payloadHeader->GetGlen();
  size_t const numberOfBytesPerFiringHeader      = payloadHeader->GetFlen();
  size_t const numberOfBytesPerFiringReturn = payloadHeader->GetNumberOfBytesPerFiringReturn();
  size_t const numberOfBytesPerFiring = payloadHeader->GetNumberOfBytesPerFiring();

  // Skip optional extension headers.
  auto nxhdr = payloadHeader->GetNxhdr();
  while (nxhdr != 0)
  {
    ExtensionHeader const * extensionHeader =
      reinterpretCastWithChecks<ExtensionHeader>(data, dataLength, index);
    if (extensionHeader == nullptr)
    {
      return;
    }
    ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, extensionHeader, void())
    nxhdr = extensionHeader->GetNxhdr();
  }

  // The included distance types may be specified by a bit mask in DSET, for
  // example 0110 indicates the presence of distance type 0100 and 0010. To
  // display this information, we need to retrieve the type in the firing loop
  // below. To avoid redundant bit calculations to retrieve the nth bit from the
  // mask, we cache the results in a vector instead. The order of the distance
  // types is determined by their bit values in the mask per the specification
  // so this is safe.

  // To do this, we start with the DSET mask and determine the value of the
  // first set bit. This is the distance type of the first distance. We then
  // remove that bit from the mask by subtraction and proceed to the next bit
  // and repeat. The values are stored successively so that they can be indexed
  // below.
  std::remove_const<decltype(dsetMask)>::type dsetRemainingMask = dsetMask;
  decltype(dsetRemainingMask) dsetBit;
  std::vector<decltype(dsetRemainingMask)> distanceTypes;
  for (distanceIndex = 0; distanceIndex < distanceCount; ++distanceIndex)
  {
    if (isDsetMask)
    {
      dsetBit = FIRST_SET_BIT(dsetRemainingMask);
      distanceTypes.push_back(dsetBit);
      dsetRemainingMask -= dsetBit;
    }
    // DSET may specify a count instead of a mask, in which case the distance
    // types are not specified. Use 0 in that case, which has no meaning in this
    // context.
    else
    {
      distanceTypes.push_back(0);
    }
  }

  // Resize the arrays if necessary.
  size_t currentArraySize = this->Points->GetNumberOfPoints();
  size_t safeArraySize    = this->NumberOfPointsInCurrentFrame +
                         payloadHeader->MaximumNumberOfPointsPerPacket();
  if (currentArraySize < safeArraySize)
  {
    this->SetNumberOfItems(safeArraySize);
  }

  // Loop through firing groups until a frame shift is detected. The number of
  // firings in each group is variable so we need to step through all of them to
  // get to the startPosition calculated by PreProcessPacket.
  size_t loopCount = 0;
  while (index < dataLength)
  {
    align_to_word_size(index);
    FiringGroupHeader const * firingGroupHeader =
      reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);
    if (firingGroupHeader == nullptr)
    {
      return;
    }
    // The payload header checks above ensure that this value is non-zero and
    // that the loop will therefore eventually terminate.
    index += numberOfBytesPerFiringGroupHeader;

    // Skip the firings and jump to the next firing group header.
    if (
      (loopCount++) < static_cast<size_t>(
        reinterpret_cast<VelodyneSpecificFrameInformation *>(
          this->ParserMetaData.SpecificInformation.get()
        )->FiringToSkip
      )
    )
    {
      index += numberOfBytesPerFiring * firingGroupHeader->GetFcnt();
      continue;
    }

    bool isNewFrame = this->CurrentFrameTracker->Update(payloadHeader, firingGroupHeader);
    if (isNewFrame)
    {
      this->SplitFrame();
    }

    auto const timeFractionOffsetInNanoseconds         = firingGroupHeader->GetToffs();
    auto const coChannelSpan                           = firingGroupHeader->GetFspn();
    auto const coChannelTimeFractionDelayInNanoseconds = firingGroupHeader->GetFdly();
    auto const vdfl                                    = firingGroupHeader->GetVdfl();
    // double const verticalAngleInDegrees             = firingGroupHeader->GetVerticalDeflection();
    auto const azimuth                                 = firingGroupHeader->GetAzm();
    double const azimuthInDegrees                      = firingGroupHeader->GetAzimuth();
    auto const numberOfFirings                         = firingGroupHeader->GetFcnt();

    for (
      std::remove_const<decltype(numberOfFirings)>::type i = 0;
      i < numberOfFirings;
      ++i
    )
    {
      // TODO
      // This assumes that the spans are returned in order in the firing group.
      // Check that this is the case. If not, determine how to handle this
      // (e.g. by using the channel number?).

      // Intentional truncating integer division. The span index counts the
      // number of spans and should only increment after multiples of
      // coChannelSpan.
      // Also note that these operations are safe despite the mixed types due
      // to the rules for implicit integer promotion.
      decltype(timeFractionOffsetInNanoseconds) iSpan = i / coChannelSpan;
      decltype(timeFractionOffsetInNanoseconds) channelTimeFractionOffsetInNanoseconds =
        timeFractionOffsetInNanoseconds + (coChannelTimeFractionDelayInNanoseconds * iSpan);

      decltype(packetTimeReferenceInNanoseconds) firingTimeInNanoseconds =
        packetTimeReferenceInNanoseconds + channelTimeFractionOffsetInNanoseconds;

      FiringHeader const * firingHeader = reinterpretCastWithChecks<FiringHeader>(data, dataLength, index);
      if (firingHeader == nullptr)
      {
        return;
      }
      index += numberOfBytesPerFiringHeader;

      auto const channelNumber = firingHeader->GetLcn();
      // only process point when the laser is selected
      if (!this->LaserSelection[static_cast<int>(channelNumber)])
      {
        index += numberOfBytesPerFiringReturn * distanceCount;
        continue;
      }

      // auto const firingMode = firingHeader->GetFm();
      // auto const firingModeString = toString(firingMode);
      auto const power = firingHeader->GetPwr();
      auto const noise = firingHeader->GetNf();
      // Status is also an enum and requires a string conversion.
      // auto const status = firingHeader->GetStat();
      // auto const statusString = toString(status);

      for (distanceIndex = 0; distanceIndex < distanceCount; ++distanceIndex)
      {
        // Given the checks in place in IsLidarPacket and above, this should not
        // be necessary because this point of code should only be reached by
        // well-formed packets of the expected length.
        //
        // This was added due to a bug reported about a crash when no intensity
        // values are provided. The test data was not provided so the cause
        // could not be determined with certainty. This check was added as a
        // precaution but it may be unnecessary and unrelated.
        decltype(dataLength) availableBytes = ((dataLength > index) ? dataLength - index : 0);
        if (availableBytes > numberOfBytesPerFiringReturn)
        {
          availableBytes = numberOfBytesPerFiringReturn;
        }
        FiringReturn firingReturn(data + index, availableBytes);
        index += numberOfBytesPerFiringReturn;

        uint32_t const distance = firingReturn.GetDistance<uint32_t>(distanceSize);
        if (this->IgnoreZeroDistances && distance == 0)
        {
          continue;
        }

        RawValues rawValues(azimuth, vdfl, distance);
        CorrectedValues correctedValues;
        double (& position)[3] = correctedValues.position;

        this->ComputeCorrectedValues(
          rawValues,
          channelNumber,
          correctedValues,
          false);

        // Check if the point should be cropped out.
        if (this->shouldBeCroppedOut(position))
        {
          continue;
        }

        auto arrayIndex = this->NumberOfPointsInCurrentFrame++;
        this->Points->SetPoint(arrayIndex, position);

//! @brief Convencience macro for setting info array values.
#define VAPI_SET_VALUE(my_array, value)                                        \
  this->INFO_##my_array->SetValue(arrayIndex, value);

        auto distType = distanceTypes[distanceIndex];
        VAPI_SET_VALUE(Xs, position[0])
        VAPI_SET_VALUE(Ys, position[1])
        VAPI_SET_VALUE(Zs, position[2])
        VAPI_SET_VALUE(Azimuths, azimuthInDegrees)
        VAPI_SET_VALUE(Distances, correctedValues.distance)
        VAPI_SET_VALUE(RawDistances, distance)
        VAPI_SET_VALUE(DistanceTypes, distType)
        // VAPI_SET_VALUE(DistanceTypeStrings, toString(toDistanceType(distType)))
        VAPI_SET_VALUE(Pseqs, pseq)
        VAPI_SET_VALUE(ChannelNumbers, channelNumber)
        VAPI_SET_VALUE(TimeFractionOffsets, channelTimeFractionOffsetInNanoseconds)
        VAPI_SET_VALUE(Timestamps, firingTimeInNanoseconds)
        VAPI_SET_VALUE(Powers, power)
        VAPI_SET_VALUE(Noises, noise)
        VAPI_SET_VALUE(VerticalAngles, correctedValues.elevation)

        //! @brief Convenience macro for setting intensity values
#define VAPI_INSERT_INTENSITY(my_array, iset_flag)                             \
  this->INFO_##my_array->SetValue(                                             \
    arrayIndex,                                                                \
    (iset & (ISET_##iset_flag)) ? firingReturn.GetIntensity<uint32_t>(         \
                                    distanceSize, iset, (ISET_##iset_flag))    \
                                : 0);

        // TODO: Make the inclusion of these columns fully optional at runtime.

        // Add additional values here when ISET is expanded in future versions.
        VAPI_INSERT_INTENSITY(Reflectivities, REFLECTIVITY)
        VAPI_INSERT_INTENSITY(Intensities, INTENSITY)
        VAPI_INSERT_INTENSITY(Confidences, CONFIDENCE)
      }
    }
  }
}
//------------------------------------------------------------------------------
bool
vtkVelodyneAdvancedPacketInterpreter::IsLidarPacket(
  unsigned char const * data,
  unsigned int dataLength)
{
  decltype(dataLength) index = 0;

  // This checks that PayloadHeader's IsValid function, which in turn checks
  // that the version is 1 and that expected lengths are consistent.
  PayloadHeader const * payloadHeader =
    reinterpretCastWithChecks<PayloadHeader>(data, dataLength, index);
  if ((payloadHeader == nullptr) || (payloadHeader->GetHlen() > dataLength))
  {
    return false;
  }
  ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, payloadHeader, false);

  auto nxhdr = payloadHeader->GetNxhdr();
  while (nxhdr != 0)
  {
    ExtensionHeader const * extensionHeader =
      reinterpretCastWithChecks<ExtensionHeader>(data, dataLength, index);
    if (extensionHeader == nullptr)
    {
      return false;
    }
    ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, extensionHeader, false);
    nxhdr = extensionHeader->GetNxhdr();
  }

  // Check for empty distance counts, which mean there are no firings.
  if (payloadHeader->GetDistanceCount() != 0)
  {
    size_t numberOfBytesPerFiring = payloadHeader->GetNumberOfBytesPerFiring();
    size_t numberOfBytesPerFiringGroupHeader = payloadHeader->GetGlen();

    while (index < dataLength)
    {
      align_to_word_size(index);
      FiringGroupHeader const * firingGroupHeader =
        reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);
      if (firingGroupHeader == nullptr)
      {
        return false;
      }
      // TODO
      // Add firing header checks if necessary here. See ProcessPacket for an
      // example of how to loop over each firing and advance the index.
      index += (numberOfBytesPerFiring * firingGroupHeader->GetFcnt()) +
        numberOfBytesPerFiringGroupHeader;
    }
  }

  // return true;
  return index == dataLength;
}

//------------------------------------------------------------------------------
/*!
 * @brief         Initialize an array for datapoint attributes and add it to the
 *                polyData.
 * @tparam        T                The type of the array. This is templated so
 *                                 that the caller does not need to consider the
 *                                 type, which may change with the
 *                                 specification.
 * @param[in,out] array            The input array.
 * @param[in]     numberOfElements The number of elements that the array must be
 *                                 able to hold after initialization.
 * @param[out]    polyData         The PolyData instance to which the array
 *                                 should be added.
 */
template <typename T>
inline void
InitializeDataArrayForPolyData(
  T & array,
  char const * name,
  vtkIdType numberOfElements,
  vtkPolyData * polyData)
{
  array = T::New();
  array->SetNumberOfValues(numberOfElements);
  array->SetName(name);
  polyData->GetPointData()->AddArray(array);
}

//------------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData>
vtkVelodyneAdvancedPacketInterpreter::CreateNewEmptyFrame(
  vtkIdType numberOfPoints,
  vtkIdType vtkNotUsed(prereservedNumberOfPoints))
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  this->UpdateMaxFrameSize(numberOfPoints);

  // Points.
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(this->MaxFrameSize);
  // Same name as vtkVelodyneHDLReader.
  points->GetData()->SetName("Points_m_XYZ");

  // Replace the old points.
  this->Points = points.GetPointer();

  // Point the polyData to the points.
  polyData->SetPoints(points.GetPointer());

  // Replace and initialize all of the associated data arrays.

//! @brief Convencience macro for initializing info arrays.
#define VAPI_INIT_INFO_ARR(arr_name, disp_name)                                \
  InitializeDataArrayForPolyData(                                              \
    this->INFO_##arr_name, disp_name, this->MaxFrameSize, polyData);

  VAPI_INIT_INFO_ARR(Xs, "X")
  VAPI_INIT_INFO_ARR(Ys, "Y")
  VAPI_INIT_INFO_ARR(Zs, "Z")
  VAPI_INIT_INFO_ARR(Distances, "distance_m")
  VAPI_INIT_INFO_ARR(RawDistances, "distance_raw")
  VAPI_INIT_INFO_ARR(DistanceTypes, "distance_type")
  VAPI_INIT_INFO_ARR(Azimuths, "azimuth")
  VAPI_INIT_INFO_ARR(VerticalAngles, "vertical_angle")
/*
  VAPI_INIT_INFO_ARR(DistanceTypeStrings  , "distance_type_name")
  VAPI_INIT_INFO_ARR(FiringModeStrings    , "firing_mode")
  VAPI_INIT_INFO_ARR(StatusStrings        , "status")
*/
  VAPI_INIT_INFO_ARR(Intensities, "intensity")
  VAPI_INIT_INFO_ARR(Confidences, "confidence")
  VAPI_INIT_INFO_ARR(Reflectivities, "reflectivity")
  VAPI_INIT_INFO_ARR(ChannelNumbers, "logical_channel_number")
  VAPI_INIT_INFO_ARR(TimeFractionOffsets, "time_fraction_offset")
  VAPI_INIT_INFO_ARR(Timestamps, "timestamp")
  VAPI_INIT_INFO_ARR(Powers, "power")
  VAPI_INIT_INFO_ARR(Noises, "noise")
  VAPI_INIT_INFO_ARR(Pseqs, "packet_sequence_number")

  this->NumberOfPointsInCurrentFrame = 0;
  this->CurrentArraySize             = numberOfPoints;
  return polyData;
}


//------------------------------------------------------------------------------
// TODO: Revisit this if the frequency still needs to be calculated here.
bool
vtkVelodyneAdvancedPacketInterpreter::SplitFrame(bool force)
{
  auto numberOfAllocatedPoints = this->Points->GetNumberOfPoints();
  // Update the MaxId to the current number of points.
  this->SetNumberOfItems(this->NumberOfPointsInCurrentFrame);
  // this->CurrentFrame->Modified();
  bool wasSplit = this->vtkLidarPacketInterpreter::SplitFrame(force);
  // If the frame was split then CreateNewEmptyDataFrame was called and the
  // array sizes have already been adjusted. If not, we need to reset the MaxId
  // to allow for further insertions.
  if (!wasSplit)
  {
    this->SetNumberOfItems(numberOfAllocatedPoints);
  }
  return wasSplit;
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::ResetCurrentFrame()
{
  this->CurrentFrame = this->CreateNewEmptyFrame(0);
  this->CurrentFrameTracker->Reset();
  this->Frames.clear();
}

//------------------------------------------------------------------------------
bool
vtkVelodyneAdvancedPacketInterpreter::PreProcessPacket(
  const unsigned char * data,
  unsigned int dataLength,
  fpos_t filePosition,
  double packetNetworkTime,
  std::vector<FrameInformation> * frameCatalog)
{
  this->ParserMetaData.FilePosition           = filePosition;
  this->ParserMetaData.FirstPacketNetworkTime = packetNetworkTime;
  //! @todo
  //  this->ParserMetaData.FirstPacketDataTime = packetNetworkTime;
  auto * velFrameInfo =
    reinterpret_cast<VelodyneSpecificFrameInformation *>(
      this->ParserMetaData.SpecificInformation.get());
  //  if (dataPacket->gpsTimestamp < this->lastGpsTimestamp)
  //  {
  //    velFrameInfo->NbrOfRollingTime++;
  //  }

  decltype(dataLength) index = 0;
  PayloadHeader const * payloadHeader =
    reinterpretCastWithChecks<PayloadHeader>(data, dataLength, index);
  if ((payloadHeader == nullptr) || (payloadHeader->GetDistanceCount() == 0))
  {
    return false;
  }
  ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, payloadHeader, false)

  // Skip optional extension headers.
  auto nxhdr = payloadHeader->GetNxhdr();
  while (nxhdr != 0)
  {
    ExtensionHeader const * extensionHeader =
      reinterpretCastWithChecks<ExtensionHeader>(data, dataLength, index);
    if (extensionHeader == nullptr)
    {
      return false;
    }
    ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, extensionHeader, false)
    nxhdr = extensionHeader->GetNxhdr();
  }

  // Loop through firing groups until a frame shift is detected.
  size_t numberOfBytesPerFiringGroupHeader = payloadHeader->GetGlen();
  size_t numberOfBytesPerFiring = payloadHeader->GetNumberOfBytesPerFiring();
  int firingCount               = 0;
  bool isNewFrame               = false;
  while (index < dataLength)
  {
    align_to_word_size(index);
    FiringGroupHeader const * firingGroupHeader =
      reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);
    if (firingGroupHeader == nullptr)
    {
      return isNewFrame;
    }
    // The payload header checks above ensure that this value is non-zero and
    // that the loop will therefore eventually terminate.
    isNewFrame = this->CurrentFrameTracker->Update(payloadHeader, firingGroupHeader);

    if (isNewFrame)
    {
      velFrameInfo->FiringToSkip = firingCount;
      frameCatalog->push_back(this->ParserMetaData);
      // Create a copy of the current meta data state
      // at a different memory location than the one
      // added to the catalog
      return isNewFrame;
    }
    firingCount++;
    index += (numberOfBytesPerFiring * firingGroupHeader->GetFcnt()) +
             numberOfBytesPerFiringGroupHeader;
  }
  return isNewFrame;
}


//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::UpdateMaxFrameSize(size_t frameSize)
{
  if (frameSize > this->MaxFrameSize)
  {
    size_t difference = frameSize - this->MaxFrameSize;
    this->MaxFrameSize +=
      ((difference + (MEM_STEP_SIZE - 1)) / MEM_STEP_SIZE) * MEM_STEP_SIZE;
  }
}

//------------------------------------------------------------------------------
// Macro-based methods.
//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::ResizeArrays()
{
  size_t newSize     = this->MaxFrameSize;
  size_t currentSize = this->CurrentArraySize;
  if (newSize <= currentSize)
  {
    return;
  }

  this->Points->Resize(newSize);

#define VAPI_RESIZE(index, data, array) array->Resize(newSize);

  // "data" is an unused placeholder
  VAPI_FOREACH_INFO_ARRAY(VAPI_RESIZE, data)

  this->CurrentArraySize = newSize;
}
//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::SetNumberOfItems(size_t numberOfItems)
{
  this->UpdateMaxFrameSize(numberOfItems);
  if (numberOfItems > static_cast<size_t>(this->Points->GetNumberOfPoints()))
  {
    this->ResizeArrays();
  }

  this->Points->SetNumberOfPoints(numberOfItems);

#define VAPI_SET_NUMBER_OF_VALUES(index, data, array)                          \
  array->SetNumberOfValues(numberOfItems);

  VAPI_FOREACH_INFO_ARRAY(VAPI_SET_NUMBER_OF_VALUES, data)
}

//------------------------------------------------------------------------------
void vtkVelodyneAdvancedPacketInterpreter::LoadCalibration(const std::string& filename)
{
  this->vtkVelodyneBasePacketInterpreter::LoadCalibration(filename);
  this->CurrentFrameTracker->SetFramingLogic(this->GetFramingLogic());
}

