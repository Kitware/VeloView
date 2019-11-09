#include "vtkVelodyneLegacyPacketInterpreter.h"

#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkTransform.h>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include "vtkDataPacket.h"
#include "vtkRollingDataAccumulator.h"
#include "vtkVelodyneMetaPacketInterpreter.h"

using namespace DataPacketFixedLength;

#define PacketProcessingDebugMacro(x)                                                              \
  {                                                                                                \
    if (this->OutputPacketProcessingDebugInfo)                                                     \
    {                                                                                              \
      std::cout << " " x;                                                                          \
    }                                                                                              \
  }

//namespace
//{
//! @todo this method are actually usefull for every Interpreter and should go to the top
template<typename T>
vtkSmartPointer<T> CreateDataArray(const char* name, vtkIdType np, vtkIdType prereserved_np, vtkPolyData* pd)
{
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->Allocate(prereserved_np);
  array->SetName(name);
  if (np > 0)
  {
    array->SetNumberOfTuples(np);
  }
  if (pd)
  {
    pd->GetPointData()->AddArray(array);
  }

  return array;
}

// Structure to compute RPM and handle degenerated cases
struct RPMCalculator
{
  // Determines if the rpm computation is available
  bool IsReady;
  // Determines if the corresponding value has been set
  bool ValueReady[4];
  int MinAngle;
  int MaxAngle;
  unsigned int MinTime;
  unsigned int MaxTime;

  void Reset()
  {
    this->IsReady = false;
    this->ValueReady[0] = false;
    this->ValueReady[1] = false;
    this->ValueReady[2] = false;
    this->ValueReady[3] = false;
    this->MinAngle = std::numeric_limits<int>::max();
    this->MaxAngle = std::numeric_limits<int>::min();
    this->MinTime = std::numeric_limits<unsigned int>::max();
    this->MaxTime = std::numeric_limits<unsigned int>::min();
  }

  double GetRPM()
  {
    // If the calculator is not ready i.e : one
    // of the attributes is not initialized yet
    // (MaxAngle, MinAngle, MaxTime, MinTime)
    if (!this->IsReady)
    {
      return 0;
    }

    // delta angle in number of laps
    double dAngle = static_cast<double>(this->MaxAngle - this->MinAngle) / (100.0 * 360.0);

    // delta time in minutes
    double dTime = static_cast<double>(this->MaxTime - this->MinTime) / (60e6);

    // epsilon to test if the delta time / angle is not too small
    const double epsilon = 1e-12;

    // if one the deltas is too small
    if ((std::abs(dTime) < epsilon) || (std::abs(dAngle) < epsilon))
    {
      return 0;
    }

    return dAngle / dTime;
  }

  void AddData(const HDLDataPacket* HDLPacket, unsigned int rawtime)
  {
    if (HDLPacket->firingData[0].getRotationalPosition() < this->MinAngle)
    {
      this->MinAngle = HDLPacket->firingData[0].getRotationalPosition();
      this->ValueReady[0] = true;
    }
    if (HDLPacket->firingData[0].getRotationalPosition() > this->MaxAngle)
    {
      this->MaxAngle = HDLPacket->firingData[0].getRotationalPosition();
      this->ValueReady[1] = true;
    }
    if (rawtime < this->MinTime)
    {
      this->MinTime = rawtime;
      this->ValueReady[2] = true;
    }
    if (rawtime > this->MaxTime)
    {
      this->MaxTime = rawtime;
      this->ValueReady[3] = true;
    }

    // Check if all of the 4th parameters
    // have been set
    this->IsReady = true;
    for (int k = 0; k < 4; ++k)
    {
      this->IsReady &= this->ValueReady[k];
    }
  }
};

//-----------------------------------------------------------------------------
int MapFlags(unsigned int flags, unsigned int low, unsigned int high)
{
  return (flags & low ? -1 : flags & high ? 1 : 0);
}

//-----------------------------------------------------------------------------
int MapDistanceFlag(unsigned int flags)
{
  return MapFlags(flags & vtkVelodyneMetaPacketInterpreter::DUAL_DISTANCE_MASK,
    vtkVelodyneMetaPacketInterpreter::DUAL_DISTANCE_NEAR, vtkVelodyneMetaPacketInterpreter::DUAL_DISTANCE_FAR);
}

//-----------------------------------------------------------------------------
int MapIntensityFlag(unsigned int flags)
{
  return MapFlags(flags & vtkVelodyneMetaPacketInterpreter::DUAL_INTENSITY_MASK,
    vtkVelodyneMetaPacketInterpreter::DUAL_INTENSITY_LOW, vtkVelodyneMetaPacketInterpreter::DUAL_INTENSITY_HIGH);
}

//-----------------------------------------------------------------------------
double HDL32AdjustTimeStamp(int firingblock, int dsr, const bool isDualReturnMode)
{
  if (!isDualReturnMode)
  {
    return (firingblock * 46.08) + (dsr * 1.152);
  }
  else
  {
    return (firingblock / 2 * 46.08) + (dsr * 1.152);
  }
}

//-----------------------------------------------------------------------------
double VLP16AdjustTimeStamp(
  int firingblock, int dsr, int firingwithinblock, const bool isDualReturnMode)
{
  if (!isDualReturnMode)
  {
    return (firingblock * 110.592) + (dsr * 2.304) + (firingwithinblock * 55.296);
  }
  else
  {
    return (firingblock / 2 * 110.592) + (dsr * 2.304) + (firingwithinblock * 55.296);
  }
}

//-----------------------------------------------------------------------------
double VLP32AdjustTimeStamp(int firingblock, int dsr, const bool isDualReturnMode)
{
  if (!isDualReturnMode)
  {
    return (firingblock * 55.296) + (dsr / 2) * 2.304;
  }
  else
  {
    return (firingblock / 2 * 55.296) + (dsr / 2) * 2.304;
  }
}

//-----------------------------------------------------------------------------
double HDL64EAdjustTimeStamp(int firingblock, int dsr, const bool isDualReturnMode)
{
  const int dsrReversed = HDL_LASER_PER_FIRING - dsr - 1;
  const int firingblockReversed = HDL_FIRING_PER_PKT - firingblock - 1;
  if (!isDualReturnMode)
  {
    const double TimeOffsetMicroSec[4] = { 2.34, 3.54, 4.74, 6.0 };
    return (std::floor(static_cast<double>(firingblockReversed) / 2.0) * 48.0) +
      TimeOffsetMicroSec[(dsrReversed % 4)] + (dsrReversed / 4) * TimeOffsetMicroSec[3];
  }
  else
  {
    const double TimeOffsetMicroSec[4] = { 3.5, 4.7, 5.9, 7.2 };
    return (std::floor(static_cast<double>(firingblockReversed) / 4.0) * 57.6) +
      TimeOffsetMicroSec[(dsrReversed % 4)] + (dsrReversed / 4) * TimeOffsetMicroSec[3];
  }
}

//-----------------------------------------------------------------------------
double VLS128AdjustTimeStamp(int firingblock, int dsrBase32, const bool isDualReturnMode)
{
  const static double dt = 2.665;
  const static double firingblock_num_cycles = 20;
  const static double firingblock_duration = (dt * firingblock_num_cycles);
  const static int n_blocks_per_firing = 4;

  int dsr = (dsrBase32 + 32 * (firingblock % n_blocks_per_firing));

  //dsr >= 64 needs an additional two cycles of delay to account for interleaved maintenance cycles

  if (!isDualReturnMode)
  {
    return (firingblock_duration * static_cast<int>(firingblock / n_blocks_per_firing)) +
      (static_cast<int>(dsr / 8) + (static_cast<int>(dsr / 64) * 2)) * dt;
  }
  else
  {
    return (firingblock_duration * static_cast<int>(firingblock / (n_blocks_per_firing * 2))) +
      (static_cast<int>(dsr / 8) + (static_cast<int>(dsr / 64) * 2)) * dt;
  }
}

//-----------------------------------------------------------------------------
double VelArrayAdjustTimeStamp(
  unsigned int firingblock, unsigned int dsr, const bool isDualReturnMode)
{
  const static double dt = 2.304;
  const static double firingblock_num_cycles = 18;
  const static double firingblock_duration = (dt * firingblock_num_cycles);
  const static int n_simultaneous_firing = 2;
  if (!isDualReturnMode)
  {
    return (firingblock * firingblock_duration) +
      static_cast<int>(dsr / n_simultaneous_firing) * dt;
  }
  else
  {
    return (firingblock_duration * static_cast<unsigned int>(firingblock / 2)) +
      static_cast<int>(dsr / n_simultaneous_firing) * dt;
  }
}

//-----------------------------------------------------------------------------
class FramingState
{
  int LastAzimuthDir;
  int LastElevationDir;
  int LastElevation;
  int LastAzimuthSlope;

public:
  FramingState() { reset(); }
  void reset()
  {
    LastAzimuthDir = -1;
    LastElevationDir = -1;
    LastElevation = -1;
    LastAzimuthSlope = 0;
  }
  bool hasChangedWithValue(const HDLFiringData& firingData)
  {
    bool hasLastAzimuth = (LastAzimuthDir != -1);
    // bool azimuthFrameSplit = hasChangedWithValue(
    //  firingData.getRotationalPosition(), hasLastAzimuth, LastAzimuth, LastAzimuthSlope);
    if (!firingData.isVelArrayFiring())
    {
      bool azimuthFrameSplit = hasChangedWithValue(
        firingData.getRotationalPosition(), hasLastAzimuth, LastAzimuthDir, LastAzimuthSlope);
        return azimuthFrameSplit;
    }
    bool azimuthFrameSplit =
      hasLastAzimuth ? (firingData.getScanningHorizontalDir() != LastAzimuthDir) : false;
    LastAzimuthDir = firingData.getScanningHorizontalDir();

    bool hasLastElevation = (LastElevationDir != -1);
    int previousElevation = LastElevation;
    bool elevationSplit =
      hasLastElevation ? (firingData.getScanningVerticalDir() != LastElevationDir) : false;
    LastElevationDir = firingData.getScanningVerticalDir();
    LastElevation = firingData.getElevation100th();

    return elevationSplit;
    if (azimuthFrameSplit)
    {
      if (firingData.getElevation100th() == previousElevation)
      {
        // Change of azimuth scanning direction, without a elevation change
        // Either a double sweep with same elevation or fixed elevation
        // That is a new frame. Reset the elevationSlope
        LastElevationDir = -1;
        return true;
      }
      return elevationSplit;
    }
    return azimuthFrameSplit;
  }

  static bool hasChangedWithValue(int curValue, bool& hasLastValue, int& lastValue, int& lastSlope)
  {
    // If we dont have previous value, dont change
    if (!hasLastValue)
    {
      lastValue = curValue;
      hasLastValue = true;
      return false;
    }
    int curSlope = curValue - lastValue;
    lastValue = curValue;
    if (curSlope == 0)
      return false;
    int isSlopeSameDirection = curSlope * lastSlope;
    // curSlope has same sign as lastSlope: no change
    if (isSlopeSameDirection > 0)
      return false;
    // curSlope has different sign as lastSlope: change!
    else if (isSlopeSameDirection < 0)
    {
      lastSlope = 0;
      return true;
    }
    // LastAzimuthSlope not set: set the slope
    if (lastSlope == 0 && curSlope != 0)
    {
      lastSlope = curSlope;
      return false;
    }
    vtkGenericWarningMacro("Unhandled sequence of value in state.");
    return false;
  }

  static bool willChangeWithValue(int curValue, bool hasLastValue, int lastValue, int lastSlope)
  {
    return hasChangedWithValue(curValue, hasLastValue, lastValue, lastSlope);
  }
};

//} // End namespace

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneLegacyPacketInterpreter)

//-----------------------------------------------------------------------------
vtkVelodyneLegacyPacketInterpreter::vtkVelodyneLegacyPacketInterpreter()
{
  this->RpmCalculator_ = new RPMCalculator();
  this->UseIntraFiringAdjustment = true;
  this->alreadyWarnedForIgnoredHDL64FiringPacket = false;
  this->OutputPacketProcessingDebugInfo = false;
  this->SensorPowerMode = 0;
  this->CurrentFrameState = new FramingState;
  this->LastTimestamp = std::numeric_limits<unsigned int>::max();
  this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
  this->FiringsSkip = 0;
  this->ShouldCheckSensor = true;

  std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);

  this->LaserSelection.resize(HDL_MAX_NUM_LASERS, true);
  this->DualReturnFilter = 0;
  this->IsHDL64Data = false;
  this->ReportedFactoryField1 = 0;
  this->ReportedFactoryField2 = 0;
  this->DistanceResolutionM = 0.002;
  this->WantIntensityCorrection = false;
  this->lastGpsTimestamp = 0;
  this->ParserMetaData.SpecificInformation = std::make_shared<VelodyneSpecificFrameInformation>();

  this->rollingCalibrationData = new vtkRollingDataAccumulator();
  this->Init();
}

vtkVelodyneLegacyPacketInterpreter::~vtkVelodyneLegacyPacketInterpreter()
{
  if (this->rollingCalibrationData)
  {
    delete this->rollingCalibrationData;
  }
  delete this->CurrentFrameState;
}

//-----------------------------------------------------------------------------
void vtkVelodyneLegacyPacketInterpreter::ProcessPacket(unsigned char const * data, unsigned int dataLength)
{
  if (!this->IsLidarPacket(data, dataLength))
  {
    return;
  }

  const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket*>(data);

  this->IsHDL64Data |= dataPacket->isHDL64();

  // Accumulate HDL64 Status byte data
  if (IsHDL64Data && this->IsCorrectionFromLiveStream &&
    !this->IsCalibrated)
  {
    this->rollingCalibrationData->appendData(dataPacket->gpsTimestamp, dataPacket->factoryField1, dataPacket->factoryField2);
    this->HDL64LoadCorrectionsFromStreamData(this->rollingCalibrationData);
    return;
  }

  if (this->ShouldCheckSensor)
  {
    this->CheckReportedSensorAndCalibrationFileConsistent(dataPacket);
    ShouldCheckSensor = false;
  }

  // Check if the time has rolled during this packet
  if (dataPacket->gpsTimestamp < this->ParserMetaData.FirstPacketDataTime)
  {
    reinterpret_cast<VelodyneSpecificFrameInformation*>
        (this->ParserMetaData.SpecificInformation.get())->NbrOfRollingTime++;
  }
  this->ParserMetaData.FirstPacketDataTime = dataPacket->gpsTimestamp;

  const unsigned int rawtime = dataPacket->gpsTimestamp;
  const double timestamp = this->ComputeTimestamp(dataPacket->gpsTimestamp, this->ParserMetaData);

  // Update the rpm computation (by packets)
  this->RpmCalculator_->AddData(dataPacket, rawtime);

  // Update the transforms here and then call internal
  // transform
  if (SensorTransform) this->SensorTransform->Update();

  VelodyneSpecificFrameInformation* velodyneFrameInfo =
      reinterpret_cast<VelodyneSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());
  int firingBlock = velodyneFrameInfo->FiringToSkip;
  velodyneFrameInfo->FiringToSkip = 0;

  bool isVLS128 = dataPacket->isVLS128();
  // Compute the list of total azimuth advanced during one full firing block
  std::vector<int> diffs(HDL_FIRING_PER_PKT - 1);
  for (int i = 0; i < HDL_FIRING_PER_PKT - 1; ++i)
  {
    int localDiff = 0;

    if (isVLS128)
    {
        localDiff = (36000 + 18000 + dataPacket->firingData[i + 4].getRotationalPosition() -
                      dataPacket->firingData[i].getRotationalPosition()) %
                      36000 - 18000;
    }
    else
    {
        localDiff = (36000 + 18000 + dataPacket->firingData[i + 1].getRotationalPosition() -
                      dataPacket->firingData[i].getRotationalPosition()) %
                      36000 - 18000;
    }
    diffs[i] = localDiff;
  }


  if (!IsHDL64Data)
  { // with HDL64, it should be filled by LoadCorrectionsFromStreamData
    this->ReportedSensor = dataPacket->getSensorType();
    this->ReportedSensorReturnMode = dataPacket->getDualReturnSensorMode();
  }

  std::sort(diffs.begin(), diffs.end());
  // Assume the median of the packet's rotationalPosition differences
  int azimuthDiff = diffs[HDL_FIRING_PER_PKT / 2];
  if (this->IsHDL64Data)
  {
    azimuthDiff = diffs[HDL_FIRING_PER_PKT - 2];
  }
  else if (isVLS128)
  {
    azimuthDiff = diffs[HDL_FIRING_PER_PKT - 1];
  }

  // assert(azimuthDiff > 0);

  // Add DualReturn-specific arrays if newly detected dual return packet
  if (dataPacket->isDualModeReturn() && !this->HasDualReturn)
  {
    this->HasDualReturn = true;
    this->CurrentFrame->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
    this->CurrentFrame->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
    this->CurrentFrame->GetPointData()->AddArray(this->DualReturnMatching.GetPointer());
  }

  for (; firingBlock < HDL_FIRING_PER_PKT; ++firingBlock)
  {
    const HDLFiringData* firingData = &(dataPacket->firingData[firingBlock]);
    // clang-format off
    int multiBlockLaserIdOffset =
        (firingData->blockIdentifier == BLOCK_0_TO_31)  ?  0 :(
        (firingData->blockIdentifier == BLOCK_32_TO_63) ? 32 :(
        (firingData->blockIdentifier == BLOCK_64_TO_95) ? 64 :(
        (firingData->blockIdentifier == BLOCK_96_TO_127)? 96 :(
                                                           0))));
    // clang-format on

    // Skip dummy blocks of VLS-128 dual mode last 4 blocks
    if (isVLS128 && (firingData->blockIdentifier == 0 || firingData->blockIdentifier == 0xFFFF))
    {
      continue;
    }


    if (this->CurrentFrameState->hasChangedWithValue(*firingData))
    {
      this->SplitFrame();
      this->LastTimestamp = std::numeric_limits<unsigned int>::max();
    }

    if (isVLS128)
    {
      azimuthDiff = dataPacket->getRotationalDiffForVLS128(firingBlock);
    }

    // For variable speed sensors, get this firing block exact azimuthDiff
    if (dataPacket->getSensorType() == VelArray)
    {
      azimuthDiff = dataPacket->getRotationalDiffForVelarrayFiring(firingBlock);
      // only use the azimuth-diff between firings if below 2 degrees
      if (abs(azimuthDiff) > 200)
        azimuthDiff = 0;
    }
    // Skip this firing every PointSkip
    if (this->FiringsSkip == 0 || firingBlock % (this->FiringsSkip + 1) == 0)
    {
      this->ProcessFiring(firingData, multiBlockLaserIdOffset, firingBlock, azimuthDiff, timestamp,
        rawtime, dataPacket->isDualReturnFiringBlock(firingBlock), dataPacket->isDualModeReturn());
    }
  }
}

//-----------------------------------------------------------------------------
bool vtkVelodyneLegacyPacketInterpreter::IsLidarPacket(unsigned char const * vtkNotUsed(data), unsigned int dataLength)
{
  if (dataLength == HDLDataPacket::getDataByteLength())
  {
    // Data-Packet Specifications says that laser-packets are 1206 byte long.
    //  That is : (2+2+(2+1)*32)*12 + 4 + 1 + 1
    //                #lasers^   ^#firingPerPkt
    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
void vtkVelodyneLegacyPacketInterpreter::ProcessFiring(const HDLFiringData *firingData, int firingBlockLaserOffset, int firingBlock, int azimuthDiff, double timestamp, unsigned int rawtime, bool isThisFiringDualReturnData, bool isDualReturnPacket)
{
  // First return block of a dual return packet: init last point of laser
  if (!isThisFiringDualReturnData &&
    (!this->IsHDL64Data || (this->IsHDL64Data && ((firingBlock % 4) == 0))))
  {
    this->FirstPointIdOfDualReturnPair = this->Points->GetNumberOfPoints();
  }

  unsigned short firingElevation100th = firingData->getElevation100th();

  for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++)
  {
    const unsigned char rawLaserId = static_cast<unsigned char>(dsr + firingBlockLaserOffset);
    unsigned char laserId = rawLaserId;
    const unsigned short azimuth = firingData->getRotationalPosition();

    // Detect VLP-16 data and adjust laser id if necessary
    int firingWithinBlock = 0;

    if (this->CalibrationReportedNumLasers == 16)
    {
      if (firingBlockLaserOffset != 0)
      {
        if (!this->alreadyWarnedForIgnoredHDL64FiringPacket)
        {
          vtkGenericWarningMacro("Error: Received a HDL-64 UPPERBLOCK firing packet "
                                 "with a VLP-16 calibration file. Ignoring the firing.");
          this->alreadyWarnedForIgnoredHDL64FiringPacket = true;
        }
        return;
      }
      if (laserId >= 16)
      {
        laserId -= 16;
        firingWithinBlock = 1;
      }
    }

    // Interpolate azimuths and timestamps per laser within firing blocks
    double timestampadjustment = 0;
    int azimuthadjustment = 0;
    if (this->UseIntraFiringAdjustment)
    {
      double blockdsr0 = 0, nextblockdsr0 = 1;
      switch (this->CalibrationReportedNumLasers)
      {
        case 128:
        {
          timestampadjustment = VLS128AdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);

          // dsr0 occurs every fourth block on VLS-128
          nextblockdsr0 = VLS128AdjustTimeStamp(
            (firingBlock / 4) * 4 + (isDualReturnPacket ? 8 : 4), 0, isDualReturnPacket);
          blockdsr0 = VLS128AdjustTimeStamp((firingBlock / 4) * 4, 0, isDualReturnPacket);
          break;
        }
        case 64:
        {
          timestampadjustment = -HDL64EAdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
          nextblockdsr0 = -HDL64EAdjustTimeStamp(
            firingBlock + (isDualReturnPacket ? 4 : 2), 0, isDualReturnPacket);
          blockdsr0 = -HDL64EAdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
          break;
        }
        case 32:
        {
          if (this->ReportedSensor == VLP32AB || this->ReportedSensor == VLP32C)
          {
            timestampadjustment = VLP32AdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
            nextblockdsr0 = VLP32AdjustTimeStamp(
              firingBlock + (isDualReturnPacket ? 2 : 1), 0, isDualReturnPacket);
            blockdsr0 = VLP32AdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
          }
          else if (this->ReportedSensor == VelArray)
          {
            timestampadjustment = VelArrayAdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
            nextblockdsr0 = VelArrayAdjustTimeStamp(
              firingBlock + (isDualReturnPacket ? 2 : 1), 0, isDualReturnPacket);
            blockdsr0 = VelArrayAdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
          }
          else
          {
            timestampadjustment = HDL32AdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
            nextblockdsr0 = HDL32AdjustTimeStamp(
              firingBlock + (isDualReturnPacket ? 2 : 1), 0, isDualReturnPacket);
            blockdsr0 = HDL32AdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
          }
          break;
        }
        case 16:
        {
          timestampadjustment =
            VLP16AdjustTimeStamp(firingBlock, laserId, firingWithinBlock, isDualReturnPacket);
          nextblockdsr0 = VLP16AdjustTimeStamp(
            firingBlock + (isDualReturnPacket ? 2 : 1), 0, 0, isDualReturnPacket);
          blockdsr0 = VLP16AdjustTimeStamp(firingBlock, 0, 0, isDualReturnPacket);
          break;
        }
        default:
        {
          timestampadjustment = 0.0;
          blockdsr0 = 0.0;
          nextblockdsr0 = 1.0;
        }
      }
      azimuthadjustment = vtkMath::Round(
        azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
      timestampadjustment = vtkMath::Round(timestampadjustment);
    }
    if ((!this->IgnoreZeroDistances || firingData->laserReturns[dsr].distance != 0.0) &&
      this->LaserSelection[laserId])
    {
      const unsigned short adjustedAzimuth =
        (36000 + (static_cast<int>(azimuth) + azimuthadjustment)) % 36000;
      this->PushFiringData(laserId, rawLaserId, adjustedAzimuth, firingElevation100th,
        timestamp + timestampadjustment, rawtime + static_cast<unsigned int>(timestampadjustment),
        &(firingData->laserReturns[dsr]), dsr + firingBlockLaserOffset,
        isThisFiringDualReturnData);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneLegacyPacketInterpreter::PushFiringData(unsigned char laserId,
              const unsigned char rawLaserId, unsigned short azimuth, const unsigned short firingElevation100th,
              const double timestamp, const unsigned int rawtime, const HDLLaserReturn* laserReturn,
              const unsigned int channelNumber,
              const bool isFiringDualReturnData)
{
  azimuth %= 36000;
  const vtkIdType thisPointId = this->Points->GetNumberOfPoints();
  // double firingElevation = static_cast<double>(firingElevation100th) / 100.0;

  // Compute raw position
  bool applyIntensityCorrection =
    this->WantIntensityCorrection && this->IsHDL64Data && !(this->SensorPowerMode == CorrectionOn);

  RawValues rawValues(azimuth, firingElevation100th, laserReturn->distance, laserReturn->intensity);
  CorrectedValues correctedValues;

  this->ComputeCorrectedValues(
      rawValues,
      channelNumber,
      correctedValues,
      applyIntensityCorrection
    );

  double & distanceM = correctedValues.distance;;
  short intensity = correctedValues.intensity;
  double (& pos)[3] = correctedValues.position;

  // Apply sensor transform
//  cout << "this->SensorTransform" << this->SensorTransform->GetPosition()[0]
// << " " << this->SensorTransform->GetPosition()[0] << " " << this->SensorTransform->GetPosition()[0] << endl;
  if (SensorTransform) this->SensorTransform->InternalTransformPoint(pos, pos);

  if (this->shouldBeCroppedOut(pos))
    return;

  // Do not add any data before here as this might short-circuit
  if (isFiringDualReturnData)
  {
    const vtkIdType dualPointId = this->LastPointId[rawLaserId];
    if (dualPointId < this->FirstPointIdOfDualReturnPair)
    {
      // No matching point from first set (skipped?)
      this->Flags->InsertNextValue(DUAL_DOUBLED);
      this->DistanceFlag->InsertNextValue(0);
      this->DualReturnMatching->InsertNextValue(-1); // std::numeric_limits<vtkIdType>::quiet_NaN()
      this->IntensityFlag->InsertNextValue(0);
    }
    else
    {
      const short dualIntensity = this->Intensity->GetValue(dualPointId);
      const double dualDistance = this->Distance->GetValue(dualPointId);
      unsigned int firstFlags = this->Flags->GetValue(dualPointId);
      unsigned int secondFlags = 0;

      if (dualDistance == distanceM && intensity == dualIntensity)
      {
        // ignore duplicate point and leave first with original flags
        return;
      }

      if (dualIntensity < intensity)
      {
        firstFlags &= ~DUAL_INTENSITY_HIGH;
        secondFlags |= DUAL_INTENSITY_HIGH;
      }
      else
      {
        firstFlags &= ~DUAL_INTENSITY_LOW;
        secondFlags |= DUAL_INTENSITY_LOW;
      }

      if (dualDistance < distanceM)
      {
        firstFlags &= ~DUAL_DISTANCE_FAR;
        secondFlags |= DUAL_DISTANCE_FAR;
      }
      else
      {
        firstFlags &= ~DUAL_DISTANCE_NEAR;
        secondFlags |= DUAL_DISTANCE_NEAR;
      }

      // We will output only one point so return out of this
      if (this->DualReturnFilter)
      {
        if (!(secondFlags & this->DualReturnFilter))
        {
          // second return does not match filter; skip
          this->Flags->SetValue(dualPointId, firstFlags);
          this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(firstFlags));
          this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(firstFlags));
          return;
        }
        if (!(firstFlags & this->DualReturnFilter))
        {
          // first return does not match filter; replace with second return
          this->Points->SetPoint(dualPointId, pos);
          this->Distance->SetValue(dualPointId, distanceM);
          this->DistanceRaw->SetValue(dualPointId, laserReturn->distance);
          this->Intensity->SetValue(dualPointId, intensity);
          this->Timestamp->SetValue(dualPointId, timestamp);
          this->RawTime->SetValue(dualPointId, rawtime);
          this->Flags->SetValue(dualPointId, secondFlags);
          this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(secondFlags));
          this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(secondFlags));
          return;
        }
      }

      this->Flags->SetValue(dualPointId, firstFlags);
      this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(firstFlags));
      this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(firstFlags));
      this->Flags->InsertNextValue(secondFlags);
      this->DistanceFlag->InsertNextValue(MapDistanceFlag(secondFlags));
      this->IntensityFlag->InsertNextValue(MapIntensityFlag(secondFlags));
      // The first return indicates the dual return
      // and the dual return indicates the first return
      this->DualReturnMatching->InsertNextValue(dualPointId);
      this->DualReturnMatching->SetValue(dualPointId, thisPointId);
    }
  }
  else
  {
    this->Flags->InsertNextValue(DUAL_DOUBLED);
    this->DistanceFlag->InsertNextValue(0);
    this->IntensityFlag->InsertNextValue(0);
    this->DualReturnMatching->InsertNextValue(-1); // std::numeric_limits<vtkIdType>::quiet_NaN()
  }

  this->Points->InsertNextPoint(pos);
  this->PointsX->InsertNextValue(pos[0]);
  this->PointsY->InsertNextValue(pos[1]);
  this->PointsZ->InsertNextValue(pos[2]);
  this->Azimuth->InsertNextValue(azimuth);
  this->Intensity->InsertNextValue(intensity);
  this->LaserId->InsertNextValue(laserId);
  this->Timestamp->InsertNextValue(timestamp);
  this->RawTime->InsertNextValue(rawtime);
  this->Distance->InsertNextValue(distanceM);
  this->DistanceRaw->InsertNextValue(laserReturn->distance);
  this->LastPointId[rawLaserId] = thisPointId;
  this->VerticalAngle->InsertNextValue(correctedValues.elevation);
}

//-----------------------------------------------------------------------------
void vtkVelodyneLegacyPacketInterpreter::Init()
{
  this->InitTrigonometricTables();
  this->ResetCurrentFrame();
}

//-----------------------------------------------------------------------------
double vtkVelodyneLegacyPacketInterpreter::ComputeTimestamp(unsigned int tohTime, const FrameInformation& frameInfo)
{
  VelodyneSpecificFrameInformation* velInfo = reinterpret_cast<VelodyneSpecificFrameInformation*>(frameInfo.SpecificInformation.get());
  static const double hourInMilliseconds = 3600.0 * 1e6;
  return tohTime + velInfo->NbrOfRollingTime * hourInMilliseconds;
}



//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneLegacyPacketInterpreter::CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints)
{
  const int defaultPrereservedNumberOfPointsPerFrame = 60000;
  // prereserve for 50% points more than actually received in previous frame
  prereservedNumberOfPoints = std::max(static_cast<int>(prereservedNumberOfPoints * 1.5), defaultPrereservedNumberOfPointsPerFrame);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // points
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->Allocate(prereservedNumberOfPoints);
  if (numberOfPoints > 0 )
  {
    points->SetNumberOfPoints(numberOfPoints);
  }
  points->GetData()->SetName("Points_m_XYZ");
  polyData->SetPoints(points.GetPointer());
//  polyData->SetVerts(NewVertexCells(numberOfPoints));

  // intensity
  this->Points = points.GetPointer();
  this->PointsX = CreateDataArray<vtkDoubleArray>("X", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->PointsY = CreateDataArray<vtkDoubleArray>("Y", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->PointsZ = CreateDataArray<vtkDoubleArray>("Z", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Intensity = CreateDataArray<vtkUnsignedCharArray>("intensity", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->LaserId = CreateDataArray<vtkUnsignedCharArray>("laser_id", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Azimuth = CreateDataArray<vtkUnsignedShortArray>("azimuth", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Distance = CreateDataArray<vtkDoubleArray>("distance_m", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->DistanceRaw =
    CreateDataArray<vtkUnsignedShortArray>("distance_raw", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Timestamp = CreateDataArray<vtkDoubleArray>("adjustedtime", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->RawTime = CreateDataArray<vtkUnsignedIntArray>("timestamp", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->DistanceFlag = CreateDataArray<vtkIntArray>("dual_distance", numberOfPoints, prereservedNumberOfPoints, nullptr);
  this->IntensityFlag = CreateDataArray<vtkIntArray>("dual_intensity", numberOfPoints, prereservedNumberOfPoints, nullptr);
  this->Flags = CreateDataArray<vtkUnsignedIntArray>("dual_flags", numberOfPoints, prereservedNumberOfPoints, nullptr);
  this->DualReturnMatching =
    CreateDataArray<vtkIdTypeArray>("dual_return_matching", numberOfPoints, prereservedNumberOfPoints, nullptr);
  this->VerticalAngle = CreateDataArray<vtkDoubleArray>("vertical_angle", numberOfPoints, prereservedNumberOfPoints, polyData);

  // FieldData : RPM
  vtkSmartPointer<vtkDoubleArray> rpmData = vtkSmartPointer<vtkDoubleArray>::New();
  rpmData->SetNumberOfTuples(1);     // One tuple
  rpmData->SetNumberOfComponents(1); // One value per tuple, the scalar
  rpmData->SetName("RotationPerMinute");
  rpmData->SetTuple1(0, this->Frequency);
  polyData->GetFieldData()->AddArray(rpmData);

  if (this->HasDualReturn)
  {
    polyData->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
    polyData->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
    polyData->GetPointData()->AddArray(this->DualReturnMatching.GetPointer());
  }

  return polyData;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneLegacyPacketInterpreter::SplitFrame(bool force)
{
  if (this->vtkLidarPacketInterpreter::SplitFrame(force))
  {
    for (size_t n = 0; n < HDL_MAX_NUM_LASERS; ++n)
    {
      this->LastPointId[n] = -1;
    }
    // compute th rpm and add it to the splited frame
    this->Frequency = this->RpmCalculator_->GetRPM();
    this->RpmCalculator_->Reset();
    this->Frames.back()->GetFieldData()
      ->GetArray("RotationPerMinute")
      ->SetTuple1(0, this->Frequency);

    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
void vtkVelodyneLegacyPacketInterpreter::ResetCurrentFrame()
{
  std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);
  this->CurrentFrameState->reset();
  this->LastTimestamp = std::numeric_limits<unsigned int>::max();
  this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();

  this->rollingCalibrationData->clear();
  this->HasDualReturn = false;
  this->IsHDL64Data = false;
  this->IsVLS128 = false;
  this->Frames.clear();
  this->CurrentFrame = this->CreateNewEmptyFrame(0);

  this->ShouldCheckSensor = true;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneLegacyPacketInterpreter::PreProcessPacket(unsigned char const* data,
  unsigned int vtkNotUsed(dataLength), fpos_t filePosition, double packetNetworkTime,
  std::vector<FrameInformation>* frameCatalog)
{
  const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket*>(data);
  //! @todo don't use static value here this is ugly...
  static bool isEmptyFrame = true;
  static FramingState currentFrameState;
  static int numberOfFiringPackets = 0;
  static int lastnumberOfFiringPackets = 0;
  static int frameNumber;

  numberOfFiringPackets++;
  bool isNewFrame = false;

  //! @todo this could be useful at a higher level
  if (this->ShouldCheckSensor)
  {
    this->CheckReportedSensorAndCalibrationFileConsistent(dataPacket);
    this->ShouldCheckSensor = false;
  }

  //    unsigned int timeDiff = dataPacket->gpsTimestamp - lastTimestamp;
  //    if (timeDiff > 600 && lastTimestamp != 0)
  //      {
  //      printf("missed %d packets\n",  static_cast<int>(floor((timeDiff/553.0) + 0.5)));
  //      }

  // Check if the time has rolled between this packet and
  // the previous one. There is only one timestamp per packet
  // this is why the check is not performed per firing or per laser
  VelodyneSpecificFrameInformation* velFrameInfo =
      reinterpret_cast<VelodyneSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());
  if (dataPacket->gpsTimestamp < this->lastGpsTimestamp)
  {
    velFrameInfo->NbrOfRollingTime++;
  }
  this->lastGpsTimestamp = dataPacket->gpsTimestamp;

  this->ParserMetaData.FilePosition = filePosition;

  // update the timestamps information
  this->ParserMetaData.FirstPacketDataTime = dataPacket->gpsTimestamp;
  this->ParserMetaData.FirstPacketNetworkTime = packetNetworkTime;

  this->IsHDL64Data |= dataPacket->isHDL64();

  this->IsVLS128 = dataPacket->isVLS128();

  for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
  {
    const HDLFiringData& firingData = dataPacket->firingData[i];

    // Skip dummy blocks of VLS-128 dual mode last 4 blocks
    if (IsVLS128 && (firingData.blockIdentifier == 0 || firingData.blockIdentifier == 0xFFFF))
    {
      continue;
    }

    // Test if at least one laser has a positive distance
    if (this->IgnoreZeroDistances)
    {
      for (int laserID = 0; laserID < HDL_LASER_PER_FIRING; laserID++)
      {
        if (firingData.laserReturns[laserID].distance != 0)
        {
          isEmptyFrame = false;
          break;
        }
      }
    }
    else
    {
      isEmptyFrame = false;
    }

    if (currentFrameState.hasChangedWithValue(firingData))
    {
      // Add file position if the frame is not empty
      if (!isEmptyFrame || !this->IgnoreEmptyFrames)
      {
        // update the firing to skip information
        // and add the current frame information
        // to the catalog
        velFrameInfo->FiringToSkip = i;
        if (frameCatalog)
        {
          frameCatalog->push_back(this->ParserMetaData);
        }
        isNewFrame = true;

        frameNumber++;
        PacketProcessingDebugMacro(
          << "\n\nEnd of frame #" << frameNumber
          << ". #packets: " << numberOfFiringPackets - lastnumberOfFiringPackets << "\n\n"
          << "RotationalPositions: ");
        lastnumberOfFiringPackets = numberOfFiringPackets;
      }
      // We start a new frame, reinitialize the boolean
      isEmptyFrame = true;
    }
    PacketProcessingDebugMacro(<< firingData.rotationalPosition << ", ");
  }

  // Accumulate HDL64 Status byte data
  if (IsHDL64Data && this->IsCorrectionFromLiveStream &&
    !this->IsCalibrated)
  {
    this->rollingCalibrationData->appendData(dataPacket->gpsTimestamp, dataPacket->factoryField1, dataPacket->factoryField2);
    this->HDL64LoadCorrectionsFromStreamData(this->rollingCalibrationData);
  }
  return isNewFrame;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneLegacyPacketInterpreter::CheckReportedSensorAndCalibrationFileConsistent(const HDLDataPacket* dataPacket)
{
  // Get the number of laser from sensor type
  int reportedSensorNumberLaser = num_laser(dataPacket->getSensorType());
  this->IsHDL64Data = dataPacket->isHDL64();
  this->ReportedSensor = dataPacket->getSensorType();
  this->ReportedFactoryField1 = dataPacket->factoryField1;
  this->ReportedFactoryField2 = dataPacket->factoryField2;


  if (this->IsCorrectionFromLiveStream)
  {
    return true;
  }
  // compare the numbers of lasers
  if (reportedSensorNumberLaser != this->CalibrationReportedNumLasers)
  {
    std::stringstream warningMessage;
    if (reportedSensorNumberLaser == 0)
    {
      warningMessage << "The data-packet from the sensor has an unrecognised "
                     << "factory byte (0x" << hex << this->ReportedSensor << dec << ")";
    }
    else
    {
      warningMessage << "The data-packet from the sensor has a factory byte "
                     << "(0x" << hex << this->ReportedSensor << dec << ") "
                     << "recognized as having " << reportedSensorNumberLaser << " lasers";
    }
    warningMessage << ", " << SOFTWARE_NAME << " will interpret data-packets and show points"
                   << " based on the XML calibration file only (currently: "
                   << this->CalibrationReportedNumLasers << " lasers).";
    vtkGenericWarningMacro(<< warningMessage.str());
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
std::string vtkVelodyneLegacyPacketInterpreter::GetSensorInformation()
{
  std::stringstream streamInfo;
  streamInfo << "Factory Field 1: " << (int)this->ReportedFactoryField1 << " (0x"
             << std::hex << (int)this->ReportedFactoryField1 << std::dec << ") "
             << DataPacketFixedLength::DualReturnSensorModeToString(
                  static_cast<DataPacketFixedLength::DualReturnSensorMode>(this->ReportedFactoryField1))
             << "  |  "
             << "FF 2: " << (int)this->ReportedFactoryField2 << " (0x"
             << std::hex << (int)this->ReportedFactoryField2 << std::dec << ") "
             << DataPacketFixedLength::SensorTypeToString(
                  static_cast<SensorType>(this->ReportedFactoryField2));

  return std::string(streamInfo.str());
}

