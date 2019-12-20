#include "vtkVelodyneSpecialVelarrayPacketInterpreter.h"

#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkTransform.h>

#include <bitset>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include "vtkDataPacket.h"
#include "vtkVelodyneMetaPacketInterpreter.h"

using namespace DataPacketFixedLength;
using namespace SpecialVelarrayPacket;

#define PacketProcessingDebugMacro(x)                                                              \
  {                                                                                                \
    if (this->OutputPacketProcessingDebugInfo)                                                     \
    {                                                                                              \
      std::cout << " " x;                                                                          \
    }                                                                                              \
  }

template<typename T, typename I, typename U>
void SetValue(T array, I id, U value)
{
  if (array != nullptr)
  {
    array->SetValue(id, value);
  }
}

template<typename T, typename U>
void InsertNextValue(T array, U value)
{
  if (array != nullptr)
  {
    array->InsertNextValue(value);
  }
}

template<typename T, typename U>
void AddArray(T holder, U array)
{
  if (array != nullptr)
  {
    holder->AddArray(array);
  }
}

//namespace
//{
//! @todo this method are actually usefull for every Interpreter and should go to the top
template<typename T>
vtkSmartPointer<T> vtkVelodyneSpecialVelarrayPacketInterpreter::CreateDataArray(bool isAdvanced, const char* name, vtkIdType np, vtkIdType prereserved_np, vtkPolyData* pd)
{
  if (isAdvanced && !this->EnableAdvancedArrays)
  {
    return nullptr;
  }
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

namespace
{

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
}

//} // End namespace

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneSpecialVelarrayPacketInterpreter)

//-----------------------------------------------------------------------------
vtkVelodyneSpecialVelarrayPacketInterpreter::vtkVelodyneSpecialVelarrayPacketInterpreter()
{
  this->UseIntraFiringAdjustment = true;
  this->OutputPacketProcessingDebugInfo = false;
  this->SensorPowerMode = 0;
  this->CurrentFrameState = new FrameTracker;
  this->LastTimestamp = std::numeric_limits<unsigned int>::max();
  this->LastAzimuth = 0;
  this->LastAzimuthDiff = 0;
  this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
  this->FiringsSkip = 0;
  this->ShouldCheckSensor = true;

  std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);

  this->LaserSelection.resize(HDL_MAX_NUM_LASERS, true);
  this->DualReturnFilter = 0;
  this->DistanceResolutionM = 0.002;
  this->WantIntensityCorrection = false;
  this->ParserMetaData.SpecificInformation = std::make_shared<VelodyneSpecificFrameInformation>();

  this->Init();
}

vtkVelodyneSpecialVelarrayPacketInterpreter::~vtkVelodyneSpecialVelarrayPacketInterpreter()
{
  delete this->CurrentFrameState;
}

//-----------------------------------------------------------------------------
void vtkVelodyneSpecialVelarrayPacketInterpreter::ProcessPacket(unsigned char const* data, unsigned int dataLength)
{
  if (!this->IsLidarPacket(data, dataLength))
  {
    return;
  }

  VelodyneSpecificFrameInformation* velFrameInfo =
      reinterpret_cast<VelodyneSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());
  int firstFiring = velFrameInfo->FiringToSkip;

  const Packet* dataPacket = reinterpret_cast<const Packet*>(data);

  this->ReportedMICField = dataPacket->header.GetMIC();

  if (this->ShouldCheckSensor)
  {
    this->CheckReportedSensorAndCalibrationFileConsistent(dataPacket);
    ShouldCheckSensor = false;
  }

  for (long i = firstFiring; i < Packet::FiringCount(dataLength); i++)
  {
    this->ProcessFiring(&dataPacket->header,
                        &dataPacket->firings[i],
                        dataPacket->GetFooter(dataLength));
  }
  //////////// this->ParserMetaData.FirstPacketDataTime = dataPacket->gpsTimestamp;

  //////////// const unsigned int rawtime = dataPacket->gpsTimestamp;
  //////////// const double timestamp = dataPacket->gpsTimestamp;

  //////////// // Update the transforms here and then call internal
  //////////// // transform
  //////////// if (SensorTransform) this->SensorTransform->Update();

  //////////// VelodyneSpecificFrameInformation* velodyneFrameInfo =
  ////////////     reinterpret_cast<VelodyneSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());
  //////////// int firingBlock = velodyneFrameInfo->FiringToSkip;
  //////////// velodyneFrameInfo->FiringToSkip = 0;

  //////////// // Compute the list of total azimuth advanced during one full firing block
  //////////// std::vector<int> diffs(HDL_FIRING_PER_PKT - 1);
  //////////// for (int i = 0; i < HDL_FIRING_PER_PKT - 1; ++i)
  //////////// {
  ////////////   int localDiff = 0;

  ////////////   localDiff = (36000 + 18000 + dataPacket->firingData[i + 1].getRotationalPosition() -
  ////////////                 dataPacket->firingData[i].getRotationalPosition()) %
  ////////////                 36000 - 18000;
  ////////////   diffs[i] = localDiff;
  //////////// }

  //////////// std::sort(diffs.begin(), diffs.end());
  //////////// // Assume the median of the packet's rotationalPosition differences
  //////////// int azimuthDiff = diffs[HDL_FIRING_PER_PKT / 2];
  //////////// // assert(azimuthDiff > 0);

  //////////// // Add DualReturn-specific arrays if newly detected dual return packet
  //////////// if (dataPacket->isDualModeReturn() && !this->HasDualReturn)
  //////////// {
  ////////////   this->HasDualReturn = true;
  ////////////   AddArray(this->CurrentFrame->GetPointData(), this->DualReturnMatching.GetPointer());
  //////////// }

  //////////// for (; firingBlock < HDL_FIRING_PER_PKT; ++firingBlock)
  //////////// {
  ////////////   const HDLFiringData* firingData = &(dataPacket->firingData[firingBlock]);
  ////////////   const HDLFiringData* extData = NULL;
  ////////////   // clang-format off
  ////////////   int multiBlockLaserIdOffset =
  ////////////       (firingData->blockIdentifier == BLOCK_0_TO_31)  ?  0 :(
  ////////////       (firingData->blockIdentifier == BLOCK_32_TO_63) ? 32 :(
  ////////////       (firingData->blockIdentifier == BLOCK_64_TO_95) ? 64 :(
  ////////////       (firingData->blockIdentifier == BLOCK_96_TO_127)? 96 :(
  ////////////                                                          0))));
  ////////////   // clang-format on

  ////////////   if (this->CurrentFrameState->hasChangedWithValue(*firingData))
  ////////////   {
  ////////////     this->SplitFrame();
  ////////////     this->LastTimestamp = std::numeric_limits<unsigned int>::max();
  ////////////   }

  ////////////   // For variable speed sensors, get this firing block exact azimuthDiff
  ////////////   if (dataPacket->getSensorType() == VelArray)
  ////////////   {
  ////////////     azimuthDiff = dataPacket->getRotationalDiffForVelarrayFiring(firingBlock);
  ////////////     // only use the azimuth-diff between firings if below 2 degrees
  ////////////     if (abs(azimuthDiff) > 200)
  ////////////       azimuthDiff = 0;
  ////////////   }
  ////////////   // Skip this firing every PointSkip
  ////////////   if (this->FiringsSkip == 0 || firingBlock % (this->FiringsSkip + 1) == 0)
  ////////////   {
  ////////////     this->ProcessFiring(firingData, multiBlockLaserIdOffset, firingBlock, azimuthDiff, timestamp,
  ////////////       rawtime, dataPacket->isDualReturnFiringBlock(firingBlock), dataPacket->isDualModeReturn(), extData, dataPacket->getExtDataPacketType());
  ////////////   }
  //////////// }
}

//-----------------------------------------------------------------------------
bool vtkVelodyneSpecialVelarrayPacketInterpreter::IsLidarPacket(unsigned char const * data, unsigned int dataLength)
{
  // current specs are from "Special Velarray Data Packet Format_v7.pdf" dated 12/10/2019
  // Document Number 63-9663 Revision 1, sha1: 7b0773f992df623f3cdea1a25a21c289c337a5a1

  // TODO: improve this (CRC check ?)
  // TODO: check reserved fields (not just MIC) (why are leading zeros specified)
  bool status = true;
  status = status && (dataLength == 1304);
  const Packet* dataPacket = reinterpret_cast<const Packet*>(data);

  status = status && dataPacket->header.GetMIC() == 0x40;
  return status;
}

//-----------------------------------------------------------------------------
void vtkVelodyneSpecialVelarrayPacketInterpreter::ProcessFiring(const PayloadHeader* header,
                                                         const FiringReturn* firingData,
                                                         const PayloadFooter* footer)
{
  // TODO: what does azimuth adjustement becomes ?
  // TODO: what does timestamp adjustement becomes ?
  // TODO check that we just need VDFL, and not calib
  // TODO check that we just need TREF for timestamp
  const vtkIdType thisPointId = this->Points->GetNumberOfPoints();
  RawValues rawValues(firingData->GetAZM(), firingData->GetVDFL(), firingData->GetDIST(), firingData->GetRFT());
  CorrectedValues correctedValues;

  this->ComputeCorrectedValues(
      rawValues,
      firingData->GetLCN(),
      correctedValues,
      false
    );

  double & distanceM = correctedValues.distance;;
  short intensity = correctedValues.intensity;
  double (& pos)[3] = correctedValues.position;

  // Apply sensor transform
  if (SensorTransform) this->SensorTransform->InternalTransformPoint(pos, pos);

  if (this->shouldBeCroppedOut(pos))
    return;

  // Do not add any data before here as this might short-circuit
  bool isFiringDualReturnData = false; // TODO remove
  if (isFiringDualReturnData)
  {
    const vtkIdType dualPointId = this->LastPointId[firingData->GetLCN()];
    if (dualPointId < this->FirstPointIdOfDualReturnPair)
    {
      // No matching point from first set (skipped?)
      InsertNextValue(this->Flags, DUAL_DOUBLED);
      InsertNextValue(this->DualReturnMatching, -1); // std::numeric_limits<vtkIdType>::quiet_NaN()
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
          SetValue(this->Flags, dualPointId, firstFlags);
          return;
        }
        if (!(firstFlags & this->DualReturnFilter))
        {
          // first return does not match filter; replace with second return
          this->Points->SetPoint(dualPointId, pos);
          SetValue(this->Distance, dualPointId, distanceM);
          SetValue(this->DistanceRaw, dualPointId, firingData->GetDIST());
          SetValue(this->Intensity, dualPointId, intensity);
          SetValue(this->Timestamp, dualPointId, header->GetTREF()); // TODO: adjust time ??
          // SetValue(this->RawTime, dualPointId, rawtime);
          SetValue(this->Flags, dualPointId, secondFlags);
          return;
        }
      }

      SetValue(this->Flags, dualPointId, firstFlags);
      InsertNextValue(this->Flags, secondFlags);
      // The first return indicates the dual return
      // and the dual return indicates the first return
      InsertNextValue(this->DualReturnMatching, dualPointId);
      SetValue(this->DualReturnMatching, dualPointId, thisPointId);
    }
  }
  else
  {
    // InsertNextValue(this->Flags, DUAL_DOUBLED); // TODO remove
    // InsertNextValue(this->DualReturnMatching, -1); // std::numeric_limits<vtkIdType>::quiet_NaN()
  }

  this->Points->InsertNextPoint(pos);
  InsertNextValue(this->PointsX, pos[0]);
  InsertNextValue(this->PointsY, pos[1]);
  InsertNextValue(this->PointsZ, pos[2]);
  InsertNextValue(this->Azimuth, firingData->GetAZM());
  InsertNextValue(this->Intensity, intensity);
  InsertNextValue(this->LaserId, firingData->GetLCN());
  InsertNextValue(this->Timestamp, header->GetTREF()); // TODO adjust  time ??
  // InsertNextValue(this->RawTime, rawtime);
  InsertNextValue(this->Distance, distanceM);
  InsertNextValue(this->DistanceRaw, firingData->GetDIST());
  this->LastPointId[firingData->GetLCN()] = thisPointId;
  InsertNextValue(this->VerticalAngle, correctedValues.elevation);
  InsertNextValue(this->HDir, firingData->GetHDIR());
  InsertNextValue(this->VDir, firingData->GetVDIR());
  InsertNextValue(this->LaserCounters, this->LaserCounter);
  this->LaserCounter++;
  InsertNextValue(this->TREF, header->GetTREF());
  InsertNextValue(this->PSEQ, header->GetPSEQ());
  InsertNextValue(this->PSEQF, footer->GetPSEQF());
  InsertNextValue(this->AC, footer->GetAC());

  bool isNewFrame = this->CurrentFrameState->Update(firingData->GetAZM(),
                                                    firingData->GetVDFL(),
                                                    static_cast<HorizontalDirection>(firingData->GetHDIR()),
                                                    static_cast<VerticalDirection>(firingData->GetVDIR()));
  if (isNewFrame)
  {
    this->SplitFrame();
    this->LastTimestamp = std::numeric_limits<unsigned int>::max();
  }

  // // First return block of a dual return packet: init last point of laser
  // if (!isThisFiringDualReturnData)
  // {
  //   this->FirstPointIdOfDualReturnPair = this->Points->GetNumberOfPoints();
  // }

  // unsigned short firingElevation100th = firingData->getElevation100th();

  // for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++)
  // {
  //   const unsigned char rawLaserId = static_cast<unsigned char>(dsr + firingBlockLaserOffset);
  //   unsigned char laserId = rawLaserId;
  //   const unsigned short azimuth = firingData->getRotationalPosition();

  //   // Interpolate azimuths and timestamps per laser within firing blocks
  //   double timestampadjustment = 0;
  //   int azimuthadjustment = 0;
  //   if (this->UseIntraFiringAdjustment)
  //   {
  //     double blockdsr0 = 0, nextblockdsr0 = 1;
  //     switch (this->CalibrationReportedNumLasers)
  //     {
  //       case 32:
  //       {
  //         if (this->ReportedSensor == VelArray)
  //         {
  //           timestampadjustment = VelArrayAdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
  //           nextblockdsr0 = VelArrayAdjustTimeStamp(
  //             firingBlock + (isDualReturnPacket ? 2 : 1), 0, isDualReturnPacket);
  //           blockdsr0 = VelArrayAdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
  //         }
  //         break;
  //       }
  //       default:
  //       {
  //         timestampadjustment = 0.0;
  //         blockdsr0 = 0.0;
  //         nextblockdsr0 = 1.0;
  //       }
  //     }
  //     azimuthadjustment = vtkMath::Round(
  //       azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
  //     timestampadjustment = vtkMath::Round(timestampadjustment);

  //   }


  //   if ((!this->IgnoreZeroDistances || firingData->laserReturns[dsr].distance != 0.0) &&
  //     this->LaserSelection[laserId])
  //   {
  //     const unsigned short adjustedAzimuth =
  //       (36000 + (static_cast<int>(azimuth) + azimuthadjustment)) % 36000;
  //     this->PushFiringData(laserId, rawLaserId, adjustedAzimuth, firingElevation100th,
  //       timestamp + timestampadjustment, rawtime + static_cast<unsigned int>(timestampadjustment),
  //       &(firingData->laserReturns[dsr]), dsr + firingBlockLaserOffset,
  //       isThisFiringDualReturnData, extDataPacketType, &(extData->laserReturns[dsr]));
  //   }
  // }
}

//-----------------------------------------------------------------------------
void vtkVelodyneSpecialVelarrayPacketInterpreter::Init()
{
  this->InitTrigonometricTables();
  this->ResetCurrentFrame();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneSpecialVelarrayPacketInterpreter::CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints)
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
  this->PointsX = CreateDataArray<vtkDoubleArray>(true, "X", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->PointsY = CreateDataArray<vtkDoubleArray>(true, "Y", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->PointsZ = CreateDataArray<vtkDoubleArray>(true, "Z", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Intensity = CreateDataArray<vtkUnsignedCharArray>(false, "intensity", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->LaserId = CreateDataArray<vtkUnsignedCharArray>(false, "laser_id", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Azimuth = CreateDataArray<vtkUnsignedShortArray>(false, "azimuth", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Distance = CreateDataArray<vtkDoubleArray>(false, "distance_m", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->DistanceRaw =
    CreateDataArray<vtkUnsignedShortArray>(true, "distance_raw", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->Timestamp = CreateDataArray<vtkDoubleArray>(false, "adjustedtime", numberOfPoints, prereservedNumberOfPoints, polyData);
  // this->RawTime = CreateDataArray<vtkUnsignedIntArray>(false, "timestamp", numberOfPoints, prereservedNumberOfPoints, polyData); // TODO remove
  // this->Flags = CreateDataArray<vtkUnsignedIntArray>(false, "dual_flags", numberOfPoints, prereservedNumberOfPoints, nullptr);
  // this->DualReturnMatching =
  //   CreateDataArray<vtkIdTypeArray>(true, "dual_return_matching", numberOfPoints, prereservedNumberOfPoints, nullptr);
  this->VerticalAngle = CreateDataArray<vtkDoubleArray>(false, "vertical_angle", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->HDir = CreateDataArray<vtkUnsignedCharArray>(true, "HDir", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->VDir = CreateDataArray<vtkUnsignedCharArray>(true, "VDir", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->LaserCounters = CreateDataArray<vtkUnsignedIntArray>(true, "LaserCounter", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->TREF = CreateDataArray<vtkUnsignedLongArray>(true, "TREF", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->PSEQ = CreateDataArray<vtkUnsignedIntArray>(true, "PSEQ", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->PSEQF = CreateDataArray<vtkUnsignedCharArray>(true, "PSEQF", numberOfPoints, prereservedNumberOfPoints, polyData);
  this->AC = CreateDataArray<vtkUnsignedCharArray>(true, "AC", numberOfPoints, prereservedNumberOfPoints, polyData);

  // if (this->HasDualReturn) // TODO remove
  // {
  //   AddArray(polyData->GetPointData(), this->DualReturnMatching.GetPointer());
  // }

  return polyData;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneSpecialVelarrayPacketInterpreter::SplitFrame(bool force)
{
  if (this->vtkLidarPacketInterpreter::SplitFrame(force))
  {
    for (size_t n = 0; n < HDL_MAX_NUM_LASERS; ++n)
    {
      this->LastPointId[n] = -1;
    }

    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
void vtkVelodyneSpecialVelarrayPacketInterpreter::ResetCurrentFrame()
{
  std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);
  this->CurrentFrameState->Reset();
  this->LastTimestamp = std::numeric_limits<unsigned int>::max();
  this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();

  // this->HasDualReturn = false;
  this->Frames.clear();
  this->CurrentFrame = this->CreateNewEmptyFrame(0);

  this->ShouldCheckSensor = true;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneSpecialVelarrayPacketInterpreter::PreProcessPacket(unsigned char const* data,
  unsigned int dataLength, fpos_t filePosition, double packetNetworkTime,
  std::vector<FrameInformation>* frameCatalog)
{
  const Packet* dataPacket = reinterpret_cast<const Packet*>(data);
  //! @todo don't use static value here this is ugly...
  static FrameTracker currentFrameState;

  bool isNewFrame = false;

  //! @todo this could be useful at a higher level
  if (this->ShouldCheckSensor)
  {
    this->CheckReportedSensorAndCalibrationFileConsistent(dataPacket);
    this->ShouldCheckSensor = false;
  }

  VelodyneSpecificFrameInformation* velFrameInfo =
      reinterpret_cast<VelodyneSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());

  this->ParserMetaData.FilePosition = filePosition;

  // update the timestamps information
  this->ParserMetaData.FirstPacketDataTime = 0.0; // TODO
  this->ParserMetaData.FirstPacketNetworkTime = packetNetworkTime;

  for (long i = 0; i < Packet::FiringCount(dataLength); i++)
  {
    const FiringReturn* firingData = &dataPacket->firings[i];
    bool firingGivesNewFrame = this->CurrentFrameState->Update(firingData->GetAZM(),
                                                               firingData->GetVDFL(),
                                                               static_cast<HorizontalDirection>(firingData->GetHDIR()),
                                                               static_cast<VerticalDirection>(firingData->GetVDIR()));
    // TODO: similarily to other interpreters, check that the frame is not empty
    if (firingGivesNewFrame)
    {
      isNewFrame = true;
      velFrameInfo->FiringToSkip = i;
      if (frameCatalog)
      {
        frameCatalog->push_back(this->ParserMetaData);
      }
    }
  }

  return isNewFrame;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneSpecialVelarrayPacketInterpreter::CheckReportedSensorAndCalibrationFileConsistent(const Packet* vtkNotUsed(dataPacket))
{
  int reportedSensorNumberLaser = 32;
  // compare the numbers of lasers
  if (reportedSensorNumberLaser != this->CalibrationReportedNumLasers)
  {
    std::stringstream warningMessage;
    warningMessage << "This interpreter expect "
                   << reportedSensorNumberLaser
                   << " lasers. But the calibration file indicates "
                   << this->CalibrationReportedNumLasers
                   << " lasers.";
    warningMessage << ", " << SOFTWARE_NAME << " will interpret data-packets and show points"
                   << " based on the XML calibration file only (currently: "
                   << this->CalibrationReportedNumLasers << " lasers).";
    vtkGenericWarningMacro(<< warningMessage.str());
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
std::string vtkVelodyneSpecialVelarrayPacketInterpreter::GetSensorInformation()
{
  std::stringstream streamInfo;
  streamInfo << "MIC Field : " << (int)this->ReportedMICField << " (0x"
             << std::hex << (int)this->ReportedMICField << std::dec << ") ";

  return std::string(streamInfo.str());
}

