// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVelodyneHDLReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkVelodyneHDLReader.h"

#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"
#include "vtkVelodyneTransformInterpolator.h"
#include "vtkRollingDataAccumulator.h"

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedShortArray.h>

#include <vtkTransform.h>

#include <sstream>
#include <algorithm>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/preprocessor.hpp>

#include <Eigen/Dense>

#ifdef _MSC_VER
# include <boost/cstdint.hpp>
typedef boost::uint8_t uint8_t;
#else
# include <stdint.h>
#endif

#include "vtkDataPacket.h"
using namespace DataPacketFixedLength;

namespace
{
enum CropModeEnum
{
  None = 0,
  Cartesian = 1,
  Spherical = 2,
  Cylindric = 3,
};


//-----------------------------------------------------------------------------
int MapFlags(unsigned int flags, unsigned int low, unsigned int high)
{
  return (flags & low ? -1 : flags & high ? 1 : 0);
}

//-----------------------------------------------------------------------------
int MapDistanceFlag(unsigned int flags)
{
  return MapFlags(flags & vtkVelodyneHDLReader::DUAL_DISTANCE_MASK,
                  vtkVelodyneHDLReader::DUAL_DISTANCE_NEAR,
                  vtkVelodyneHDLReader::DUAL_DISTANCE_FAR);
}

//-----------------------------------------------------------------------------
int MapIntensityFlag(unsigned int flags)
{
  return MapFlags(flags & vtkVelodyneHDLReader::DUAL_INTENSITY_MASK,
                  vtkVelodyneHDLReader::DUAL_INTENSITY_LOW,
                  vtkVelodyneHDLReader::DUAL_INTENSITY_HIGH);
}

//-----------------------------------------------------------------------------
  double HDL32AdjustTimeStamp(int firingblock,
                              int dsr,
                              const bool isDualReturnMode)
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
double VLP16AdjustTimeStamp(int firingblock,
                            int dsr,
                            int firingwithinblock,
                            const bool isDualReturnMode)
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
double VLP32AdjustTimeStamp(int firingblock,
                            int dsr,
                            const bool isDualReturnMode)
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
double HDL64EAdjustTimeStamp(int firingblock,
                            int dsr,
                            const bool isDualReturnMode)
{
  const int dsrReversed = HDL_LASER_PER_FIRING - dsr - 1;
  const int firingblockReversed = HDL_FIRING_PER_PKT - firingblock - 1;
  if (!isDualReturnMode)
    {
      const double TimeOffsetMicroSec[4] = {2.34, 3.54, 4.74, 6.0};
      return (std::floor(static_cast<double>(firingblockReversed)/ 2.0) * 48.0)
          + TimeOffsetMicroSec[(dsrReversed % 4)]
          + (dsrReversed / 4) * TimeOffsetMicroSec[3];
    }
  else
    {
      const double TimeOffsetMicroSec[4] = {3.5, 4.7, 5.9, 7.2};
      return (std::floor(static_cast<double>(firingblockReversed)/ 4.0) * 57.6)
          + TimeOffsetMicroSec[(dsrReversed % 4)]
          + (dsrReversed / 4) * TimeOffsetMicroSec[3];
    }
}
}

//-----------------------------------------------------------------------------
void GetSphericalCoordinates(double p_in[3], double p_out[3])
{
  // We will compute R, theta and phi
  double R, theta, phi, rho;
  Eigen::Vector3f P(p_in[0],p_in[1],p_in[2]);
  Eigen::Vector3f projP(p_in[0],p_in[1],0); // Projection on the (X,Y) plane
  Eigen::Vector3f ez(0,0,1);
  Eigen::Vector3f ex(1,0,0);
  Eigen::Vector3f u_r = P.normalized(); // u_r vector in spherical coordinates
  Eigen::Vector3f u_theta = projP.normalized(); // u_theta in polar/cylindric coordinates
  R = P.norm();
  rho = projP.norm();

  // Phi is the angle between OP and ez :
  double cosPhi = u_r.dot(ez);
  double sinPhi = u_r.dot(u_theta);

  phi = std::abs(std::atan2(sinPhi,cosPhi)) / vtkMath::Pi() * 180;

  // Theta is the angle between u_theta and ex :
  // Since u_theta is normalized cos(theta) = u_theta[0]
  // and sin(theta) = u_theta[1]
  theta = (std::atan2(u_theta[1],u_theta[0]) + vtkMath::Pi()) / vtkMath::Pi() * 180;

  p_out[0] = theta;
  p_out[1] = phi;
  p_out[2] = R;
}

//-----------------------------------------------------------------------------
class vtkVelodyneHDLReader::vtkInternal
{
public:

  vtkInternal()
  {
    this->AlreadyWarnAboutCalibration = false;
    this->CropMode = Cartesian;
    this->HasDualReturn = false;
    this->ShouldAddDualReturnArray = false;
    this->alreadyWarnedForIgnoredHDL64FiringPacket = false;
    this->SensorPowerMode = 0;
    this->Skip = 0;
    this->LastAzimuth = -1;
    this->LastTimestamp = std::numeric_limits<unsigned int>::max();
    this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
    this->Reader = 0;
    this->SplitCounter = 0;
    this->NumberOfTrailingFrames = 0;
    this->ApplyTransform = 0;
    this->PointsSkip = 0;
    this->CropReturns = false;
    this->CropInside = false;
    this->CropRegion[0] = this->CropRegion[1] = 0.0;
    this->CropRegion[2] = this->CropRegion[3] = 0.0;
    this->CropRegion[4] = this->CropRegion[5] = 0.0;
    this->CorrectionsInitialized = false;
    this->currentRpm = 0;
    this->firstTimestamp = 0;
    this->firstAngle = std::numeric_limits<int>::max();

    std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);

    this->LaserSelection.resize(HDL_MAX_NUM_LASERS, true);
    this->DualReturnFilter = 0;
    this->IsDualReturnSensorMode = false;
    this->IsHDL64Data = false;
    this->CalibrationReportedNumLasers = -1;
    this->skipFirstFrame = true;
    this->distanceResolutionM = 0.002;
    this->WantIntensityCorrection = false;

    this->rollingCalibrationData = new vtkRollingDataAccumulator();
    this->Init();
  }

  ~vtkInternal()
  {
    if (this->rollingCalibrationData)
      {
        delete this->rollingCalibrationData;
      }
  }

  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;
  vtkSmartPointer<vtkPolyData> CurrentDataset;

  vtkNew<vtkTransform> SensorTransform;
  vtkSmartPointer<vtkVelodyneTransformInterpolator> Interp;

  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkDoubleArray> PointsX;
  vtkSmartPointer<vtkDoubleArray> PointsY;
  vtkSmartPointer<vtkDoubleArray> PointsZ;
  vtkSmartPointer<vtkUnsignedCharArray> Intensity;
  vtkSmartPointer<vtkUnsignedCharArray> LaserId;
  vtkSmartPointer<vtkUnsignedShortArray> Azimuth;
  vtkSmartPointer<vtkDoubleArray> Distance;
  vtkSmartPointer<vtkDoubleArray> Timestamp;
  vtkSmartPointer<vtkDoubleArray> VerticalAngle;
  vtkSmartPointer<vtkUnsignedIntArray> RawTime;
  vtkSmartPointer<vtkIntArray> IntensityFlag;
  vtkSmartPointer<vtkIntArray> DistanceFlag;
  vtkSmartPointer<vtkUnsignedIntArray> Flags;
  vtkSmartPointer<vtkIdTypeArray> DualReturnMatching;
  vtkSmartPointer<vtkDoubleArray> SelectedDualReturn;
  bool ShouldAddDualReturnArray;

  bool IsDualReturnSensorMode;
  SensorType ReportedSensor;
  DualReturnSensorMode ReportedSensorReturnMode;
  bool IsHDL64Data;
  bool skipFirstFrame;
  bool alreadyWarnedForIgnoredHDL64FiringPacket;

  //Bolean to manage the correction of intensity which indicates if the user want to correct the intensities
  bool WantIntensityCorrection;

  int LastAzimuth;
  unsigned int LastTimestamp;
  double currentRpm;
  std::vector<double> RpmByFrames;
  double TimeAdjust;
  double firstTimestamp;
  int firstAngle;
  vtkIdType LastPointId[HDL_MAX_NUM_LASERS];
  vtkIdType FirstPointIdOfDualReturnPair;

  std::vector<fpos_t> FilePositions;
  std::vector<int> Skips;
  int Skip;
  vtkPacketFileReader* Reader;

  unsigned char SensorPowerMode;
  CropModeEnum CropMode;

  // Number of allowed split, for frame-range retrieval.
  int SplitCounter;

  // Parameters ready by calibration
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
  double XMLColorTable[HDL_MAX_NUM_LASERS][3];
  int CalibrationReportedNumLasers;
  bool CorrectionsInitialized;
  bool IsCorrectionFromLiveStream;

  // Sensor parameters presented as rolling data, extracted from enough packets
  vtkRollingDataAccumulator * rollingCalibrationData;

  // User configurable parameters
  int NumberOfTrailingFrames;
  int ApplyTransform;
  int PointsSkip;

  bool CropReturns;
  bool CropInside;
  bool AlreadyWarnAboutCalibration;
  double CropRegion[6];
  double distanceResolutionM;

  std::vector<bool> LaserSelection;
  unsigned int DualReturnFilter;

  void SplitFrame(bool force=false);
  vtkSmartPointer<vtkPolyData> CreateData(vtkIdType numberOfPoints);
  vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);

  void Init();
  void InitTrigonometricTables();
  void PrecomputeCorrectionCosSin();
  void LoadCorrectionsFile(const std::string& filename);
  bool HDL64LoadCorrectionsFromStreamData();
  bool HasDualReturn;
  bool shouldBeCroppedOut(double pos[3],double theta);

  void ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived);

  double ComputeTimestamp(unsigned int tohTime);
  void ComputeOrientation(double timestamp, vtkTransform* geotransform);

  bool isCurrentFrameValid();

  // Process the laser return from the firing data
  // firingData - one of HDL_FIRING_PER_PKT from the packet
  // hdl64offset - either 0 or 32 to support 64-laser systems
  // firingBlock - block of packet for firing [0-11]
  // azimuthDiff - average azimuth change between firings
  // timestamp - the timestamp of the packet
  // geotransform - georeferencing transform
  void ProcessFiring(HDLFiringData* firingData,
                     int hdl65offset,
                     int firingBlock,
                     int azimuthDiff,
                     double timestamp,
                     unsigned int rawtime,
                     vtkTransform* geotransform);

  void PushFiringData(const unsigned char laserId,
                      const unsigned char rawLaserId,
                      unsigned short azimuth,
                      const double timestamp,
                      const unsigned int rawtime,
                      const HDLLaserReturn* laserReturn,
                      const HDLLaserCorrection* correction,
                      vtkTransform* geotransform,
                      const bool hasDualReturn);
  void ComputeCorrectedValues(const unsigned short azimuth,
                              const HDLLaserReturn* laserReturn,
                              const HDLLaserCorrection* correction,
                              double pos[3],
                              double & distanceM ,
                              short & intensity, bool correctIntensity);
};

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLReader);

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::vtkVelodyneHDLReader()
{
  this->Internal = new vtkInternal;
  this->UnloadData();
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::~vtkVelodyneHDLReader()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLReader::GetFileName()
{
  return this->FileName;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetApplyTransform(int apply)
{
  if(apply != this->Internal->ApplyTransform)
    {
    this->Modified();
    }
  this->Internal->ApplyTransform = apply;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetApplyTransform()
{
  return this->Internal->ApplyTransform;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetSensorTransform(vtkTransform* transform)
{
  if (transform)
    {
    this->Internal->SensorTransform->SetMatrix(transform->GetMatrix());
    }
  else
    {
    this->Internal->SensorTransform->Identity();
    }
  this->Modified();
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator* vtkVelodyneHDLReader::GetInterpolator() const
{
  return this->Internal->Interp;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetInterpolator(
  vtkVelodyneTransformInterpolator* interpolator)
{
  this->Internal->Interp = interpolator;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetFileName(const std::string& filename)
{
  if (filename == this->FileName)
    {
    return;
    }

  this->FileName = filename;
  this->Internal->FilePositions.clear();
  this->Internal->Skips.clear();
  this->UnloadData();
  this->Modified();
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLReader::GetCorrectionsFile()
{
  return this->CorrectionsFile;
}

//-----------------------------------------------------------------------------

#define PARAM(z,n, data) int x##n,
#define VAL(z,n, data) x##n,
#define B_HDL_MAX_NUM_LASERS 64
void vtkVelodyneHDLReader::SetLaserSelection(BOOST_PP_REPEAT(BOOST_PP_DEC(B_HDL_MAX_NUM_LASERS), PARAM, "") int BOOST_PP_CAT(x, B_HDL_MAX_NUM_LASERS))
{
  assert(HDL_MAX_NUM_LASERS == B_HDL_MAX_NUM_LASERS);
  int mask[HDL_MAX_NUM_LASERS] = {BOOST_PP_REPEAT(BOOST_PP_DEC(B_HDL_MAX_NUM_LASERS), VAL, "") BOOST_PP_CAT(x, B_HDL_MAX_NUM_LASERS)};
  this->SetLaserSelection(mask);
}
#undef B_HDL_MAX_NUM_LASERS
#undef PARAM
#undef VAL

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetLaserSelection(int LaserSelection[HDL_MAX_NUM_LASERS])
{
  for(int i = 0; i < HDL_MAX_NUM_LASERS; ++i)
    {
    this->Internal->LaserSelection[i] = LaserSelection[i];
    }
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::GetLaserSelection(int LaserSelection[HDL_MAX_NUM_LASERS])
{
  for(int i = 0; i < HDL_MAX_NUM_LASERS; ++i)
    {
    LaserSelection[i] = this->Internal->LaserSelection[i];
    }
}

//-----------------------------------------------------------------------------
double vtkVelodyneHDLReader::GetCurrentRpm()
{
  return this->Internal->currentRpm;
}

//-----------------------------------------------------------------------------
unsigned int vtkVelodyneHDLReader::GetDualReturnFilter() const
{
  return this->Internal->DualReturnFilter;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetDualReturnFilter(unsigned int filter)
{
  if (this->Internal->DualReturnFilter != filter)
    {
    this->Internal->DualReturnFilter = filter;
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::GetLaserCorrections(
    double verticalCorrection[HDL_MAX_NUM_LASERS],
    double rotationalCorrection[HDL_MAX_NUM_LASERS],
    double distanceCorrection[HDL_MAX_NUM_LASERS],
    double distanceCorrectionX[HDL_MAX_NUM_LASERS],
    double distanceCorrectionY[HDL_MAX_NUM_LASERS],
    double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
    double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS],
    double focalDistance[HDL_MAX_NUM_LASERS],
    double focalSlope[HDL_MAX_NUM_LASERS],
    double minIntensity[HDL_MAX_NUM_LASERS],
    double maxIntensity[HDL_MAX_NUM_LASERS]  )
{
  for(int i = 0; i < HDL_MAX_NUM_LASERS; ++i)
    {
    verticalCorrection[i] = this->Internal->laser_corrections_[i].verticalCorrection;
    rotationalCorrection[i] = this->Internal->laser_corrections_[i].rotationalCorrection;
    distanceCorrection[i] = this->Internal->laser_corrections_[i].distanceCorrection;
    distanceCorrectionX[i] = this->Internal->laser_corrections_[i].distanceCorrectionX;
    distanceCorrectionY[i] = this->Internal->laser_corrections_[i].distanceCorrectionY;
    verticalOffsetCorrection[i] = this->Internal->laser_corrections_[i].verticalOffsetCorrection;
    horizontalOffsetCorrection[i] = this->Internal->laser_corrections_[i].horizontalOffsetCorrection;
    focalDistance[i] = this->Internal->laser_corrections_[i].focalDistance;
    focalSlope[i] = this->Internal->laser_corrections_[i].focalSlope;
    minIntensity[i] = this->Internal->laser_corrections_[i].minIntensity;
    maxIntensity[i] = this->Internal->laser_corrections_[i].maxIntensity;
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::GetXMLColorTable(double XMLColorTable[4*HDL_MAX_NUM_LASERS])
{
  for(int i = 0; i < HDL_MAX_NUM_LASERS; ++i)
    {
    XMLColorTable[i*4] = static_cast<double>(i)/63.0 * 255.0;
    for (int j = 0; j < 3; ++j)
      {
      XMLColorTable[i*4+j+1] = this->Internal->XMLColorTable[i][j];
      }
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetDummyProperty(int vtkNotUsed(dummy))
{
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetPointsSkip(int pr)
{
  this->Internal->PointsSkip = pr;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetNumberOfTrailingFrames(int numTrailing)
{
  assert(numTrailing >= 0);
  this->Internal->NumberOfTrailingFrames = numTrailing;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropReturns(int crop)
{
  if (!this->Internal->CropReturns == !!crop)
    {
    this->Internal->CropReturns = !!crop;
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropInside(int crop)
{
  if (!this->Internal->CropInside == !!crop)
    {
    this->Internal->CropInside = !!crop;
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropRegion(double region[6])
{
  std::copy(region, region + 6, this->Internal->CropRegion);
  this->Modified();
}

void vtkVelodyneHDLReader::SetCropMode(int cropMode)
{
  this->Internal->CropMode = static_cast<CropModeEnum>(cropMode);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropRegion(
  double xl, double xu, double yl, double yu, double zl, double zu)
{
  this->Internal->CropRegion[0] = xl;
  this->Internal->CropRegion[1] = xu;
  this->Internal->CropRegion[2] = yl;
  this->Internal->CropRegion[3] = yu;
  this->Internal->CropRegion[4] = zl;
  this->Internal->CropRegion[5] = zu;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCorrectionsFile(const std::string& correctionsFile)
{
  // Live calibration choice passes an empty string as correctionsFile
  if (correctionsFile != "")
    {
    if (correctionsFile == this->CorrectionsFile)
      {
      return;
      }
    if (!boost::filesystem::exists(correctionsFile) ||
        boost::filesystem::is_directory(correctionsFile))
      {
      vtkErrorMacro("Invalid sensor configuration file" << correctionsFile);
      return;
      }
    this->Internal->LoadCorrectionsFile(correctionsFile);
    this->Internal->IsCorrectionFromLiveStream = false;

    }
  else
    {
    this->Internal->CorrectionsInitialized = false;
    this->Internal->IsCorrectionFromLiveStream = true;
    }

  this->CorrectionsFile = correctionsFile;
  this->UnloadData();
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::UnloadData()
{
  std::fill(this->Internal->LastPointId, this->Internal->LastPointId + HDL_MAX_NUM_LASERS, -1);
  this->Internal->LastAzimuth = -1;
  this->Internal->LastTimestamp = std::numeric_limits<unsigned int>::max();
  this->Internal->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
  this->Internal->firstAngle = std::numeric_limits<int>::max();

  this->Internal->rollingCalibrationData->clear();
  this->Internal->IsDualReturnSensorMode = false;
  this->Internal->IsHDL64Data = false;
  this->Internal->Datasets.clear();
  this->Internal->CurrentDataset = this->Internal->CreateData(0);
  this->Internal->HasDualReturn = false;

  this->ShouldCheckSensor = true;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetTimestepInformation(vtkInformation *info)
{
  const size_t numberOfTimesteps = this->Internal->FilePositions.size();
  std::vector<double> timesteps;
  for (size_t i = 0; i < numberOfTimesteps; ++i)
    {
    timesteps.push_back(i);
    }

  if (numberOfTimesteps)
    {
    double timeRange[2] = {timesteps.front(), timesteps.back()};
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
    }
  else
    {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
    }
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::RequestData(vtkInformation *request,
                              vtkInformationVector **inputVector,
                              vtkInformationVector *outputVector)
{
  vtkPolyData *output = vtkPolyData::GetData(outputVector);
  vtkInformation *info = outputVector->GetInformationObject(0);

  if (!this->FileName.length())
    {
    vtkErrorMacro("FileName has not been set.");
    return 0;
    }

  if (!this->Internal->CorrectionsInitialized)
    {
    vtkErrorMacro("Corrections have not been set");
    return 0;
    }

  int timestep = 0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
    {
    double timeRequest = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
    timestep = static_cast<int>(floor(timeRequest+0.5));
    }

  if (timestep < 0 || timestep >= this->GetNumberOfFrames())
    {
    vtkErrorMacro("Cannot meet timestep request: " << timestep << ".  Have " << this->GetNumberOfFrames() << " datasets.");
    output->ShallowCopy(this->Internal->CreateData(0));
    return 0;
    }

  //check if the reported sensor is consistent with the calibration sensor
  if(this->ShouldCheckSensor)
    {
    const unsigned char* data;
    unsigned int dataLength;
    double timeSinceStart;
    //Open the .pcap
    this->Open();
    //set the position to the first full frame
    this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[1]);
    this->Internal->Skip = this->Internal->Skips[1];
    //Read the data of a packet
    this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart);
    //Update the sensor type
    this->updateReportedSensor(data);
    //Compare the number of lasers from calibration and from sensor
    this->isReportedSensorAndCalibrationFileConsistent(true);
    //close the .pcap file
    this->Close();
    //check is done
    this->ShouldCheckSensor = false;
    }

  this->Open();

  if(this->Internal->NumberOfTrailingFrames > 0)
    {
    output->ShallowCopy(this->GetFrameRange(timestep - this->Internal->NumberOfTrailingFrames,
                                            this->Internal->NumberOfTrailingFrames));
    }
  else
    {
    output->ShallowCopy(this->GetFrame(timestep));
    if(this->Internal->ShouldAddDualReturnArray)
      {
      output->GetPointData()->AddArray(this->Internal->SelectedDualReturn);
      }
    }

  this->Close();
  return 1;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  if (this->FileName.length()
      && (!this->Internal->FilePositions.size()
      || this->Internal->IsCorrectionFromLiveStream))
    {
    this->ReadFrameInformation();
    }

  vtkInformation *info = outputVector->GetInformationObject(0);
  this->SetTimestepInformation(info);
  return 1;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FileName: " << this->FileName << endl;
  os << indent << "CorrectionsFile: " << this->CorrectionsFile << endl;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::CanReadFile(const char *fname)
{
  return 1;
}

//-----------------------------------------------------------------------------
 void vtkVelodyneHDLReader::SetShouldAddDualReturnArray(bool input)
 {
   this->Internal->ShouldAddDualReturnArray = input;
 }

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::ProcessHDLPacket(unsigned char *data, unsigned int bytesReceived)
{
  this->Internal->ProcessHDLPacket(data, bytesReceived);
}

//-----------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkPolyData> >& vtkVelodyneHDLReader::GetDatasets()
{
  return this->Internal->Datasets;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetNumberOfFrames()
{
  return this->Internal->FilePositions.size();;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetNumberOfChannels()
{
  return this->Internal->CalibrationReportedNumLasers;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::Open()
{
  this->Close();
  this->Internal->Reader = new vtkPacketFileReader;
  if (!this->Internal->Reader->Open(this->FileName))
    {
    vtkErrorMacro("Failed to open packet file: " << this->FileName << endl << this->Internal->Reader->GetLastError());
    this->Close();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::Close()
{
  delete this->Internal->Reader;
  this->Internal->Reader = 0;
}


//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::DumpFrames(int startFrame, int endFrame, const std::string& filename)
{
  if (!this->Internal->Reader)
    {
    vtkErrorMacro("DumpFrames() called but packet file reader is not open.");
    return;
    }

  vtkPacketFileWriter writer;
  if (!writer.Open(filename))
    {
    vtkErrorMacro("Failed to open packet file for writing: " << filename);
    return;
    }

  pcap_pkthdr* header = 0;
  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  unsigned int lastAzimuth = 0;
  int currentFrame = startFrame;

  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrame]);
  int skip = this->Internal->Skips[startFrame];
  const unsigned int ethernetUDPHeaderLength = 42;
  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart, &header) &&
         currentFrame <= endFrame)
    {
    if (dataLength == (HDLDataPacket::getDataByteLength() + ethernetUDPHeaderLength) ||
        dataLength == (512 + ethernetUDPHeaderLength))
      {
      writer.WritePacket(header, const_cast<unsigned char*>(data));
      }

    // dont check for frame counts if it was not a firing packet
    if(dataLength != HDLDataPacket::getPacketByteLength())
      {
      continue;
      }

    // Check if we cycled a frame and decrement
    const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data + ethernetUDPHeaderLength);

    for (int i = skip; i < HDL_FIRING_PER_PKT; ++i)
      {
      HDLFiringData firingData = dataPacket->firingData[i];

      if (firingData.rotationalPosition != 0 && firingData.rotationalPosition < lastAzimuth)
        {
        currentFrame++;
        if(currentFrame > endFrame)
          {
          break;
          }
        }
      lastAzimuth = firingData.rotationalPosition;
      }
    skip = 0;
    }

  writer.Close();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::GetFrameRange(int startFrame, int wantedNumberOfFrames)
{
  this->UnloadData();
  if (!this->Internal->Reader)
    {
    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
    }
  if (!this->Internal->CorrectionsInitialized)
    {
    vtkErrorMacro("Corrections have not been set");
    return 0;
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  if(startFrame < 0)
    {
    wantedNumberOfFrames -= startFrame;
    startFrame = 0;
    }
  assert(wantedNumberOfFrames > 0);

  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrame]);
  this->Internal->Skip = this->Internal->Skips[startFrame];

  this->Internal->SplitCounter = wantedNumberOfFrames;

  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
    {
    this->ProcessHDLPacket(const_cast<unsigned char*>(data), dataLength);

    if (this->Internal->Datasets.size())
      {
      this->Internal->SplitCounter = 0;
      return this->Internal->Datasets.back();
      }
    }

  this->Internal->SplitFrame(true);
  this->Internal->SplitCounter = 0;
  return this->Internal->Datasets.back();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::GetFrame(int frameNumber)
{
  this->UnloadData();
  if (!this->Internal->Reader)
    {
    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
    }
  if (!this->Internal->CorrectionsInitialized)
    {
    vtkErrorMacro("Corrections have not been set");
    return 0;
    }

  assert(this->Internal->FilePositions.size() == this->Internal->Skips.size());
  if(frameNumber < 0 || frameNumber > this->Internal->FilePositions.size())
    {
    vtkErrorMacro("Invalid frame requested");
    return 0;
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;


  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[frameNumber]);
  this->Internal->Skip = this->Internal->Skips[frameNumber];

  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
    {
    this->ProcessHDLPacket(const_cast<unsigned char*>(data), dataLength);

    if (this->Internal->Datasets.size())
      {
      return this->Internal->Datasets.back();
      }
    }

  this->Internal->SplitFrame(true);
  return this->Internal->Datasets.back();
}

namespace
{
  template <typename T>
  vtkSmartPointer<T> CreateDataArray(const char* name, vtkIdType np, vtkPolyData* pd)
  {
    vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
    array->Allocate(60000);
    array->SetName(name);
    array->SetNumberOfTuples(np);

    if (pd)
      {
      pd->GetPointData()->AddArray(array);
      }

    return array;
  }
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::vtkInternal::CreateData(vtkIdType numberOfPoints)
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // points
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->Allocate(60000);
  points->SetNumberOfPoints(numberOfPoints);
  points->GetData()->SetName("Points_m_XYZ");
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  // intensity
  this->Points = points.GetPointer();
  this->PointsX = CreateDataArray<vtkDoubleArray>("X", numberOfPoints, polyData);
  this->PointsY = CreateDataArray<vtkDoubleArray>("Y", numberOfPoints, polyData);
  this->PointsZ = CreateDataArray<vtkDoubleArray>("Z", numberOfPoints, polyData);
  this->Intensity = CreateDataArray<vtkUnsignedCharArray>("intensity", numberOfPoints, polyData);
  this->LaserId = CreateDataArray<vtkUnsignedCharArray>("laser_id", numberOfPoints, polyData);
  this->Azimuth = CreateDataArray<vtkUnsignedShortArray>("azimuth", numberOfPoints, polyData);
  this->Distance = CreateDataArray<vtkDoubleArray>("distance_m", numberOfPoints, polyData);
  this->Timestamp = CreateDataArray<vtkDoubleArray>("adjustedtime", numberOfPoints, polyData);
  this->RawTime = CreateDataArray<vtkUnsignedIntArray>("timestamp", numberOfPoints, polyData);
  this->DistanceFlag = CreateDataArray<vtkIntArray>("dual_distance", numberOfPoints, 0);
  this->IntensityFlag = CreateDataArray<vtkIntArray>("dual_intensity", numberOfPoints, 0);
  this->Flags = CreateDataArray<vtkUnsignedIntArray>("dual_flags", numberOfPoints, 0);
  this->DualReturnMatching = CreateDataArray<vtkIdTypeArray>("dual_return_matching", numberOfPoints, 0);
  this->VerticalAngle = CreateDataArray<vtkDoubleArray>("vertical_angle", numberOfPoints, polyData);

  //FieldData : RPM
  vtkSmartPointer<vtkDoubleArray> rpmData = vtkSmartPointer<vtkDoubleArray>::New();
  rpmData->SetNumberOfTuples(1); //One tuple
  rpmData->SetNumberOfComponents(1); //One value per tuple, the scalar
  rpmData->SetName("RotationPerMinute"); 
  rpmData->SetTuple1(0,this->currentRpm);
  polyData->GetFieldData()->AddArray(rpmData);
  

  if (this->IsDualReturnSensorMode)
    {
    polyData->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
    polyData->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
    polyData->GetPointData()->AddArray(this->DualReturnMatching.GetPointer());
    }

  return polyData;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> vtkVelodyneHDLReader::vtkInternal::NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

bool vtkVelodyneHDLReader::vtkInternal::shouldBeCroppedOut(double pos[3],double theta){
    // Test if point is cropped
    if (!this->CropReturns)
    {
      return false;
    }
    switch(this->CropMode)
    {
      case Cartesian:// Cartesian cropping mode
      {
        bool pointOutsideOfBox = pos[0] >= this->CropRegion[0] && pos[0] <= this->CropRegion[1] &&
          pos[1] >= this->CropRegion[2] && pos[1] <= this->CropRegion[3] &&
          pos[2] >= this->CropRegion[4] && pos[2] <= this->CropRegion[5];
        return ((pointOutsideOfBox && !this->CropInside) ||
            (!pointOutsideOfBox && this->CropInside));
        break;
      }
      case Spherical:
      // Spherical mode
      {
        double R = std::sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
        bool pointInsideOfBounds;
        if(this->CropRegion[0] <= this->CropRegion[1])
          {
          pointInsideOfBounds = theta >= this->CropRegion[0] && theta <= this->CropRegion[1] &&
          R >= this->CropRegion[4] && R <= this->CropRegion[5];
          }
        else
          {
          pointInsideOfBounds = (theta >= this->CropRegion[0] || theta <= this->CropRegion[1]) &&
          R >= this->CropRegion[4] && R <= this->CropRegion[5];
          }
        return ((pointInsideOfBounds && !this->CropInside) ||
            (!pointInsideOfBounds && this->CropInside));
        break;
      }
      case Cylindric:
      {
        // space holder for future implementation
      }
    }
    return false;
  }
//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::PushFiringData(const unsigned char laserId,
                                                       const unsigned char rawLaserId,
                                                       unsigned short azimuth,
                                                       const double timestamp,
                                                       const unsigned int rawtime,
                                                       const HDLLaserReturn* laserReturn,
                                                       const HDLLaserCorrection* correction,
                                                       vtkTransform* geotransform,
                                                       const bool hasDualReturn)
{
  azimuth %= 36000;
  const vtkIdType thisPointId = this->Points->GetNumberOfPoints();
  short intensity = laserReturn->intensity;

  // Compute raw position
  double distanceM;
  double pos[3];
  bool applyIntensityCorrection =
      this->WantIntensityCorrection && this->IsHDL64Data && !(this->SensorPowerMode == CorrectionOn);
  ComputeCorrectedValues(azimuth, laserReturn, correction,
                         pos, distanceM, intensity, applyIntensityCorrection);

  // Apply sensor transform
  this->SensorTransform->InternalTransformPoint(pos, pos);

  if (this->shouldBeCroppedOut(pos,static_cast<double> (azimuth)/100.0))
    return;

  // Do not add any data before here as this might short-circuit
  if (hasDualReturn)
    {
    const vtkIdType dualPointId = this->LastPointId[rawLaserId];
    if (dualPointId < this->FirstPointIdOfDualReturnPair)
      {
      // No matching point from first set (skipped?)
      this->Flags->InsertNextValue(DUAL_DOUBLED);
      this->DistanceFlag->InsertNextValue(0);
      this->DualReturnMatching->InsertNextValue(-1); //std::numeric_limits<vtkIdType>::quiet_NaN()
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
          // Apply geoposition transform
          geotransform->InternalTransformPoint(pos, pos);
          this->Points->SetPoint(dualPointId, pos);
          this->Distance->SetValue(dualPointId, distanceM);
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
      //The first return indicates the dual return
      //and the dual return indicates the first return
      this->DualReturnMatching->InsertNextValue(dualPointId);
      this->DualReturnMatching->SetValue(dualPointId,thisPointId);
      }
    }
  else
    {
    this->Flags->InsertNextValue(DUAL_DOUBLED);
    this->DistanceFlag->InsertNextValue(0);
    this->IntensityFlag->InsertNextValue(0);
    this->DualReturnMatching->InsertNextValue(-1); //std::numeric_limits<vtkIdType>::quiet_NaN()
    }

  // Apply geoposition transform
  geotransform->InternalTransformPoint(pos, pos);
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
  this->LastPointId[rawLaserId] = thisPointId;
  this->VerticalAngle->InsertNextValue(this->laser_corrections_[laserId].verticalCorrection);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::InitTrigonometricTables()
{
  if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
    {
    cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
      {
      double rad = HDL_Grabber_toRadians(i / 100.0);
      cos_lookup_table_[i] = std::cos(rad);
      sin_lookup_table_[i] = std::sin(rad);
      }
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile(const std::string& correctionsFile)
{
  boost::property_tree::ptree pt;
  try
    {
    read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
    }
  catch (boost::exception const&)
    {
    vtkGenericWarningMacro("LoadCorrectionsFile: error reading calibration file: " << correctionsFile);
    return;
    }
  // Read distLSB if provided
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB"))
    {
    if (v.first == "distLSB_")
      {// Stored in cm in xml
      distanceResolutionM = atof(v.second.data().c_str()) / 100.0;
      }
    }

  int i, j;
  i = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type &p, pt.get_child("boost_serialization.DB.colors_"))
    {
    if(p.first == "item")
      {
      j = 0;
      BOOST_FOREACH (boost::property_tree::ptree::value_type &v, p.second.get_child("rgb"))
          if(v.first == "item")
          {
            std::stringstream ss;
            double val;
            ss << v.second.data();
            ss >> val;

            XMLColorTable[i][j] = val;
            j++;
          }
      i++;
      }
    }

  int enabledCount = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.enabled_"))
    {
    std::stringstream ss;
    if(v.first == "item")
      {
      ss << v.second.data();
      int test = 0;
      ss >> test;
      if(!ss.fail() && test == 1)
        {
        enabledCount++;
        }
      }
    }
  this->CalibrationReportedNumLasers = enabledCount;

  // Getting min & max intensities from XML
  int laserId = 0;
  int minIntensity[HDL_MAX_NUM_LASERS], maxIntensity[HDL_MAX_NUM_LASERS];
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.minIntensity_"))
    {
    std::stringstream ss;
    if(v.first == "item")
      {
      ss << v.second.data();
      ss >> minIntensity[laserId];
      laserId++;
      }
    }

  laserId = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.maxIntensity_"))
  {
  std::stringstream ss;
  if(v.first == "item")
    {
    ss << v.second.data();
    ss >> maxIntensity[laserId];
    laserId++;
    }
  }

  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.points_"))
    {
    if (v.first == "item")
      {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH (boost::property_tree::ptree::value_type &px, points)
        {
        if (px.first == "px")
          {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          HDLLaserCorrection xmlData={0};

          BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
            {
            if (item.first == "id_")
              index = atoi(item.second.data().c_str());
            if (item.first == "rotCorrection_")
              xmlData.rotationalCorrection = atof(item.second.data().c_str());
            if (item.first == "vertCorrection_")
              xmlData.verticalCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrection_")
              xmlData.distanceCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrectionX_")
              xmlData.distanceCorrectionX = atof(item.second.data().c_str());
            if (item.first == "distCorrectionY_")
              xmlData.distanceCorrectionY = atof(item.second.data().c_str());
            if (item.first == "vertOffsetCorrection_")
              xmlData.verticalOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "horizOffsetCorrection_")
              xmlData.horizontalOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "focalDistance_")
              xmlData.focalDistance = atof(item.second.data().c_str());
            if (item.first == "focalSlope_")
              xmlData.focalSlope = atof(item.second.data().c_str());
            if (item.first == "closeSlope_")
              xmlData.closeSlope = atof(item.second.data().c_str());
            }
          if (index != -1 && index < HDL_MAX_NUM_LASERS)
            {
            laser_corrections_[index]=xmlData;
            // Angles are already stored in degrees in xml
            // Distances are stored in centimeters in xml, and we store meters.
            laser_corrections_[index].distanceCorrection /= 100.0;
            laser_corrections_[index].distanceCorrectionX /= 100.0;
            laser_corrections_[index].distanceCorrectionY /= 100.0;
            laser_corrections_[index].verticalOffsetCorrection /= 100.0;
            laser_corrections_[index].horizontalOffsetCorrection /= 100.0;
            laser_corrections_[index].focalDistance /= 100.0;
            laser_corrections_[index].focalSlope /= 100.0;
            laser_corrections_[index].closeSlope /= 100.0;
            if(laser_corrections_[index].closeSlope == 0.0)
              laser_corrections_[index].closeSlope = laser_corrections_[index].focalSlope;
            laser_corrections_[index].minIntensity = minIntensity[index];
            laser_corrections_[index].maxIntensity = maxIntensity[index];
            }
          }
        }
      }
    }

  int idx=0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v,
                 pt.get_child("boost_serialization.DB.minIntensity_"))
    {
    std::stringstream ss;
    if(v.first == "item")
      {
      ss << v.second.data();
      int intensity = 0;
      ss >> intensity;
      if(!ss.fail()  && idx < HDL_MAX_NUM_LASERS)
        {
        laser_corrections_[idx].minIntensity = intensity;
        }
      idx++;
      }
    }

  idx=0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v,
                 pt.get_child("boost_serialization.DB.maxIntensity_"))
    {
    std::stringstream ss;
    if(v.first == "item")
      {
      ss << v.second.data();
      int intensity = 0;
      ss >> intensity;
      if(!ss.fail()  && idx < HDL_MAX_NUM_LASERS)
        {
        laser_corrections_[idx].maxIntensity = intensity;
        }
      idx++;
      }
    }

  PrecomputeCorrectionCosSin();
  this->CorrectionsInitialized = true;
}
void vtkVelodyneHDLReader::vtkInternal::PrecomputeCorrectionCosSin()
  {

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
    {
      HDLLaserCorrection &correction = laser_corrections_[i];
      correction.cosVertCorrection =
          std::cos (HDL_Grabber_toRadians(correction.verticalCorrection));
      correction.sinVertCorrection =
          std::sin (HDL_Grabber_toRadians(correction.verticalCorrection));
      correction.cosRotationalCorrection =
          std::cos (HDL_Grabber_toRadians(correction.rotationalCorrection));
      correction.sinRotationalCorrection =
          std::sin (HDL_Grabber_toRadians(correction.rotationalCorrection));
      correction.sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
      correction.cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::Init()
{
  this->InitTrigonometricTables();
  this->SensorTransform->Identity();
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::vtkInternal::isCurrentFrameValid()
{
  return (this->CurrentDataset->GetNumberOfPoints() != 0 &&
    this->currentRpm != std::numeric_limits<double>::infinity());
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::SplitFrame(bool force)
{
  /*if(this->skipFirstFrame)
    {
    this->skipFirstFrame = false;
    return;
    }*/
  if (!this->isCurrentFrameValid() && !force)
    {
    return;
    }
  if(this->SplitCounter > 0 && !force)
    {
    this->SplitCounter--;
    return;
    }

  for (size_t n = 0; n < HDL_MAX_NUM_LASERS; ++n)
    {
    this->LastPointId[n] = -1;
    }

  this->CurrentDataset->SetVerts(this->NewVertexCells(this->CurrentDataset->GetNumberOfPoints()));
  this->Datasets.push_back(this->CurrentDataset);
  this->CurrentDataset = this->CreateData(0);
}

//-----------------------------------------------------------------------------
double vtkVelodyneHDLReader::vtkInternal::ComputeTimestamp(
  unsigned int tohTime)
{
  static const double hourInMilliseconds = 3600.0 * 1e6;

  if (tohTime < this->LastTimestamp)
    {
    if (!vtkMath::IsFinite(this->TimeAdjust))
      {
      // First adjustment; must compute adjustment number
      if (this->Interp && this->Interp->GetNumberOfTransforms())
        {
        const double ts = static_cast<double>(tohTime) * 1e-6;
        const double hours = (this->Interp->GetMinimumT() - ts) / 3600.0;
        this->TimeAdjust = vtkMath::Round(hours) * hourInMilliseconds;
        }
      else
        {
        // Ought to warn about this, but happens when applogic is checking that
        // we can read the file :-(
        this->TimeAdjust = 0;
        }
      }
    else
      {
      // Hour has wrapped; add an hour to the update adjustment value
      this->TimeAdjust += hourInMilliseconds;
      }
    }

  this->LastTimestamp = tohTime;
  return static_cast<double>(tohTime) + this->TimeAdjust;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::ComputeOrientation(
  double timestamp, vtkTransform* geotransform)
{
  if(this->ApplyTransform && this->Interp && this->Interp->GetNumberOfTransforms())
    {
    // NOTE: We store time in milliseconds, but the interpolator uses seconds,
    //       so we need to adjust here
    const double t = timestamp * 1e-6;
    this->Interp->InterpolateTransform(t, geotransform);
    }
  else
    {
    geotransform->Identity();
    }
}

void vtkVelodyneHDLReader::vtkInternal::ComputeCorrectedValues(
    const unsigned short azimuth, const HDLLaserReturn* laserReturn,
    const HDLLaserCorrection* correction, double pos[3], double & distanceM , short & intensity, bool correctIntensity)
  {
  intensity = laserReturn->intensity;

  double cosAzimuth, sinAzimuth;
  if (correction->rotationalCorrection == 0)
  {
    cosAzimuth = this->cos_lookup_table_[azimuth];
    sinAzimuth = this->sin_lookup_table_[azimuth];
  }
  else
  {
    // realAzimuth = azimuth/100 - rotationalCorrection
    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    cosAzimuth = this->cos_lookup_table_[azimuth] * correction->cosRotationalCorrection
                 + this->sin_lookup_table_[azimuth] * correction->sinRotationalCorrection;
    sinAzimuth = this->sin_lookup_table_[azimuth] * correction->cosRotationalCorrection
                 - this->cos_lookup_table_[azimuth] * correction->sinRotationalCorrection;
  }
  // Compute the distance in the xy plane (w/o accounting for rotation)
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathemathical
   * model we used.
   */
  double distanceMRaw = laserReturn->distance * this->distanceResolutionM;
  distanceM = distanceMRaw + correction->distanceCorrection;
  double xyDistance = distanceM * correction->cosVertCorrection
                      - correction->sinVertOffsetCorrection;

  pos[0] = xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth;
  pos[1] = xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth;
  pos[2] = distanceM * correction->sinVertCorrection + correction->verticalOffsetCorrection;

  if (correctIntensity && (correction->minIntensity < correction->maxIntensity)){
    // Compute corrected intensity

    /* Please refer to the manual:
      "Velodyne, Inc. ©2013  63‐HDL64ES3 REV G" Appendix F. Pages 45-46
      PLease note: in the manual, focalDistance is in centimeters, distance is the raw short from the laser
      & the graph is in meter */

    // Casting the input values to double for the computation

    double computedIntensity = static_cast<double>(intensity);
    double minIntensity = static_cast<double>(correction->minIntensity);
    double maxIntensity = static_cast<double>(correction->maxIntensity);

    //Rescale the intensity between 0 and 255
    computedIntensity = (computedIntensity - minIntensity) / (maxIntensity - minIntensity) *255.0;

    if (computedIntensity < 0)
      {
      computedIntensity = 0;
      }

    double focalOffset = 256*pow(1.0 - correction->focalDistance / 131.0, 2);
    double insideAbsValue = std::abs(focalOffset -
                256*pow(1.0 - static_cast<double>(laserReturn->distance)/65535.0f, 2));

    if (insideAbsValue > 0)
      {
      computedIntensity = computedIntensity + correction->focalSlope * insideAbsValue;
      }
    else
      {
      computedIntensity = computedIntensity + correction->closeSlope * insideAbsValue;
      }
    computedIntensity = std::max( std::min(computedIntensity, 255.0),1.0);

    intensity = static_cast<short>(computedIntensity);
    }
 
  }

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::ProcessFiring(HDLFiringData* firingData,
                                                      int firingBlockLaserOffset,
                                                      int firingBlock,
                                                      int azimuthDiff,
                                                      double timestamp,
                                                      unsigned int rawtime,
                                                      vtkTransform* geotransform)
{
  // Here the logic is the following: Each firing in a packet encodes 32 lasers.
  // Hence two consecutive firings with same azimuth denotes dual-return
  //    for sensors with 32 dsr or less
  // For 64 lasers sensors, dual return datas is detected if firing idx 2 has same
  //  rotationalPosition as firing idx 1.
  //  Once DualReturnMode is detected, dual returns are firing 2,3 6,7 10,11 [0-based]
  const bool isThisFiringDualReturnData =
    (!this->IsHDL64Data)?(this->LastAzimuth == firingData->rotationalPosition):
    (((this->LastAzimuth == firingData->rotationalPosition) && (firingBlock == 2))
      || this->IsDualReturnSensorMode && (firingBlock % 4 >=2));

  this->HasDualReturn = this->HasDualReturn || isThisFiringDualReturnData;

  if (isThisFiringDualReturnData  && !this->IsDualReturnSensorMode)
    {
    this->IsDualReturnSensorMode = true;
    this->CurrentDataset->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
    this->CurrentDataset->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
    this->CurrentDataset->GetPointData()->AddArray(this->DualReturnMatching.GetPointer());
    }

  if(!isThisFiringDualReturnData
       && (!this->IsHDL64Data || (this->IsHDL64Data && ((firingBlock % 4)==0))))
    {
    this->FirstPointIdOfDualReturnPair = this->Points->GetNumberOfPoints();
    }

  for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++)
    {
    const unsigned char rawLaserId = static_cast<unsigned char>(dsr + firingBlockLaserOffset);
    unsigned char laserId = rawLaserId;
    const unsigned short azimuth = firingData->rotationalPosition;

    // Detect VLP-16 data and adjust laser id if necessary
    int firingWithinBlock = 0;

    if(this->CalibrationReportedNumLasers == 16)
      {
      if(firingBlockLaserOffset != 0)
        {
        if (!this->alreadyWarnedForIgnoredHDL64FiringPacket)
          {
          vtkGenericWarningMacro("Error: Received a HDL-64 UPPERBLOCK firing packet "
                      "with a VLP-16 calibration file. Ignoring the firing.");
          this->alreadyWarnedForIgnoredHDL64FiringPacket = true;
          }
        return;
        }
      if(laserId >= 16)
        {
        laserId -= 16;
        firingWithinBlock = 1;
        }
      }

    // Interpolate azimuths and timestamps per laser within firing blocks
    double timestampadjustment, blockdsr0, nextblockdsr0;
    int azimuthadjustment;
    switch(this->CalibrationReportedNumLasers){
    case 64:
      {
      timestampadjustment = -HDL64EAdjustTimeStamp(firingBlock, dsr, this->IsDualReturnSensorMode);
      nextblockdsr0 = -HDL64EAdjustTimeStamp(firingBlock + (this->IsDualReturnSensorMode?4:2), 0, this->IsDualReturnSensorMode);
      blockdsr0 = -HDL64EAdjustTimeStamp(firingBlock, 0, this->IsDualReturnSensorMode);
      break;
      }
    case 32:
      {
      if (this->ReportedSensor == VLP32AB || this->ReportedSensor == VLP32C)
        {
        timestampadjustment = VLP32AdjustTimeStamp(firingBlock, dsr,this->IsDualReturnSensorMode);
        nextblockdsr0 = VLP32AdjustTimeStamp(firingBlock + (this->IsDualReturnSensorMode?2:1), 0, this->IsDualReturnSensorMode);
        blockdsr0 = VLP32AdjustTimeStamp(firingBlock, 0, this->IsDualReturnSensorMode);
        }
      else
        {
        timestampadjustment = HDL32AdjustTimeStamp(firingBlock, dsr,this->IsDualReturnSensorMode);
        nextblockdsr0 = HDL32AdjustTimeStamp(firingBlock + (this->IsDualReturnSensorMode?2:1), 0, this->IsDualReturnSensorMode);
        blockdsr0 = HDL32AdjustTimeStamp(firingBlock, 0, this->IsDualReturnSensorMode);
        }
      break;
      }
    case 16:
      {
      timestampadjustment = VLP16AdjustTimeStamp(firingBlock, laserId, firingWithinBlock, this->IsDualReturnSensorMode);
      nextblockdsr0 = VLP16AdjustTimeStamp(firingBlock + (this->IsDualReturnSensorMode?2:1), 0, 0, this->IsDualReturnSensorMode);
      blockdsr0 = VLP16AdjustTimeStamp(firingBlock, 0, 0, this->IsDualReturnSensorMode);
      break;
      }
    default:
      {
      timestampadjustment = 0.0;
      blockdsr0 = 0.0;
      nextblockdsr0 = 1.0;
      }
    }
    azimuthadjustment = vtkMath::Round(azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
    timestampadjustment = vtkMath::Round(timestampadjustment);

    if (firingData->laserReturns[dsr].distance != 0.0 && this->LaserSelection[laserId])
      {
      this->PushFiringData(laserId,
                           rawLaserId,
                           azimuth + azimuthadjustment,
                           timestamp + timestampadjustment,
                           rawtime + static_cast<unsigned int>(timestampadjustment),
                           &(firingData->laserReturns[dsr]),
                           &(laser_corrections_[dsr + firingBlockLaserOffset]),
                           geotransform,
                           isThisFiringDualReturnData );
      }
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived)
{
  if (bytesReceived != 1206)
    {
    // Data-Packet Specifications says that laser-packets are 1206 byte long.
    //  That is : (2+2+(2+1)*32)*12 + 4 + 1 + 1
    //                #lasers^   ^#firingPerPkt
    return;
    }

  HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);

  vtkNew<vtkTransform> geotransform;
  const unsigned int rawtime = dataPacket->gpsTimestamp;
  const double timestamp = this->ComputeTimestamp(dataPacket->gpsTimestamp);
  this->ComputeOrientation(timestamp, geotransform.GetPointer());

  // Update the transforms here and then call internal
  // transform
  this->SensorTransform->Update();
  geotransform->Update();

  int firingBlock = this->Skip;
  this->Skip = 0;

  // Compute the total azimuth advanced during one full firing block
  std::vector<int> diffs (HDL_FIRING_PER_PKT - 1);
  for(int i = 0; i < HDL_FIRING_PER_PKT - 1; ++i)
    {
    int localDiff = (36000 + dataPacket->firingData[i+1].rotationalPosition -
                     dataPacket->firingData[i].rotationalPosition) % 36000;
    diffs[i] = localDiff;
    }

  this->IsHDL64Data |= dataPacket->isHDL64();

  if(!IsHDL64Data){ // with HDL64, it should be filled by LoadCorrectionsFromStreamData
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
  assert(azimuthDiff > 0);

  //If it is the first packet of the current frame (ie : the first angle is not defined yet)
  if(this->firstAngle >= std::numeric_limits<int>::max())
    {
    //Save the "first angle" of the frame = last angle of the first packet
    this->firstAngle = dataPacket->firingData[HDL_FIRING_PER_PKT-1].rotationalPosition;
    //Save the first timestamp of the frame = timestamp of the first packet
    this->firstTimestamp = timestamp;//timestamp;
    }
  for ( ; firingBlock < HDL_FIRING_PER_PKT; ++firingBlock)
    {
    HDLFiringData* firingData = &(dataPacket->firingData[firingBlock]);
    int multiBlockLaserIdOffset = (firingData->blockIdentifier == BLOCK_0_TO_31) ? 0 :
                                  (firingData->blockIdentifier == BLOCK_32_TO_63 ? 32 : 0);

    if (firingData->rotationalPosition < this->LastAzimuth)
      {
      //At the end of a frame : 
      //Compute the deltaAngle / deltaTime (in rpm)
      double deltaRotation = static_cast<double>(this->LastAzimuth - this->firstAngle)/(36000.0); //in number of lap
      double deltaTime = (static_cast<double>(timestamp)-firstTimestamp)/(60e6); //in minutes
      this->currentRpm = deltaRotation/deltaTime;
      //Put the current rpm in a FieldData attached to the current dataset
      this->CurrentDataset->GetFieldData()->GetArray("RotationPerMinute")->SetTuple1(0,this->currentRpm);
      //Create a new dataset (new frame)
      this->SplitFrame();
      this->LastAzimuth = -1;
      this->LastTimestamp = std::numeric_limits<unsigned int>::max();
      this->firstAngle = std::numeric_limits<int>::max();
      }

    // Skip this firing every PointSkip
    if(this->PointsSkip == 0 || firingBlock % (this->PointsSkip + 1) == 0)
      {
      this->ProcessFiring(firingData,
                          multiBlockLaserIdOffset,
                          firingBlock,
                          azimuthDiff,
                          timestamp,
                          rawtime,
                          geotransform.GetPointer());
      }

    this->LastAzimuth = firingData->rotationalPosition;
    }
}

//-----------------------------------------------------------------------------
double vtkVelodyneHDLReader::GetDistanceResolutionM()
{
  return this->Internal->distanceResolutionM;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::ReadFrameInformation()
{
  vtkPacketFileReader reader;
  if (!reader.Open(this->FileName))
    {
    vtkErrorMacro("Failed to open packet file: " << this->FileName << endl << reader.GetLastError());
    return 0;
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  unsigned int lastAzimuth = 0;
  unsigned int lastTimestamp = 0;

  std::vector<fpos_t> filePositions;
  std::vector<int> skips;
  unsigned long long numberOfFiringPackets= 0;

  fpos_t lastFilePosition;
  reader.GetFilePosition(&lastFilePosition);

  bool IsHDL64Data = false;
  bool isEmptyFrame = true;
  while (reader.NextPacket(data, dataLength, timeSinceStart))
    {

    if (dataLength != 1206)
      {
      continue;
      }

    const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data);
    numberOfFiringPackets++;

//    unsigned int timeDiff = dataPacket->gpsTimestamp - lastTimestamp;
//    if (timeDiff > 600 && lastTimestamp != 0)
//      {
//      printf("missed %d packets\n",  static_cast<int>(floor((timeDiff/553.0) + 0.5)));
//      }

    for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
      {
      const HDLFiringData & firingData = dataPacket->firingData[i];

      IsHDL64Data |= (firingData.blockIdentifier == BLOCK_32_TO_63);

      // Test if all lasers had a positive distance
      for(int laserID = 0; laserID < HDL_LASER_PER_FIRING; laserID++)
        {
        if(firingData.laserReturns[laserID].distance != 0)
            isEmptyFrame = false;
        }

      if (firingData.rotationalPosition < lastAzimuth)
        {
        // Add file position if the frame is not empty
        if(!isEmptyFrame)
        {
          filePositions.push_back(lastFilePosition);
          skips.push_back(i);
        }
        this->UpdateProgress(0.0);
        // We start a new frame, reinitialize the boolean
        isEmptyFrame = true;
        }

      lastAzimuth = firingData.rotationalPosition;
      }

    // Accumulate HDL64 Status byte data
    if(IsHDL64Data && this->Internal->IsCorrectionFromLiveStream
       && !this->Internal->CorrectionsInitialized)
      {
        this->appendRollingDataAndTryCorrection(data);
      }
    lastTimestamp = dataPacket->gpsTimestamp;
    reader.GetFilePosition(&lastFilePosition);
    }

  this->Internal->FilePositions = filePositions;
  this->Internal->Skips = skips;
  return this->GetNumberOfFrames();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::updateReportedSensor(const unsigned char* data)
{
    if(HDLDataPacket::isValidPacket(data, 1206))
    {
      const HDLDataPacket * dataPacket = reinterpret_cast<const HDLDataPacket *>(data);
      this->Internal->IsHDL64Data = dataPacket->isHDL64();
      this->Internal->ReportedSensor = dataPacket->getSensorType();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetSelectedPointsWithDualReturn(double *data, int Npoints)
{
  this->Internal->SelectedDualReturn = vtkSmartPointer<vtkDoubleArray>::New();
  this->Internal->SelectedDualReturn->Allocate(60000);
  this->Internal->SelectedDualReturn->SetName("dualReturn_of_selectedPoints");

  for(unsigned int k=0; k < Npoints ;++k)
    {
    this->Internal->SelectedDualReturn->InsertNextValue(data[k]);
    }
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::GetHasDualReturn()
{
  return this->Internal->HasDualReturn;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::isReportedSensorAndCalibrationFileConsistent(bool shouldWarn)
{
  //Get the number of laser from sensor type
  int reportedSensorNumberLaser = num_laser(this->Internal->ReportedSensor);

  //compare the numbers of lasers
  if(reportedSensorNumberLaser != this->Internal->CalibrationReportedNumLasers)
    {
    if(shouldWarn && !this->Internal->AlreadyWarnAboutCalibration)
      {
      std::stringstream warningMessage;
      if (reportedSensorNumberLaser == 0)
        {
        warningMessage << "The data-packet from the sensor has an unrecognised factory byte";
        }
      else
        {
        warningMessage << "The data-packet from the sensor has a factory byte recognized"
                       << " as having " << reportedSensorNumberLaser << " lasers";
        }
      warningMessage << ", Veloview will interpret data-packets and show points"
        << " based on the XML calibration file only (currently: "
        << this->Internal->CalibrationReportedNumLasers << " lasers).";
      vtkGenericWarningMacro(<<warningMessage.str());
      this->Internal->AlreadyWarnAboutCalibration = true;
      }
    return false;
    }

  return true;
}

#pragma pack(push, 1)
// Following struct are direct mapping from the manual
//      "Velodyne, Inc. ©2013  63‐HDL64ES3 REV G" Appendix E. Pages 31-42
struct HDLLaserCorrectionByte
  {
  // This is the per laser 64-byte struct in the rolling data
  // It corresponds to 4 cycles of (9 HW status bytes + 7 calibration bytes)
  // WARNING data in packets are little-endian, which enables direct casting
  //  in short ONLY on little-endian machines (Intel & Co are fine)

  //Cycle n+0
  unsigned char hour_cycle0;
  unsigned char minutes_cycle0;
  unsigned char seconds_cycle0;
  unsigned char day_cycle0;
  unsigned char month_cycle0;
  unsigned char year_cycle0;
  unsigned char gpsSignalStatus_cycle0;
  unsigned char temperature_cycle0;
  unsigned char firmwareVersion_cycle0;
  unsigned char warningBit; // 'U' in very first cycle (laser #0)
  unsigned char reserved1;  // 'N' in very first cycle (laser #0)
  unsigned char reserved2;  // 'I' in very first cycle (laser #0)
  unsigned char reserved3;  // 'T' in very first cycle (laser #0)
  unsigned char reserved4;  // '#' in very first cycle (laser #0)
  unsigned char upperBlockThreshold;  // only in very first cycle (laser #0)
  unsigned char lowerBlockThreshold;  // only in very first cycle (laser #0)

  //Cycle n+1
  unsigned char hour_cycle1;
  unsigned char minutes_cycle1;
  unsigned char seconds_cycle1;
  unsigned char day_cycle1;
  unsigned char month_cycle1;
  unsigned char year_cycle1;
  unsigned char gpsSignalStatus_cycle1;
  unsigned char temperature_cycle1;
  unsigned char firmwareVersion_cycle1;

  unsigned char channel;
  signed short verticalCorrection;    // This is in 100th of degree
  signed short rotationalCorrection;  // This is in 100th of degree
  signed short farDistanceCorrection; // This is in millimeter
  //Cycle n+2
  unsigned char hour_cycle2;
  unsigned char minutes_cycle2;
  unsigned char seconds_cycle2;
  unsigned char day_cycle2;
  unsigned char month_cycle2;
  unsigned char year_cycle2;
  unsigned char gpsSignalStatus_cycle2;
  unsigned char temperature_cycle2;
  unsigned char firmwareVersion_cycle2;

  signed short distanceCorrectionX;
  signed short distanceCorrectionV;
  signed short verticalOffset;

  unsigned char horizontalOffsetByte1;
  //Cycle n+3
  unsigned char hour_cycle3;
  unsigned char minutes_cycle3;
  unsigned char seconds_cycle3;
  unsigned char day_cycle3;
  unsigned char month_cycle3;
  unsigned char year_cycle3;
  unsigned char gpsSignalStatus_cycle3;
  unsigned char temperature_cycle3;
  unsigned char firmwareVersion_cycle3;

  unsigned char horizontalOffsetByte2;

  signed short focalDistance;
  signed short focalSlope;

  unsigned char minIntensity;
  unsigned char maxIntensity;
  };

struct last4cyclesByte{
  //Cycle n+0
  unsigned char hour_cycle0;
  unsigned char minutes_cycle0;
  unsigned char seconds_cycle0;
  unsigned char day_cycle0;
  unsigned char month_cycle0;
  unsigned char year_cycle0;
  unsigned char gpsSignalStatus_cycle0;
  unsigned char temperature_cycle0;
  unsigned char firmwareVersion_cycle0;

  unsigned char calibration_year;
  unsigned char calibration_month;
  unsigned char calibration_day;
  unsigned char calibration_hour;
  unsigned char calibration_minutes;
  unsigned char calibration_seconds;
  unsigned char humidity;
  //Cycle n+1
  unsigned char hour_cycle1;
  unsigned char minutes_cycle1;
  unsigned char seconds_cycle1;
  unsigned char day_cycle1;
  unsigned char month_cycle1;
  unsigned char year_cycle1;
  unsigned char gpsSignalStatus_cycle1;
  unsigned char temperature_cycle1;
  unsigned char firmwareVersion_cycle1;

  signed short motorRPM;
  unsigned short fovStartAngle; // in 100th of degree
  unsigned short fovEndAngle; // in 100th of degree
  unsigned char realLifeTimeByte1;
  //Cycle n+2
  unsigned char hour_cycle2;
  unsigned char minutes_cycle2;
  unsigned char seconds_cycle2;
  unsigned char day_cycle2;
  unsigned char month_cycle2;
  unsigned char year_cycle2;
  unsigned char gpsSignalStatus_cycle2;
  unsigned char temperature_cycle2;
  unsigned char firmwareVersion_cycle2;

  unsigned char realLifeTimeByte2;

  unsigned char sourceIPByte1;
  unsigned char sourceIPByte2;
  unsigned char sourceIPByte3;
  unsigned char sourceIPByte4;

  unsigned char destinationIPByte1;
  unsigned char destinationIPByte2;
  //Cycle n+3
  unsigned char hour_cycle3;
  unsigned char minutes_cycle3;
  unsigned char seconds_cycle3;
  unsigned char day_cycle3;
  unsigned char month_cycle3;
  unsigned char year_cycle3;
  unsigned char gpsSignalStatus_cycle3;
  unsigned char temperature_cycle3;
  unsigned char firmwareVersion_cycle3;

  unsigned char destinationIPByte3;
  unsigned char destinationIPByte4;
  unsigned char multipleReturnStatus; // 0= Strongest, 1= Last, 2= Both
  unsigned char reserved3;
  unsigned char powerLevelStatus;
  unsigned short calibrationDataCRC;

  };

#pragma pack(pop)
//
bool vtkVelodyneHDLReader::vtkInternal::HDL64LoadCorrectionsFromStreamData()
  {
  std::vector<unsigned char> data;
  if(!this->rollingCalibrationData->getAlignedRollingData(data))
    {
    return false;
    }
  // the rollingCalibrationData considers the marker to be "#" in reserved4
  const int idxDSRDataFromMarker = static_cast<int>(
                                     -reinterpret_cast<unsigned long>
                                    (&((HDLLaserCorrectionByte*)0)->reserved4));
  const int HDL64_RollingData_NumLaser = 64;
  for (int dsr = 0; dsr < HDL64_RollingData_NumLaser; ++dsr)
    {
    const HDLLaserCorrectionByte * correctionStream=
        reinterpret_cast<const HDLLaserCorrectionByte*>
        // The 64 here is the length of the 4 16-byte cycle
        //    containing one dsr information
          (&data[idxDSRDataFromMarker + 64 * dsr]);
    if (correctionStream->channel != dsr)
      {
      return false;
      }
    HDLLaserCorrection & vvCorrection = laser_corrections_[correctionStream->channel];
    vvCorrection.verticalCorrection = correctionStream->verticalCorrection / 100.0;
    vvCorrection.rotationalCorrection = correctionStream->rotationalCorrection / 100.0;
    vvCorrection.distanceCorrection = correctionStream->farDistanceCorrection / 1000.0;

    vvCorrection.distanceCorrectionX = correctionStream->distanceCorrectionX / 1000.0;
    vvCorrection.distanceCorrectionY = correctionStream->distanceCorrectionV / 1000.0;
    vvCorrection.verticalOffsetCorrection = correctionStream->verticalOffset / 1000.0;
    // The following manipulation is needed because of the two byte for this
    //  parameter are not side-by-side
    vvCorrection.horizontalOffsetCorrection =
        this->rollingCalibrationData->fromTwoLittleEndianBytes<signed short>(
                                correctionStream->horizontalOffsetByte1,
                                correctionStream->horizontalOffsetByte2) / 1000.0;
    vvCorrection.focalDistance = correctionStream->focalDistance / 1000.0;
    vvCorrection.focalSlope = correctionStream->focalSlope / 1000.0;
    vvCorrection.closeSlope = correctionStream->focalSlope / 1000.0;
    vvCorrection.minIntensity = correctionStream->minIntensity;
    vvCorrection.maxIntensity = correctionStream->maxIntensity;
    }

  //Get the last cycle of live correction file
  const last4cyclesByte* lastCycle =
      reinterpret_cast<const last4cyclesByte*>
        (&data[idxDSRDataFromMarker + 64 * HDL64_RollingData_NumLaser]);
  this->SensorPowerMode = lastCycle->powerLevelStatus;
  this->ReportedSensorReturnMode = ((lastCycle->multipleReturnStatus == 0)?STRONGEST_RETURN:
                                ((lastCycle->multipleReturnStatus == 1)?LAST_RETURN:DUAL_RETURN));

  this->CalibrationReportedNumLasers = HDL64_RollingData_NumLaser;
  PrecomputeCorrectionCosSin();
  this->CorrectionsInitialized = true;
  }

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::appendRollingDataAndTryCorrection(const unsigned char* data) {
  const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data);
  this->Internal->rollingCalibrationData->appendData(
        dataPacket->gpsTimestamp,
        dataPacket->factoryField1, dataPacket->factoryField2);
  this->Internal->HDL64LoadCorrectionsFromStreamData();
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::getIsHDL64Data()
{
  return this->Internal->IsHDL64Data;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::IsIntensityCorrectedBySensor()
{
  return this->Internal->SensorPowerMode == CorrectionOn;
}

//-----------------------------------------------------------------------------
bool vtkVelodyneHDLReader::getCorrectionsInitialized()
{
  return this->Internal->CorrectionsInitialized;
}

//-----------------------------------------------------------------------------
const bool& vtkVelodyneHDLReader::GetWantIntensityCorrection()
{
  return this->Internal->WantIntensityCorrection;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetIntensitiesCorrected(const bool& state)
{

  if (state != this->Internal->WantIntensityCorrection)
    {
    this->Internal->WantIntensityCorrection = state;
    this->Modified();
    }

}
