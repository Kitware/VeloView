// Copyright 2016 Velodyne Lidar, Inc.
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

#ifndef _vtkDataPacket_h
#define _vtkDataPacket_h

#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <unordered_map>
#include <vector>
#ifdef _MSC_VER
#include <boost/cstdint.hpp>
typedef boost::uint8_t uint8_t;
#else
#include <stdint.h>
#endif

namespace DataPacketFixedLength
{
#define HDL_Grabber_toRadians(x) ((x)*vtkMath::Pi() / 180.0)

static const int HDL_NUM_ROT_ANGLES = 36001;
static const int HDL_LASER_PER_FIRING = 32;
static const int HDL_MAX_NUM_LASERS = 128;
static const int HDL_FIRING_PER_PKT = 12;

enum HDLBlock
{
  BLOCK_0_TO_31 = 0xeeff,
  BLOCK_32_TO_63 = 0xddff,
  BLOCK_64_TO_95 = 0xccff,
  BLOCK_96_TO_127 = 0xbbff,
};

enum SensorType
{
  HDL32E = 0x21,     // decimal: 33
  VLP16 = 0x22,      // decimal: 34
  VLP32AB = 0x23,    // decimal: 35
  VLP16HiRes = 0x24, // decimal: 36
  VLP32C = 0x28,     // decimal: 40

  // Work around : this is not defined by any specification
  // But it is usefull to define
  HDL64 = 0xa0,  // decimal: 160
  VLS128 = 0xa1, // decimal: 161
};

static inline std::string SensorTypeToString(SensorType type)
{
  switch (type)
  {
    case SensorType::HDL32E:
      return "HDL-32E";
    case SensorType::VLP16:
      return "VLP-16";
    case SensorType::VLP32AB:
      return "VLP-32AB";
    case SensorType::VLP16HiRes:
      return "VLP-16 Hi-Res";
    case SensorType::VLP32C:
      return "VLP-32C";
    case SensorType::HDL64:
      return "HDL-64";
    case SensorType::VLS128:
      return "VLS-128";
    default:
      return "Unkown";
  }
  /*
  std::unordered_map<SensorType, std::string> toStringMap;
  toStringMap[SensorType::HDL32E] = "HDL-32E";
  toStringMap[SensorType::VLP16] = "VLP-16";
  toStringMap[SensorType::VLP32AB] = "VLP-32AB";
  toStringMap[SensorType::VLP16HiRes] = "VLP-16 Hi-Res";
  toStringMap[SensorType::VLP32C] = "VLP-32C";
  toStringMap[SensorType::HDL64] = "HDL-64";
  toStringMap[SensorType::VLS128] = "VLS-128";
  return toStringMap[type];
  */
}

static inline int num_laser(SensorType sensorType)
{
  switch (sensorType)
  {
    case HDL64:
      return 64;
    case HDL32E:
    case VLP32AB:
    case VLP32C:
      return 32;
    case VLP16:
    case VLP16HiRes:
      return 16;
    case VLS128:
      return 128;
    default:
      return 0;
  }
}

enum DualReturnSensorMode
{
  STRONGEST_RETURN = 0x37,
  LAST_RETURN = 0x38,
  DUAL_RETURN = 0x39,
};

static inline std::string DualReturnSensorModeToString(DualReturnSensorMode type)
{
  switch (type)
  {
    case DualReturnSensorMode::STRONGEST_RETURN:
      return "STRONGEST RETURN";
    case DualReturnSensorMode::LAST_RETURN:
      return "LAST RETURN";
    case DualReturnSensorMode::DUAL_RETURN:
      return "DUAL RETURN";
    default:
      return "Unkown";
  }
  /*
  std::unordered_map<DualReturnSensorMode, std::string> toStringMap;
  toStringMap[DualReturnSensorMode::STRONGEST_RETURN] = "STRONGEST RETURN";
  toStringMap[DualReturnSensorMode::LAST_RETURN] = "LAST RETURN";
  toStringMap[DualReturnSensorMode::DUAL_RETURN] = "DUAL RETURN";
  return toStringMap[type];
  */
}

enum PowerMode
{
  NO_INTERNAL_CORRECTION_0 = 0xa0,
  NO_INTERNAL_CORRECTION_1 = 0xa1,
  NO_INTERNAL_CORRECTION_2 = 0xa2,
  NO_INTERNAL_CORRECTION_3 = 0xa3,
  NO_INTERNAL_CORRECTION_4 = 0xa4,
  NO_INTERNAL_CORRECTION_5 = 0xa5,
  NO_INTERNAL_CORRECTION_6 = 0xa6,
  NO_INTERNAL_CORRECTION_7 = 0xa7,
  CorrectionOn = 0xa8,
};

#pragma pack(push, 1)
typedef struct HDLLaserReturn
{
  uint16_t distance;
  uint8_t intensity;
} HDLLaserReturn;

struct HDLFiringData
{
  uint16_t blockIdentifier;
  uint16_t rotationalPosition;
  HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];

  inline bool isUpperBlock() const { return blockIdentifier == BLOCK_32_TO_63; }
};

struct HDLDataPacket
{
  HDLFiringData firingData[HDL_FIRING_PER_PKT];
  uint32_t gpsTimestamp;
  uint8_t factoryField1;
  uint8_t factoryField2;
  SensorType getSensorType() const
  {
    if (isHDL64())
      return HDL64;
    return static_cast<SensorType>(factoryField2);
  }
  DualReturnSensorMode getDualReturnSensorMode() const
  {
    if (isHDL64())
      return isDualModeReturnHDL64() ? DUAL_RETURN : STRONGEST_RETURN;
    return static_cast<DualReturnSensorMode>(factoryField1);
  }
  static unsigned int getDataByteLength() { return 1206; }
  static inline bool isValidPacket(const unsigned char* data, unsigned int dataLength)
  {
    if (dataLength != getDataByteLength())
      return false;
    const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket*>(data);
    return (dataPacket->firingData[0].blockIdentifier == BLOCK_0_TO_31 ||
      dataPacket->firingData[0].blockIdentifier == BLOCK_32_TO_63);
  }

  inline bool isHDL64() const
  {
    return firingData[1].blockIdentifier == BLOCK_32_TO_63 &&
      firingData[2].blockIdentifier == BLOCK_0_TO_31;
  }

  inline bool isVLS128() const
  {
    return (factoryField2 == VLS128 && !isHDL64());
  }

  inline bool isDualModeReturn() const
  {
    if (isVLS128())
      return isDualModeReturnVLS128();
    if (isHDL64())
      return isDualModeReturnHDL64();
    else
      return isDualModeReturn16Or32();
  }
  inline bool isDualModeReturn16Or32() const
  {
    return firingData[1].rotationalPosition == firingData[0].rotationalPosition;
  }
  inline bool isDualModeReturnHDL64() const
  {
    return firingData[2].rotationalPosition == firingData[0].rotationalPosition;
  }
  inline bool isDualModeReturnVLS128() const { return factoryField1 == DUAL_RETURN; }

  inline bool isDualReturnFiringBlock(const int firingBlock) const
  {
    if (isVLS128())
      return isDualModeReturnVLS128() && isDualBlockOfDualPacket128(firingBlock);
    if (isHDL64())
      return isDualModeReturnHDL64() && isDualBlockOfDualPacket64(firingBlock);
    else
      return isDualModeReturn16Or32() && isDualBlockOfDualPacket16Or32(firingBlock);
  }

  inline static bool isDualBlockOfDualPacket128(const int firingBlock)
  {
    return (firingBlock % 2 == 1);
  }
  inline static bool isDualBlockOfDualPacket64(const int firingBlock)
  {
    return (firingBlock % 4 >= 2);
  }
  inline static bool isDualBlockOfDualPacket16Or32(const int firingBlock)
  {
    return (firingBlock % 2 == 1);
  }

  inline int getRotationalDiffForVLS128(int firingBlock) const
  {
    if (static_cast<DualReturnSensorMode>(factoryField1) == DUAL_RETURN)
    {
      if (firingBlock > 6)
        firingBlock = 6;
      return static_cast<int>((36000 + firingData[firingBlock + 2].rotationalPosition -
                                firingData[firingBlock].rotationalPosition) %
        36000);
    }
    else
    {
      if (firingBlock % 4 == 3)
        firingBlock--;
      return static_cast<int>((36000 + firingData[firingBlock + 1].rotationalPosition -
                                firingData[firingBlock].rotationalPosition) %
        36000);
    }
  }
};

struct HDLLaserCorrection // Internal representation of per-laser correction
{
  // In degrees
  double rotationalCorrection;
  double verticalCorrection;
  // In meters
  double distanceCorrection;
  double distanceCorrectionX;
  double distanceCorrectionY;

  double verticalOffsetCorrection;
  double horizontalOffsetCorrection;

  double focalDistance;
  double focalSlope;
  double closeSlope; // Used in HDL-64 only

  short minIntensity;
  short maxIntensity;

  // precomputed values
  double sinRotationalCorrection;
  double cosRotationalCorrection;
  double sinVertCorrection;
  double cosVertCorrection;
  double sinVertOffsetCorrection;
  double cosVertOffsetCorrection;
  HDLLaserCorrection()
  {
    rotationalCorrection = verticalCorrection = 0;
    distanceCorrection = distanceCorrectionX = distanceCorrectionY = 0;
    verticalOffsetCorrection = horizontalOffsetCorrection = 0;
    focalDistance = focalSlope = closeSlope = 0;
    minIntensity = maxIntensity = 0;
  }
};

struct HDLRGB
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
#pragma pack(pop)
} // end namespace DataPacketFixedLength
#endif
