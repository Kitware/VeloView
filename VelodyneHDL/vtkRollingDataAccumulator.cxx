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

#include "vtkRollingDataAccumulator.h"
#include "vtkVelodyneHDLReader.h"

#pragma pack(push, 1)
struct HDLLaserCorrectionByte
    // This is the 64-byte struct inn the rolling data
{
  unsigned char channel;
  unsigned short verticalCorrection;
  unsigned short rotationalCorrection;
  unsigned short farDistanceCorrection;

  unsigned char  dummychar1;
  unsigned short dummyshort11;
  unsigned short dummyshort12;
  unsigned short dummyshort13;
  unsigned short dummyshort14;
  unsigned short dummyshort15;

  short distanceCorrectionX;
  short distanceCorrectionV;
  short verticalOffset;

  unsigned char horizontalOffsetPart1;

  unsigned short dummyshort21;
  unsigned short dummyshort22;
  unsigned short dummyshort23;
  unsigned short dummyshort24;
  unsigned short dummyshort25;

  unsigned char horizontalOffsetPart2;

  unsigned short focalDistance;
  unsigned short focalSlope;

  unsigned char minIntensity;
  unsigned char maxIntensity;
};

vtkRollingDataAccumulator::vtkRollingDataAccumulator()
{
}

//--------------------------------------------------------------------------------
vtkRollingDataAccumulator::~vtkRollingDataAccumulator()
{
//  this->Close();
}


void vtkRollingDataAccumulator::appendData(TypeValueDataPair valuePair)
  {
    if (valuePair.dataType == '5' && valuePair.dataValue == '#')
      {
      beginPosition.push_back(this->accumulatedData.size());
      }
    this->accumulatedData.push_back(valuePair);
    this->accumulatedDataType.push_back(valuePair.dataType);
    this->accumulatedValue.push_back(valuePair.dataValue);
  }
void vtkRollingDataAccumulator::isCalibrationReady()
  {
    // We want to have at least twice the data, to be sure.
    return this->accumulatedData.size() > 4160*2;
  }

void vtkRollingDataAccumulator::appendData(unsigned int timestamp,
                                           unsigned char dataType, unsigned char dataValue)
  {
    this->appendData(TypeValueDataPair(timestamp,dataType,dataValue));
  }

bool vtkRollingDataAccumulator::getDSRCalibrationData()
  {
    if(!this->isCalibrationReady())
      {
        return;
      }
    long idxStart=beginPosition.front()+12;

    for (int dsr = 0; dsr < HDL_MAX_NUM_LASERS; ++dsr) {
        HDLLaserCorrectionByte * correctionStream=
            reinterpret_cast<HDLLaserCorrectionByte>(&accumulatedValue[idxStart]);
        if (correctionStream->channel != dsr){
            return false;
          }
        HDLLaserCorrection vvCorrection;
        vvCorrection.verticalCorrection = correctionStream->verticalCorrection/100.0;
        vvCorrection.rotationalCorrection = correctionStream->rotationalCorrection/100.0;
        vvCorrection.distanceCorrection = correctionStream->farDistanceCorrection/10.0;
        
        vvCorrection.distanceCorrectionX = correctionStream->distanceCorrectionX/10.0;
        vvCorrection.distanceCorrectionY = correctionStream->distanceCorrectionV/10.0;
        vvCorrection.verticalOffsetCorrection = correctionStream->verticalOffset/10.0;
        // The folowing manipulation is needed because of the two byte for this
        //  parameter are not side-by-side
        vvCorrection.horizontalOffsetCorrection = reinterpret_cast<signed short>(
          (static_cast<unsigned short>(correctionStream->horizontalOffsetPart1) << 8)
          +static_cast<unsigned short>(correctionStream->horizontalOffsetPart2)) /10.0;
        vvCorrection.focalDistance = correctionStream->focalDistance/10.0;
        vvCorrection.focalSlope = correctionStream->focalSlope/10.0;
      }

  }
