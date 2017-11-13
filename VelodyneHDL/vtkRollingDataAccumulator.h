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
#include "vtkObject.h"
#include <vector>

#ifndef VTKROLLINGDATAACCUMULATOR_H
#define VTKROLLINGDATAACCUMULATOR_H
struct TypeValueDataPair
{
  unsigned int timestamp;
  unsigned char dataType;
  unsigned char dataValue;
  TypeValueDataPair(unsigned int timestamp_, unsigned char dataType_, unsigned char dataValue_)
    : timestamp(timestamp_)
    , dataType(dataType_)
    , dataValue(dataValue_)
  {
  }
};

class vtkRollingDataAccumulator
{
public:
  void appendData(TypeValueDataPair valuePair);
  void appendData(unsigned int timestamp, unsigned char dataType, unsigned char dataValue);
  void setTotalExpectedDataLength();
  bool areRollingDataReady() const;
  bool getDSRCalibrationData() const;
  bool getGoodSequenceId(int& idRollingSequence) const;
  bool getAlignedRollingData(std::vector<unsigned char>& data) const;
  void clear();
  vtkRollingDataAccumulator();
  ~vtkRollingDataAccumulator();

  template<typename T>
  T fromTwoLittleEndianBytes(unsigned char b1, unsigned char b2)
  {
    return static_cast<T>((static_cast<unsigned short>(b2) << 8) + static_cast<unsigned short>(b1));
  }

protected:
  std::vector<TypeValueDataPair> accumulatedData;
  std::vector<unsigned char> accumulatedDataType;
  std::vector<unsigned char> accumulatedValue;
  std::vector<long> beginPosition;

private:
  static const long expectedLength = 4160;
  static const int numberOfRoundNeeded = 3;
  static const int byteBeforeMarker = 6;
  const TypeValueDataPair beginMarkerValuePair;
};
#endif // VTKROLLINGDATAACCUMULATOR_H
