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

vtkRollingDataAccumulator::vtkRollingDataAccumulator()
  : beginMarkerValuePair(0, '5', '#')
{
}

//--------------------------------------------------------------------------------
vtkRollingDataAccumulator::~vtkRollingDataAccumulator()
{
}
void vtkRollingDataAccumulator::clear()
{
  this->beginPosition.clear();
  this->accumulatedData.clear();
  this->accumulatedDataType.clear();
  this->accumulatedValue.clear();
}

void vtkRollingDataAccumulator::appendData(TypeValueDataPair valuePair)
{
  if (valuePair.dataType == this->beginMarkerValuePair.dataType &&
    valuePair.dataValue == this->beginMarkerValuePair.dataValue)
  {
    beginPosition.push_back(this->accumulatedData.size());
  }
  this->accumulatedData.push_back(valuePair);
  this->accumulatedDataType.push_back(valuePair.dataType);
  this->accumulatedValue.push_back(valuePair.dataValue);
}
bool vtkRollingDataAccumulator::areRollingDataReady() const
{
  // We want to have received numberOfRoundNeeded times the data, to be sure.
  return (this->accumulatedData.size() > this->expectedLength * numberOfRoundNeeded) &&
    (beginPosition.size() > this->numberOfRoundNeeded - 1);
}
bool vtkRollingDataAccumulator::getGoodSequenceId(int& idRollingSequence) const
{
  idRollingSequence = 0;

  // Prevent seg fault when beginPosition is empty
  if (beginPosition.size() < 3)
    return false;

  while (idRollingSequence < static_cast<int>((beginPosition.size()) - 1) &&
    (beginPosition[idRollingSequence] < byteBeforeMarker ||
      ((beginPosition[idRollingSequence + 1] - beginPosition[idRollingSequence]) !=
        expectedLength)))
  {
    idRollingSequence++;
  }
  if (idRollingSequence == static_cast<int>(beginPosition.size()) - 1)
  {
    idRollingSequence = -1;
    return false;
  }
  return true;
}

void vtkRollingDataAccumulator::appendData(
  unsigned int timestamp, unsigned char dataType, unsigned char dataValue)
{
  this->appendData(TypeValueDataPair(timestamp, dataType, dataValue));
}
bool vtkRollingDataAccumulator::getAlignedRollingData(std::vector<unsigned char>& data) const
{
  data.clear();
  int idRollingSequence = 0;
  if (!this->areRollingDataReady() || !getGoodSequenceId(idRollingSequence))
  {
    return false;
  }
  data.resize(expectedLength);
  memcpy(&data[0], &accumulatedValue[beginPosition[idRollingSequence]], expectedLength);
  return true;
}
