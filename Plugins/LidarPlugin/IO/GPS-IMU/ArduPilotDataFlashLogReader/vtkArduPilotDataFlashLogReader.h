//=========================================================================
//
// Copyright 2019 Kitware, Inc.
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
//=========================================================================

#ifndef VTKARDUPILOTDATAFLASHLOGREADER_H
#define VTKARDUPILOTDATAFLASHLOGREADER_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkTemporalTransforms.h>
// official documentation: http://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html
// Log structure: https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L1166
// GPS/RTK Data: http://ardupilot.org/copter/docs/common-gps-blending.html

/**
 * @brief Parse some DataFlash messages produced by ArduPilot.
 * Currently we only parse parts of the GPS message.
 */
class VTK_EXPORT vtkArduPilotDataFlashLogReader : public vtkPolyDataAlgorithm
{
public:
  static vtkArduPilotDataFlashLogReader *New();
  vtkTypeMacro(vtkArduPilotDataFlashLogReader, vtkPolyDataAlgorithm);

  vtkGetStringMacro(FileName)
  vtkSetStringMacro(FileName)

  vtkGetMacro(TimeOffset, double)
  vtkSetMacro(TimeOffset, double)

  vtkGetMacro(SignedUTMZone, int)

  vtkGetVector3Macro(Offset, double)

protected:
  vtkArduPilotDataFlashLogReader();
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

  vtkSmartPointer<vtkTemporalTransforms> GPSTrajectory;

  char* FileName = nullptr;
  double TimeOffset = 0.0; // in seconds, used as input to allow timeshifting
  int SignedUTMZone; // UTM zone used. +N means "UTM zone N North" (South if -N)
  // Offset is the (easting, northing, altitude) that is used as an offset for
  // the whole trajectory, in order to work with reasonably big coordinates.
  // Add the Offset to all the points of the trajectory to get UTM coordinates.
  double Offset[3] = { 0.0, 0.0, 0.0 }; // in meters
private:
  vtkArduPilotDataFlashLogReader(const vtkArduPilotDataFlashLogReader&) = delete;
  void operator = (const vtkArduPilotDataFlashLogReader&) = delete;
};

#endif // VTKARDUPILOTDATAFLASHLOGREADER_H
