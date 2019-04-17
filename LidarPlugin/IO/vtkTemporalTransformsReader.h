// Copyright 2019 Kitware SAS.
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

#ifndef VTKTEMPORALTRANSFORMSREADER_H
#define VTKTEMPORALTRANSFORMSREADER_H

// STD
#include <string>

// VTK
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>
#include "vtkTemporalTransforms.h"

/**
 * @brief vtkTemporalTransformsReader reads a csv file to generate a vtkTemporalTransform.
 *
 * The cvs file is expected to respect the following specification:
 * - Element separator = ","
 * - Line separator = "\n"
 * - Number of element >= 7
 * - header example = "time,roll,pitch,yaw,X,Y,Z"
 * - time  : expressed in microseconds
 * - roll  : expresses the sensor rotation around the X axis and is in degree
 * - pitch : expresses the sensor rotation around the Y axis and is in degree
 * - yaw   : expresses the sensor rotation around the Z axis and is in degree
 * - the rotation matrix can be recomposed this way: R = Rz(z)*Ry(y)*Rx(x)
 *
 * Remark: if you get from VeloView UI the error:
 * "vtkSIProxyDefinitionManager: No proxy that matches: group= and proxy= were found."
 * when you tried to open a file with a ".csv" extension and was asked to chose
 * between standard "Delimited Text" and "pose trajectory" then it means you
 * did not click on "pose trajectory".
 * This is a bug solved in recent ParaViews.
 * see: https://gitlab.kitware.com/paraview/paraview/issues/17594
 */
class VTK_EXPORT vtkTemporalTransformsReader : public vtkPolyDataReader
{
public:
  static vtkTemporalTransformsReader* New();
  vtkTypeMacro(vtkTemporalTransformsReader,vtkPolyDataReader)

  static vtkSmartPointer<vtkTemporalTransforms> OpenTemporalTransforms(const std::string& filename);

  //@{
  /**
   * @copydoc vtkTemporalTransformsReader::TimeOffset
   */
  vtkGetMacro(TimeOffset, double)
  vtkSetMacro(TimeOffset, double)
  //@}

protected:
  vtkTemporalTransformsReader();

  //! Read the data from the csv file and fill TrajectoryCahce
  //! can throw an error
  void ReadData();

  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector);

private:
  //! TimeOffset in seconds relative to the system clock
  double TimeOffset = 0.0;

  vtkTemporalTransformsReader(const vtkTemporalTransformsReader&) = delete;
  void operator =(const vtkTemporalTransformsReader&) = delete;

};

#endif // VTKTEMPORALTRANSFORMSREADER_H
