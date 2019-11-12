//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Data: 04-01-2019
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

#ifndef VTK_CALIBRATION_FROM_POSES_H
#define VTK_CALIBRATION_FROM_POSES_H

// VTK
#include <vtkPolyDataAlgorithm.h>

/**
 * @brief vtkLaplacianInfilling fill missing data in an image
 *        solving the Dirichlet problem.
 */
class VTK_EXPORT vtkCalibrationFromPoses : public vtkPolyDataAlgorithm
{
public:
  static vtkCalibrationFromPoses *New();
  vtkTypeMacro(vtkCalibrationFromPoses, vtkPolyDataAlgorithm)

  vtkSetMacro(TimeStep, double)
  vtkGetMacro(TimeStep, double)

  vtkSetMacro(TimeScaleAnalysisBound, double)
  vtkGetMacro(TimeScaleAnalysisBound, double)

  vtkSetMacro(TimeScaleAnalysisStep, double)
  vtkGetMacro(TimeScaleAnalysisStep, double)

protected:
  vtkCalibrationFromPoses();
  ~vtkCalibrationFromPoses() = default;

  int FillInputPortInformation(int port, vtkInformation *info);
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

private:
  vtkCalibrationFromPoses(const vtkCalibrationFromPoses&) = delete;
  void operator=(const vtkCalibrationFromPoses&) = delete;

  //! Time step between two consecutive acquisition times to add a residual
  //! function derived from the "solid-system" equations using
  //! relative reference frames
  double TimeStep = 0.4;

  //! Maximum time scale used for the multiple scale analysis
  double TimeScaleAnalysisBound = 5.0;

  //! step between two consecutives scale
  double TimeScaleAnalysisStep = 0.2;
};

#endif // VTK_CALIBRATION_FROM_POSES_H
