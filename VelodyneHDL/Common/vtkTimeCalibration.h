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

#ifndef VTK_TIME_CALIBRATION_H
#define VTK_TIME_CALIBRATION_H

#include "vtkEigenTools.h"

#include <vtkObject.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>
#include <vtkVelodyneTransformInterpolator.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "vvConfigure.h"
#include "vtkTemporalTransforms.h"

enum class CorrelationStrategy
{
  DPOS,
  SPEED_WINDOW,
  ACC_WINDOW,
  JERK_WINDOW,
  LENGTH, // time_window_width around 1 s
  DERIVATED_LENGTH, // time_window_width around 1 s
  TRAJECTORY_ANGLE, // time_window_width around 10 s
  DROT,
  ORIENTATION_ANGLE, // time_window_width around 1 s
  DERIVATED_ORIENTATION_ARC,
};

std::string ToString(CorrelationStrategy strategy);

/**
 * \brief Compute the timeshift in seconds between both pose trajectories.
 *
 * The timeshift returned must be substracted to the timestamps of the "aligned"
 * signal in order to have it synchronized with "reference".
 * Some methods use only the position part of the pose trajectories, other use
 * only the orientation part of the pose trajectory.
 * Steps:
 * 1) Compute the two 1D signals using the method correlationStrategy
 * 2) resample them
 * 3) FFT then iFFT
 **/
double VelodyneHDLPlugin_EXPORT ComputeTimeShift(vtkSmartPointer<vtkTemporalTransforms> reference,
                      vtkSmartPointer<vtkTemporalTransforms> aligned,
                      CorrelationStrategy correlationStrategy,
                      double time_window_width,
                      bool substract_mean = true);

void ShowTrajectoryInfo(vtkSmartPointer<vtkTemporalTransforms> reference,
                    vtkSmartPointer<vtkTemporalTransforms> aligned);

void DemoAllTimesyncMethods(vtkSmartPointer<vtkTemporalTransforms> reference,
                               vtkSmartPointer<vtkTemporalTransforms> aligned);

/** \bried Compute the scale between two signals that are time synchronized
 *
 * If you rescale the "aligned" pose trajectory by the inverse of the scale
 * returned, both pose trajectories will be on scale.
 **/
double VelodyneHDLPlugin_EXPORT ComputeScale(vtkSmartPointer<vtkTemporalTransforms> reference,
                  vtkSmartPointer<vtkTemporalTransforms> aligned,
                  CorrelationStrategy correlationStrategy,
                  double time_window_width);

#endif
