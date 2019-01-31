// Copyright 2018 Kitware SAS
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
  Module:    vtkMotionDetector.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_MOTION_DETECTOR_H
#define VTK_MOTION_DETECTOR_H

// LOCAL
#include "vtkSphericalMap.h"

// STD
#include <cstring>
#include <iostream>
#include <stdint.h>

// VTK
#include <vtkPolyData.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

// EIGEN
#include <Eigen/Dense>

class VTK_EXPORT vtkMotionDetector : public vtkPolyDataAlgorithm
{
public:
  static vtkMotionDetector *New();
  vtkTypeMacro(vtkMotionDetector, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent);

  // Add a frame to update the motion estimator
  void AddFrame(vtkSmartPointer<vtkPolyData>& polydata);

  // Reset the vtkMotionDetector algorithm
  void ResetAlgorithm();

protected:
  // constructor / destructor
  vtkMotionDetector();
  ~vtkMotionDetector();

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  // copy operators
  vtkMotionDetector(const vtkMotionDetector&);
  void operator=(const vtkMotionDetector&);

  // Each points that have been add to
  // the motion detector is map into a
  // Spherical Map and add to a distribution
  // modelized by a gaussian distribution. The
  // Gaussian map correspond to the map of gaussian
  // distributions along the vertical and azimuth angles
  vtkSphericalMap GaussianMap;
};

#endif // VTK_MOTION_DETECTOR_H