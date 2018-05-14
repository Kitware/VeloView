// Copyright 2017 Kitware.
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
  Module:    vtkSensorTransformFusion.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_SENSOR_TRANSFORM_FUSION_H
#define VTK_SENSOR_TRANSFORM_FUSION_H

// LOCAL
#include "vtkVelodyneTransformInterpolator.h"

// STD
#include <cstring>
#include <fstream>
#include <sstream>
#include <ostream>
#include <iostream>
#include <stdint.h>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/preprocessor.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>

// VTK
#include <vtkPolyData.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

// EIGEN
#include <Eigen/Dense>

class VTK_EXPORT vtkSensorTransformFusion : public vtkPolyDataAlgorithm
{
public:
  static vtkSensorTransformFusion *New();
  vtkTypeMacro(vtkSensorTransformFusion, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  void LoadIMUTransforms(const std::string& filename);
  void LoadSLAMTransforms(const std::string& filename);

  void MergeTransforms();

  vtkVelodyneTransformInterpolator* GetInterpolator() const;

protected:
  // constructor / destructor
  vtkSensorTransformFusion();
  ~vtkSensorTransformFusion();

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  // copy operators
  vtkSensorTransformFusion(const vtkSensorTransformFusion&);
  void operator=(const vtkSensorTransformFusion&);

  std::vector<std::vector<double> > LoadTransforms(const std::string& filename);

  vtkSmartPointer<vtkVelodyneTransformInterpolator> IMUInterp;
  vtkSmartPointer<vtkVelodyneTransformInterpolator> SLAMInterp;
  vtkSmartPointer<vtkVelodyneTransformInterpolator> MergedInterp;
};

#endif // VTK_SENSOR_TRANSFORM_FUSION_H