// Copyright 2017 Actemium.
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
  Module:    vtkStabilizationManager.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkMotionDetector.h"

#include <vtkDataSet.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataWriter.h>

// VTK
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkQuaternion.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedShortArray.h>
#include <vtkTransform.h>
#include <vtkPoints.h>
#include <vtkAppendFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkQuaternion.h>

// STD
#include <iostream>

// Implementation of the New function
vtkStandardNewMacro(vtkMotionDetector)

//----------------------------------------------------------------------------
vtkMotionDetector::vtkMotionDetector()
{
  // One input port
  this->SetNumberOfInputPorts(1);

  // The accumulation of stabilized frames
  this->SetNumberOfOutputPorts(1);

  // Initialize the intern parameters
  this->ResetAlgorithm();
}

//----------------------------------------------------------------------------
vtkMotionDetector::~vtkMotionDetector()
{

}

//----------------------------------------------------------------------------
void vtkMotionDetector::ResetAlgorithm()
{
  // reset the spherical map
  this->GaussianMap.ResetMap();
}

//-----------------------------------------------------------------------------
void vtkMotionDetector::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
void vtkMotionDetector::AddFrame(vtkSmartPointer<vtkPolyData>& polydata)
{
  // Add the new points into the Gaussian Map
  this->GaussianMap.AddFrame(polydata);
}

//-----------------------------------------------------------------------------
int vtkMotionDetector::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  std::cout << "Motion Detector asked" << std::endl;
  // Get input data
  vtkPolyData* input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  // Get the output
  vtkPolyData* output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(input);

  // Add the new points into the Gaussian Map
  this->GaussianMap.AddFrame(output);

  return 1;
}