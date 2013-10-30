// Copyright 2013 Velodyne Acoustics, Inc.
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
  Module:    vtkVelodyneOffsetFilter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkVelodyneOffsetFilter.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"
#include "vtkTupleInterpolator.h"
#include "vtkPointData.h"
#include "vtkMath.h"
#include "vtkTransform.h"

//----------------------------------------------------------------------------
class vtkVelodyneOffsetFilter::vtkInternal
{
public:

  vtkInternal()
  {
  }

  ~vtkInternal()
  {
  }

  vtkSmartPointer<vtkTupleInterpolator> Interp;
};

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneOffsetFilter);

//----------------------------------------------------------------------------
vtkVelodyneOffsetFilter::vtkVelodyneOffsetFilter()
{
  this->Internal = new vtkInternal;
}

//----------------------------------------------------------------------------
vtkVelodyneOffsetFilter::~vtkVelodyneOffsetFilter()
{
  delete this->Internal;
}

//----------------------------------------------------------------------------
void vtkVelodyneOffsetFilter::SetInterp(vtkTupleInterpolator* interp)
{
  this->Internal->Interp = interp;
}

//----------------------------------------------------------------------------
int vtkVelodyneOffsetFilter::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPointSet *output = vtkPointSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkPointSet *input = vtkPointSet::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));

  output->DeepCopy(input);

  double mint = this->Internal->Interp->GetMinimumT();
  double maxt = this->Internal->Interp->GetMaximumT();
  double range[2];
  range[0] = mint;
  range[1] = maxt;

  vtkPoints* points = output->GetPoints();
  vtkDataArray* times = output->GetPointData()->GetArray("timestamp");
  if(!times)
    {
    vtkErrorMacro("No time data in point set");
    return 0;
    }

  // Rotate
  vtkNew<vtkTransform> transform;

  assert(times->GetNumberOfTuples() == points->GetNumberOfPoints());
  for(vtkIdType i = 0; i < times->GetNumberOfTuples(); ++i)
    {
    transform->Identity();

    double t = times->GetTuple1(i);
    vtkMath::ClampValue(&t,range);

    assert(this->Internal->Interp->GetNumberOfComponents() == 5);
    double positiondata[5];
    this->Internal->Interp->InterpolateTuple(t, positiondata);

    double offset[3];
    std::copy(positiondata, positiondata + 3, offset);

    double angle = atan2(positiondata[4], positiondata[3]);
    angle = (angle > 0 ? angle : (2*vtkMath::Pi() + angle)) * 360 / (2*vtkMath::Pi());
    angle = -fmod(angle, 360);

    transform->RotateZ(angle);

    double pt[3];
    points->GetPoint(i, pt);

    transform->TransformPoint(pt, pt);

    double offsetpt[3];
    vtkMath::Add(offset, pt, offsetpt);

    points->SetPoint(i, offsetpt);
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkVelodyneOffsetFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
