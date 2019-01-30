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
  Module:    vtkVelodyneHDLReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkPlaneFitter.h"

#include "vtkDoubleArray.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkPointSet.h"
#include "vtkSmartPointer.h"
#include "vtkThreshold.h"
#include "vtkUnstructuredGrid.h"

#include <Eigen/Dense>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPlaneFitter);

//-----------------------------------------------------------------------------
vtkPlaneFitter::vtkPlaneFitter()
{
}

//-----------------------------------------------------------------------------
vtkPlaneFitter::~vtkPlaneFitter()
{
}

//-----------------------------------------------------------------------------
void vtkPlaneFitter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
void vtkPlaneFitter::PlaneFit(vtkPointSet* pts, double origin[3], double normal[3], double& minDist,
  double& maxDist, double& stdDev, double channelMean[], double channelStdDev[],
  vtkIdType channelNpts[], unsigned int nchannels)
{
  using namespace Eigen;

  vtkSmartPointer<vtkDoubleArray> ptdata = vtkSmartPointer<vtkDoubleArray>::New();
  ptdata->DeepCopy(pts->GetPoints()->GetData());

  const vtkIdType n = ptdata->GetNumberOfTuples();
  if (n < 1)
  {
    return;
  }

  assert(ptdata->GetNumberOfComponents() == 3);
  Map<MatrixXd> eigpointsraw(static_cast<double*>(ptdata->GetVoidPointer(0)),
    ptdata->GetNumberOfComponents(), ptdata->GetNumberOfTuples());

  MatrixXd eigpoints = eigpointsraw.transpose();

  VectorXd mean(3);

  mean = eigpoints.colwise().sum() / n;
  assert(mean.size() == 3);

  for (int i = 0; i < 3; ++i)
  {
    origin[i] = mean[i];
  }

  eigpoints.rowwise() -= mean.transpose();

  JacobiSVD<MatrixXd> svd(eigpoints, ComputeThinU | ComputeThinV);

  VectorXd enormal = svd.matrixV().col(2);
  assert(enormal.size() == 3);
  assert(std::fabs(enormal.norm() - 1.0) < 1.0e-8);

  for (int i = 0; i < 3; ++i)
  {
    normal[i] = enormal[i];
  }

  VectorXd distances = eigpoints * enormal;
  assert(distances.size() == n);

  minDist = distances.minCoeff();
  maxDist = distances.maxCoeff();

  stdDev = std::sqrt(distances.squaredNorm() / (n - 1));

  for (unsigned int i = 0; i < nchannels; ++i)
  {
    vtkNew<vtkThreshold> threshold;
    threshold->ThresholdBetween(i, i);
    threshold->SetInputData(pts);
    threshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "laser_id");
    threshold->SetOutputPointsPrecision(vtkAlgorithm::DEFAULT_PRECISION);
    threshold->Update();

    channelNpts[i] = threshold->GetOutput()->GetNumberOfPoints();

    vtkSmartPointer<vtkDoubleArray> threshdata = vtkSmartPointer<vtkDoubleArray>::New();
    threshdata->DeepCopy(threshold->GetOutput()->GetPoints()->GetData());

    const vtkIdType n = threshdata->GetNumberOfTuples();
    if (n < 2)
    {
      channelMean[i] = 0.0;
      channelStdDev[i] = 0.0;
      continue;
    }

    assert(threshdata->GetNumberOfComponents() == 3);
    assert(threshdata->GetNumberOfTuples() >= 2);
    Map<MatrixXd> channelraw(static_cast<double*>(threshdata->GetVoidPointer(0)),
      threshdata->GetNumberOfComponents(), threshdata->GetNumberOfTuples());

    MatrixXd channelpts = channelraw.transpose();
    channelpts.rowwise() -= mean.transpose();

    VectorXd channelds = channelpts * enormal;

    double cmean = channelds.sum() / channelds.size();
    double cstddev =
      std::sqrt((channelds.array() - cmean).matrix().squaredNorm() / (channelds.size() - 1));

    channelMean[i] = cmean;
    channelStdDev[i] = cstddev;
  }
}
