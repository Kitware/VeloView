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
  Module:    vtkSensorTransformFusion.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// LOCAL
#include "vtkSensorTransformFusion.h"

// VTK
#include <vtkDataSet.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataWriter.h>
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

struct Pose
{
  Eigen::Matrix<double, 3, 3> R;
  Eigen::Matrix<double, 3, 1> T;
  double time;
};

//-----------------------------------------------------------------------------
double ComputeTimestamp(vtkSmartPointer<vtkVelodyneTransformInterpolator> Interp, double tohTime)
{
  double TimeAdjust = 0;
  static const double hourInMilliseconds = 3600.0 * 1e6;
  // First adjustment; must compute adjustment number
  if (Interp && Interp->GetNumberOfTransforms())
  {
    const double ts = static_cast<double>(tohTime) * 1e-6;
    const double hours = (Interp->GetMinimumT() - ts) / 3600.0;
    TimeAdjust = vtkMath::Round(hours) * hourInMilliseconds;
  }
  else
  {
    // Ought to warn about this, but happens when applogic is checking that
    // we can read the file :-(
    TimeAdjust = 0;
  }

  return static_cast<double>(tohTime) + TimeAdjust;
}

//-----------------------------------------------------------------------------
void ApplyPose(std::vector<Pose>& posesToTransforms, Pose poseToApply, unsigned int start)
{
  for (unsigned int k = start; k < posesToTransforms.size(); ++k)
  {
    posesToTransforms[k].R = poseToApply.R * posesToTransforms[k].R;
    posesToTransforms[k].T = poseToApply.R * posesToTransforms[k].T + poseToApply.T;
  }
}

//-----------------------------------------------------------------------------
Pose ApplyPose(Pose posesToTransforms, Pose poseToApply)
{
  Pose result;
  result.time = posesToTransforms.time;
  result.R = poseToApply.R * posesToTransforms.R;
  result.T = poseToApply.R * posesToTransforms.T + poseToApply.T;
  return result;
}

//-----------------------------------------------------------------------------
void AddTransform(vtkSmartPointer<vtkVelodyneTransformInterpolator> interp, std::vector<std::vector<double> > transforms)
{
  for (unsigned int k = 0; k < transforms.size(); ++k)
  {
    std::vector<double> current = transforms[k];
    double time = current[0];
    double rx = current[1] * 180.0 / vtkMath::Pi();
    double ry = current[2] * 180.0 / vtkMath::Pi();
    double rz = current[3] * 180.0 / vtkMath::Pi();
    double tx = current[4];
    double ty = current[5];
    double tz = current[6];

    vtkNew<vtkTransform> VTKTransform;
    VTKTransform->PostMultiply();
    VTKTransform->RotateX(rx);
    VTKTransform->RotateY(ry);
    VTKTransform->RotateZ(rz);
    double pos[3] = {tx, ty, tz};
    VTKTransform->Translate(pos);

    interp->AddTransform(time, VTKTransform.GetPointer());
    interp->Modified();
  }
}

//-----------------------------------------------------------------------------
Eigen::Matrix<double, 3, 3> GetRotationMatrix(Eigen::Matrix<double, 3, 1> T)
{
  // Rotation and translation relative
  Eigen::Matrix<double, 3, 3> Rx, Ry, Rz, R;
  // rotation around X-axis
  Rx << 1,         0,          0,
        0, cos(T(0)), -sin(T(0)),
        0, sin(T(0)),  cos(T(0));
  // rotation around Y-axis
  Ry <<  cos(T(1)), 0, sin(T(1)),
        0,          1,         0,
        -sin(T(1)), 0, cos(T(1));
  // rotation around Z-axis
  Rz << cos(T(2)), -sin(T(2)), 0,
        sin(T(2)),  cos(T(2)), 0,
                0,          0, 1;

  // full rotation
  R = Rz * Ry * Rx;
  return R;
}

//-----------------------------------------------------------------------------
Eigen::Matrix<double, 3, 1> GetEulerAngles(Eigen::Matrix<double, 3, 3> R)
{
  Eigen::Matrix<double, 3, 1> euler;
  euler(0) = std::atan2(R(2, 1), R(2, 2));
  euler(1) = -std::asin(R(2, 0));
  euler(2) = std::atan2(R(1, 0), R(0, 0));
  return euler;
}

// Implementation of the New function
vtkStandardNewMacro(vtkSensorTransformFusion);

//----------------------------------------------------------------------------
std::vector<std::vector<double> > ConvertToDoubleVector(std::vector<Pose> transforms)
{
  std::vector<std::vector<double> > res;
  for (unsigned int k = 0; k < transforms.size(); ++k)
  {
    Pose temp = transforms[k];
    Eigen::Matrix<double, 3, 1> euler = GetEulerAngles(temp.R);
    std::vector<double> v(7, 0);
    v[0] = temp.time;
    v[1] = euler(0);
    v[2] = euler(1);
    v[3] = euler(2);
    v[4] = temp.T(0);
    v[5] = temp.T(1);
    v[6] = temp.T(2);

    res.push_back(v);
  }
  return res;
}

//----------------------------------------------------------------------------
vtkSensorTransformFusion::vtkSensorTransformFusion()
{
  this->SetNumberOfInputPorts(0);

  // The accumulation of stabilized frames
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkSensorTransformFusion::~vtkSensorTransformFusion()
{

}

//-----------------------------------------------------------------------------
void vtkSensorTransformFusion::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
int vtkSensorTransformFusion::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get the output
  vtkPolyData * output = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  vtkSmartPointer<vtkPolyData> polyData;
  vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();

  output->ShallowCopy(polyData);

  return 1;
}

//-----------------------------------------------------------------------------
void vtkSensorTransformFusion::LoadIMUTransforms(const std::string& filename)
{
  // Initialize the interpolator
  this->IMUInterp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->IMUInterp->SetInterpolationTypeToLinear();

  // load the transforms
  std::vector<std::vector<double> > transforms = this->LoadTransforms(filename);

  // fill the interpolator
  AddTransform(this->IMUInterp, transforms);

  std::cout << "IMU loaded transforms : " << this->IMUInterp->GetNumberOfTransforms() << std::endl;
}

//-----------------------------------------------------------------------------
void vtkSensorTransformFusion::LoadSLAMTransforms(const std::string& filename)
{
  // Initialize the interpolator
  this->SLAMInterp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->SLAMInterp->SetInterpolationTypeToLinear();

  // load the transforms
  std::vector<std::vector<double> > transforms = this->LoadTransforms(filename);

  // fill the interpolator
  AddTransform(this->SLAMInterp, transforms);

  std::cout << "SLAM loaded transforms : " << this->SLAMInterp->GetNumberOfTransforms() << std::endl;
}

//-----------------------------------------------------------------------------
std::vector<std::vector<double> > vtkSensorTransformFusion::LoadTransforms(const std::string& filename)
{
  std::ifstream file;
  file.open(filename);

  if (!file.is_open())
  {
    vtkGenericWarningMacro("Can't load the specified file");
  }

  std::string line;
  std::string expectedLine = "Time,Rx(Roll),Ry(Pitch),Rz(Yaw),X,Y,Z";
  std::getline(file, line);
  if (line != expectedLine)
  {
    vtkGenericWarningMacro("Header file not expected. Version incompability");
  }

  std::vector<std::vector<double> > transforms;
  while (std::getline(file, line))
  {
    std::vector<std::string> values;
    boost::split(values, line, boost::is_any_of(","));

    std::vector<double> current(7, 0);
    // time
    double t = std::atof(values[0].c_str());
    // rotation
    double rx = std::atof(values[1].c_str());
    double ry = std::atof(values[2].c_str());
    double rz = std::atof(values[3].c_str());
    // position
    double x = std::atof(values[4].c_str());
    double y = std::atof(values[5].c_str());
    double z = std::atof(values[6].c_str());
    // add the transform
    current[0] = t;
    current[1] = rx;
    current[2] = ry;
    current[3] = rz;
    current[4] = x;
    current[5] = y;
    current[6] = z;
    transforms.push_back(current);
  }

  return transforms;
}

//-----------------------------------------------------------------------------
void vtkSensorTransformFusion::MergeTransforms()
{
  // check that the data has been loaded
  if (!this->IMUInterp || !this->SLAMInterp)
  {
    vtkGenericWarningMacro("Please, load IMU and SLAM transforms before merging them");
    return;
  }

  // Get the list of slam transforms (low freq)
  std::vector<std::vector<double> > transforms = this->SLAMInterp->GetTransformList();

  // store the list of slam pose
  std::vector<Pose> slamAbsolutePose;
  std::vector<Pose> imuAbsolutePose;
  std::vector<Pose> slamRelativePose;
  std::vector<Pose> imuRelativePose;
  std::vector<Pose> mergedPose;

  double maxSlamTime = this->SLAMInterp->GetMaximumT();
  double minSlamTime = this->SLAMInterp->GetMinimumT();
  double maxImuTime = this->IMUInterp->GetMaximumT();
  double minImuTime = this->IMUInterp->GetMinimumT();
  for (unsigned int k = 0; k < transforms.size(); ++k)
  {
    // SLAM pose
    Eigen::Matrix<double, 3, 1> euler;
    euler << transforms[k][1], transforms[k][2], transforms[k][3];
    Pose temp;
    temp.T << transforms[k][4], transforms[k][5], transforms[k][6];
    temp.R = GetRotationMatrix(euler);
    temp.time = transforms[k][0];
    slamAbsolutePose.push_back(temp);

    // corresponding IMU pose
    vtkNew<vtkTransform> VTKTransform;
    vtkNew<vtkMatrix4x4> M;
    double t = minImuTime + (temp.time - minSlamTime) / (maxSlamTime - minSlamTime) * (maxImuTime - minImuTime);
    this->IMUInterp->InterpolateTransform(t, VTKTransform.GetPointer());
    VTKTransform->GetMatrix(M.GetPointer());

    Pose temp2;
    temp2.time = temp.time;
    temp2.R << M->Element[0][0], M->Element[0][1], M->Element[0][2],
               M->Element[1][0], M->Element[1][1], M->Element[1][2],
               M->Element[2][0], M->Element[2][1], M->Element[2][2];
    temp2.T << M->Element[0][3], M->Element[1][3], M->Element[2][3];
    imuAbsolutePose.push_back(temp2);
  }

  // apply first pose of imu to the transforms of the slam
  //ApplyPose(slamAbsolutePose, imuAbsolutePose[0], 0);

  std::ofstream file;
  file.open("D:/MasterMindData/TestsData/errorEvolution.csv");

  Pose correctionPose = imuAbsolutePose[0];

  // Now we will merge the sensor
  for (unsigned int k = 0; k < slamAbsolutePose.size(); ++k)
  {
    Pose slamPose = slamAbsolutePose[k];
    Pose imuPose = imuAbsolutePose[k];
    Pose newSlamPose = ApplyPose(slamPose, correctionPose);

    Eigen::Matrix<double, 3, 1> deltaT = newSlamPose.T - imuPose.T;

    if (deltaT.norm() > 6.0)
    {
      // Reset the transform to this one
      double errorBefore = deltaT.norm();
      correctionPose.time = slamPose.time;
      correctionPose.R = imuPose.R * slamPose.R.transpose();
      correctionPose.T = imuPose.T - correctionPose.R * slamPose.T;
      newSlamPose = ApplyPose(slamPose, correctionPose);

      deltaT = newSlamPose.T - imuPose.T;
      double errorAfter = deltaT.norm();
      std::cout << "error goes from: " << errorBefore << " to " << errorAfter << std::endl;

      if (std::abs(correctionPose.R(0, 0)) > 2.0)
      {
        std::cout << "Breaked" << std::endl;
        break;
      }
    }

    mergedPose.push_back(newSlamPose);
    deltaT = newSlamPose.T - imuPose.T;
    file << deltaT.norm() << std::endl;
  }

  // Initialize the interpolator
  this->MergedInterp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->MergedInterp->SetInterpolationTypeToLinear();
  AddTransform(this->MergedInterp, ConvertToDoubleVector(mergedPose));
  //this->MergedInterp = this->SLAMInterp;
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator* vtkSensorTransformFusion::GetInterpolator() const
{
  return this->MergedInterp;
}