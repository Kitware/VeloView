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

//-----------------------------------------------------------------------------
Eigen::MatrixXd ComputeResidualValues(std::vector<Eigen::Matrix<double, 3, 1> > X, std::vector<Eigen::Matrix<double, 3, 1> > Y,
                                      Eigen::Matrix<double, 3, 3> R, Eigen::Matrix<double, 3, 1> T)
{
  Eigen::MatrixXd residuals(X.size(), 1);
  Eigen::Matrix<double, 3, 1> Xp;
  for (unsigned int k = 0; k < X.size(); ++k)
  {
    Xp = R * X[k] + T;
    residuals(k) = std::sqrt(((Xp - Y[k]).transpose() * (Xp - Y[k]))(0));
  }
  return residuals;
}

//-----------------------------------------------------------------------------
Eigen::MatrixXd ComputeResidualJacobians(std::vector<Eigen::Matrix<double, 3, 1> > X, std::vector<Eigen::Matrix<double, 3, 1> > Y,
                                         Eigen::Matrix<double, 3, 1> angles, Eigen::Matrix<double, 3, 1> T)
{
  // get current parameters
  Eigen::MatrixXd residualsJacobians(X.size(), 6);
  double epsilon = 1e-5;
  double rx, ry, rz;
  rx = angles(0); ry = angles(1); rz = angles(2);
  double X1, X2, X3;
  double Y1, Y2, Y3;
  Eigen::Matrix<double, 3, 3> R = GetRotationMatrix(angles);
  Eigen::Matrix<double, 3, 3> Id = Eigen::Matrix<double, 3, 3>::Identity();

  // cosinus and sinus of the current
  // estimated angles for the ego-motion
  // This is done in order to speed the algortihm
  // full rotation
  double crx, srx;
  double cry, sry;
  double crz, srz;
  crx = std::cos(rx); srx = std::sin(rx);
  cry = std::cos(ry); sry = std::sin(ry);
  crz = std::cos(rz); srz = std::sin(rz);

  for (unsigned int k = 0; k < X.size(); ++k)
  {
    X1 = X[k](0); X2 = X[k](1); X3 = X[k](2);
    Y1 = Y[k](0); Y2 = Y[k](1); Y3 = Y[k](2);

    // represents h(R,T)
    Eigen::Matrix<double, 3, 1> h_R_t = R * X[k] + T - Y[k];

    // represent the jacobian of the G function
    // evaluated at the point h(R,T). Note that G is
    // the composition of the functions sqrt and X' * A * X
    // and is not differentiable when X'*A*X = 0
    Eigen::Matrix<double, 1, 3> JacobianG;
    double dist = std::sqrt((h_R_t.transpose() * h_R_t)(0));
    if (dist > 1e-12)
    {
      JacobianG = 1.0 / (2.0 * dist) * h_R_t.transpose() * (Id + Id.transpose());
    }

    // represent the jacobian of the H function
    // evaluated at the point R, T
    Eigen::Matrix<double, 3, 6> JacobianH;
    // dx / drx
    JacobianH(0, 0) = (srz * srx + crz * sry * crx) * X2 + (srz * crx - crz * sry * srx) * X3;
    // dx / dry
    JacobianH(0, 1) = -crz * sry * X1 + crz * cry * srx * X2 + crz * cry * crx * X3;
    // dx / drz
    JacobianH(0, 2) = -srz * cry * X1 + (-crz * crx - srz * sry * srx) * X2+ (crz * srx - srz * sry * crx) * X3;
    // dx / dtx
    JacobianH(0, 3) = 1;
    // dx / dty
    JacobianH(0, 4) = 0;
    // dx / dtz
    JacobianH(0, 5) = 0;
    // dy / drx
    JacobianH(1, 0) = (-crz * srx + srz * sry * crx) * X2 + (-crz * crx - srz * sry * srx) * X3;
    // dy / dry
    JacobianH(1, 1) = -srz * sry * X1 + srz * cry * srx * X2 + srz * cry * crx * X3;
    // dy / drz
    JacobianH(1, 2) = crz * cry * X1 + (-srz * crx + crz * sry * srx) * X2 + (srz * srx + crz * sry * crx) * X3;
    // dy / dtx
    JacobianH(1, 3) = 0;
    // dy / dty
    JacobianH(1, 4) = 1;
    // dy / dtz
    JacobianH(1, 5) = 0;
    // dz / drx
    JacobianH(2, 0) = cry * crx * X2 - cry * srx * X3;
    // dz / dry
    JacobianH(2, 1) = -cry * X1 - sry * srx * X2 - sry * crx * X3;
    // dz / drz
    JacobianH(2, 2) = 0;
    // dz / dtx
    JacobianH(2, 3) = 0;
    // dz / dty
    JacobianH(2, 4) = 0;
    // dr / dtz
    JacobianH(2, 5) = 1;

    Eigen::Matrix<double, 1, 6> currentJacobian = JacobianG * JacobianH;

    for (unsigned int i = 0; i < 6; ++i)
    {
      residualsJacobians(k, i) = currentJacobian(0, i);
    }
  }

  return residualsJacobians;
}

//-----------------------------------------------------------------------------
void GetPairMatchedPoints(vtkVelodyneTransformInterpolator* slam, vtkVelodyneTransformInterpolator* gps,
                          std::vector<Eigen::Matrix<double, 3, 1> >& slamT, std::vector<Eigen::Matrix<double, 3, 1> >& gpsT)
{
  // clear and resize vectors
  slamT.clear(); slamT.resize(0);
  gpsT.clear(); gpsT.resize(0);

  // loop over slam transforms and match with gps
  std::vector<std::vector<double> > slamTransforms = slam->GetTransformList();
  for (unsigned int k = 0; k < slamTransforms.size(); ++k)
  {
    double time = slamTransforms[k][0];

    if (time < gps->GetMinimumT())
    {
      vtkGenericWarningMacro("Time requested too low");
      continue;
    }
    if (time > gps->GetMaximumT())
    {
      vtkGenericWarningMacro("Time requested too high");
      continue;
    }

    vtkNew<vtkTransform> gpsCurrTrans;
    gps->InterpolateTransform(time, gpsCurrTrans.Get());
    gpsCurrTrans->Modified();
    gpsCurrTrans->Update();

    vtkNew<vtkMatrix4x4> M;
    gpsCurrTrans->GetMatrix(M.Get());
    M->Modified();

    Eigen::Matrix<double, 3, 1> Xs, Xg;
    Xs << slamTransforms[k][4], slamTransforms[k][5], slamTransforms[k][6];
    Xg << M->GetElement(0, 3), M->GetElement(1, 3), M->GetElement(2, 3);

    slamT.push_back(Xs);
    gpsT.push_back(Xg);
  }
}

//-----------------------------------------------------------------------------
void vtkSensorTransformFusion::ApplyTransform(Eigen::Matrix<double, 3, 1> angles, Eigen::Matrix<double, 3, 1> T, vtkVelodyneTransformInterpolator* slam)
{
  this->MergedInterp = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();
  this->MergedInterp->SetInterpolationTypeToNearestLowBounded();

  std::vector<std::vector<double> > slamTransforms = slam->GetTransformList();
  Eigen::Matrix<double, 3, 3> Rcorr = GetRotationMatrix(angles);

  for (unsigned int k = 0; k < slamTransforms.size(); ++k)
  {
    Eigen::Matrix<double, 3, 1> anglesSlam, Tslam;

    // Get Slam transforms values
    double tSlam = slamTransforms[k][0];
    Tslam << slamTransforms[k][4], slamTransforms[k][5], slamTransforms[k][6];
    anglesSlam << slamTransforms[k][1], slamTransforms[k][2], slamTransforms[k][3];
    Eigen::Matrix<double, 3, 3> Rslam = GetRotationMatrix(anglesSlam);

    Tslam = Rcorr * Tslam + T;
    Rslam = Rcorr * Rslam;

    Eigen::Matrix<double, 3, 1> newAngles = GetEulerAngles(Rslam);
    newAngles = newAngles / vtkMath::Pi() * 180.0;


    vtkNew<vtkTransform> trans;
    trans->PostMultiply();
    trans->RotateX(newAngles(0));
    trans->RotateY(newAngles(1));
    trans->RotateZ(newAngles(2));
    double pos[3] = {Tslam(0), Tslam(1), Tslam(2)};
    trans->Translate(pos);

    this->MergedInterp->AddTransform(tSlam, trans.GetPointer());
    this->MergedInterp->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSensorTransformFusion::RegisterSlamOnGps(vtkVelodyneTransformInterpolator* slam, vtkVelodyneTransformInterpolator* gps)
{
  // check that the interp are provided
  if (!slam || !gps)
  {
    vtkGenericWarningMacro("Slam or Gps not provided, return");
    return;
  }

  // check that the time interval are consistent
  if ((slam->GetMinimumT() > gps->GetMaximumT()) || (gps->GetMinimumT() > slam->GetMaximumT()))
  {
    vtkGenericWarningMacro("Time intervals not consistent, please sync your devices");
    return;
  }

  // L-M algorithm hyper-parameters
  double lambda = 1.0;
  double ratioLambda = 1.10;

  // Sample Data
  /*int Nsample = 500;
  double dt = 1.0 / static_cast<double>(Nsample);
  std::vector<Eigen::Matrix<double, 3, 1> > slamT, gpsT;

  Eigen::Matrix<double, 3, 1> angles, trans;
  angles << 12.78, -45.65, 78.487;
  angles = angles / 180.0 * vtkMath::Pi();
  trans << 10.056, -12.11, 11.25;

  Eigen::Matrix<double, 3, 3> R = GetRotationMatrix(angles);

  for (unsigned int k = 0; k < Nsample; ++k)
  {
    double time = static_cast<double>(k) * dt;
    Eigen::Matrix<double, 3, 1> X;
    X << std::cos(time * vtkMath::Pi()), std::sin(time * vtkMath::Pi()), std::sin(time * vtkMath::Pi() * 10);

    slamT.push_back(X);
    X = R * X + trans;
    gpsT.push_back(X);
  }*/

  std::vector<Eigen::Matrix<double, 3, 1> > slamT, gpsT;
  GetPairMatchedPoints(slam, gps, slamT, gpsT);

  // Find the rotation and translation that
  // minimize the least square error using
  // a levenberg marquardt algorithm (because
  // the least square is not linear due to
  // the rotational matrix parametrized using
  // Euler angles
  unsigned int maxStep = 1050;
  Eigen::Matrix<double, 6, 1> W = Eigen::Matrix<double, 6, 1>::Zero();
  std::vector<double> errors;
  for (unsigned int lmStep = 0; lmStep < maxStep; ++lmStep)
  {
    // Get current parameters
    Eigen::Matrix<double, 3, 1> currAngles, currT;
    currAngles << W(0), W(1), W(2);
    currT << W(3), W(4), W(5);
    Eigen::Matrix<double, 3, 3> currR = GetRotationMatrix(currAngles);

    // Compute residuals errors and jacobians
    Eigen::MatrixXd residualErrors = ComputeResidualValues(slamT, gpsT, currR, currT);
    Eigen::MatrixXd residualJacobians = ComputeResidualJacobians(slamT, gpsT, currAngles, currT);

    // RMSE
    double currErr = std::sqrt(1.0 / static_cast<double>(residualErrors.rows()) * (residualErrors.transpose() * residualErrors)(0));

    Eigen::MatrixXd Jt = residualJacobians.transpose();
    Eigen::MatrixXd JtJ = Jt * residualJacobians;
    Eigen::MatrixXd JtY = Jt * residualErrors;
    Eigen::Matrix<double, 6, 6> diagJtJ;
    diagJtJ << JtJ(0, 0), 0, 0, 0, 0, 0,
               0, JtJ(1, 1), 0, 0, 0, 0,
               0, 0, JtJ(2, 2), 0, 0, 0,
               0, 0, 0, JtJ(3, 3), 0, 0,
               0, 0, 0, 0, JtJ(4, 4), 0,
               0, 0, 0, 0, 0, JtJ(5, 5);

    // The next step of the L-M algorithm is computed by solving
    // (JtJ + lambda * diagJtJ) = Jt * Y. To avoid the computation
    // of the inverse of (JtJ + lambda * diagJtJ) we use a gauss-pivot
    // algorithm to solve the linear equation for this particular point
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(JtJ + lambda * diagJtJ);
    Eigen::Matrix<double, 6, 1> X = dec.solve(JtY);
    if (!vtkMath::IsFinite(X(0)) || !vtkMath::IsFinite(X(3)))
    {
      vtkGenericWarningMacro("Estimated transform not finite, skip this frame");
      break;
    }

    // Check that the cost function has decreased
    Eigen::MatrixXd Wcandidate = W - 0.1 * X;
    currAngles << Wcandidate(0), Wcandidate(1), Wcandidate(2);
    currT << Wcandidate(3), Wcandidate(4), Wcandidate(5);
    currR = GetRotationMatrix(currAngles);

    // Compute residuals errors and jacobians
    residualErrors = ComputeResidualValues(slamT, gpsT, currR, currT);
    double nextErr = std::sqrt(1.0 / static_cast<double>(residualErrors.rows()) * (residualErrors.transpose() * residualErrors)(0));
    if (nextErr > currErr)
    {
      lambda = ratioLambda * lambda;
    }
    else
    {
      lambda = 1.0 / ratioLambda * lambda;
      W = Wcandidate;
      errors.push_back(currErr);
    }
  }

  std::cout << "Errors: " << std::endl;
  for (unsigned int k = 0; k < errors.size(); ++k)
  {
    std::cout << "step: " << k << " error: " << errors[k] << std::endl;
  }
  std::cout << "total step validated: " << errors.size() << std::endl;
  std::cout << "Computed on: " << slamT.size() << " samples data" << std::endl;
  std::cout << std::endl;

  Eigen::Matrix<double, 3, 1> finalAngles, finalT;
  finalAngles << W(0), W(1), W(2);
  Eigen::Matrix<double, 3, 3> finalRot = GetRotationMatrix(finalAngles);
  finalAngles = GetEulerAngles(finalRot);
  finalT << W(3), W(4), W(5);

  // now Apply the founded rigid transform to the slam
  ApplyTransform(finalAngles, finalT, slam);

  // transform in degree to display result
  finalAngles = finalAngles * 180.0 / vtkMath::Pi();
  std::cout << "Final parameters: " << std::endl;
  std::cout << "angles: " << finalAngles.transpose() << std::endl;
  std::cout << "Positi: " << finalT.transpose() << std::endl;
}