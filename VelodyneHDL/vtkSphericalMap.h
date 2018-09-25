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
  Module:    vtkSphericalMap.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_SPHERICAL_MAP_H
#define VTK_SPHERICAL_MAP_H

// VTK
#include <vtkMath.h>
#include <vtkPolyData.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

// EIGEN
#include <Eigen/Dense>

// STD
#include <vector>
#include <cmath>
#include <list>

class Gaussian
{
public:
  Gaussian();
  double operator()(double x);

  // set the mean
  void SetMean(double mean);

  // update parameters using new value
  void UpdateParam(double x);

  // Update the time to live
  // return false if ttl falls
  // to zero value
  bool UpdateTTL();

  // return sigma value
  double GetSigma();

private:
  // mean of the gaussian
  double Mean;

  // standard deviation of the gaussian
  double Sigma;

  // number of points
  unsigned int N;

  // Time to live of the gaussian
  int TTL;

  // Maximum number of the TTL
  int MaxTTL;
};

class GaussianMixture
{
public:
  // default constructor
  GaussianMixture();

  // Evaluation
  double operator()(double x);

  // Add a data
  void AddPoint(double x);

  // return the number of points
  unsigned int GetNumberOfPoints();

  // Update the time to live
  // of the gaussians composing
  // the gaussian mixture model
  void UpdateTTL();

  // Evaluate the probability
  // of a point to be in motion
  double Evaluate(double x);

private:
  // mean of the gaussian
  double mean;

  // standard deviation of the gaussian
  double sigma;

  // points in the distribution
  std::list<Gaussian> Gaussians;
};


class vtkSphericalMap
{
public:
  // default constructor
  vtkSphericalMap();

  // Getter / Setter NPhi
  void SetNPhi(unsigned int phi);
  unsigned int GetNPhi();

  // Getter / Setter NTheta
  void SetNTheta(unsigned int theta);
  unsigned int GetNTheta();

  // Reset the Spehrical map
  // using the current parameters
  void ResetMap();

  // Add a frame to map in the spherical map and
  // update the map using the new input data
  void AddFrame(vtkSmartPointer<vtkPolyData> polydata);

  // Add a point to the corresponding pixel
  void AddPoint(unsigned int idxTheta, unsigned int idxPhi, double valueDepth);

  // Get the total number of Points stored
  unsigned int GetNumberOfPoints();

  // Update the TTL of the gaussians mixtures
  void UpdateTTL();

  // Set the sensor RPM
  void SetSensorRPM(double rpm);

private:
  // Number of sample points along
  // Phi parameter (vertical angle
  // between 90 and -90 degrees)
  unsigned int NPhi;

  // Number of sample points along
  // Theta parameter (azimuth angle
  // between 0 and 360 degrees)
  unsigned int NTheta;

  // total number of sample
  // ie NTheta x NPhi
  unsigned int NSample;

  // Theta and Phi quantum
  // ie dTheta = 360 / NTheta
  // ie dPhi = 180 / NPhi
  double dPhi;
  double dTheta;

  // Spherical map bounds
  // phi -> [-90, 90]
  // theta -> [0, 360]
  double ThetaBounds[2];
  double PhiBounds[2];

  // RPM of the sensor
  double SensorRPM;

  // Indicate the number of frame
  // processed by the algorithm
  unsigned int AddedFrames;

  // The spherical map
  std::vector<GaussianMixture> Map;

  // Base of R3 used. in some
  // case it can be changed
  Eigen::Matrix<double, 3, 1> ez;
  Eigen::Matrix<double, 3, 1> ey;
  Eigen::Matrix<double, 3, 1> ex;

  // Center of the coordinate system
  // using the ex, ey, ez base
  Eigen::Matrix<double, 3, 1> C;

  // Convert cartesian coordinates of X point in its
  // spherical coordinates
  Eigen::Matrix<double, 3, 1> GetSphericalCoordinates(const Eigen::Matrix<double, 3, 1>& X);

  // export as images parameters
  bool ShouldExportAsImage;
  std::string filenameBase;
};

#endif // VTK_SPHERICAL_MAP_H