/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkRansacPlaneModel.cxx
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// LOCAL
#include "vtkRansacPlaneModel.h"

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

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
#include <vtkTransform.h>
#include <vtkTupleInterpolator.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>

// BOOST
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Dense>

//----------------------------------------------------------------------------
struct RansacSampleInfo
{
public:
  RansacSampleInfo() {}

  RansacSampleInfo(unsigned int nInliers, unsigned int index1,
                   unsigned int index2, unsigned int index3)
  {
    this->NInliers = nInliers;
    this->Index1 = index1;
    this->Index2 = index2;
    this->Index3 = index3;
  }

  unsigned int NInliers;
  unsigned int Index1;
  unsigned int Index2;
  unsigned int Index3;
};

//----------------------------------------------------------------------------
void RefineRansac(std::vector<Eigen::Matrix<double, 3, 1> >& Points, vtkPolyData* output,
                  RansacSampleInfo sampleInfo, double threshold, double param[4])
{
  // Create inliers / outliers array information
  vtkNew<vtkUnsignedIntArray> inliersArray;
  inliersArray->SetName("ransac_plane_inliers");

  // compute plane parameters
  Eigen::Matrix<double, 3, 1> pointPlane = Points[sampleInfo.Index1];
  Eigen::Matrix<double, 3, 1> normalPlane = (Points[sampleInfo.Index3] - pointPlane).cross(Points[sampleInfo.Index2] - pointPlane);
  normalPlane.normalize();

  // compute inliers
  std::vector<Eigen::Matrix<double, 3, 1> > inliersPoints;
  for (unsigned int k = 0; k < Points.size(); ++k)
  {
    if (std::abs((Points[k] - pointPlane).dot(normalPlane)) < threshold)
    {
      inliersPoints.push_back(Points[k]);
      inliersArray->InsertNextValue(1);
    }
    else
    {
      inliersArray->InsertNextValue(0);
    }
  }

  // Now, compute the best plane using all inliers
  Eigen::MatrixXd centeredSamples(3, inliersPoints.size());
  Eigen::Matrix<double, 3, 1> center = Eigen::Matrix<double, 3, 1>::Zero();
  for (unsigned int k = 0; k < inliersPoints.size(); ++k)
  {
    centeredSamples.col(k) = inliersPoints[k];
    center += inliersPoints[k];
  }
  center /= static_cast<double>(inliersPoints.size());
  for (unsigned int k = 0; k < inliersPoints.size(); ++k)
  {
    centeredSamples.col(k) -= center;
  }
  Eigen::Matrix<double, 3, 3> varianceCovariance = centeredSamples * centeredSamples.transpose();
  varianceCovariance /= static_cast<double>(inliersPoints.size());

  // since the variance covariance matrix is a real
  // symmetric matrix it can be diagonalized in a orthonormal
  // basis. We will use the AutoAdjoint eigen solver
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3> > eigenSolver(varianceCovariance);

  // parameters
  normalPlane = eigenSolver.eigenvectors().col(0);
  pointPlane = center;
  param[0] = normalPlane(0);
  param[1] = normalPlane(1);
  param[2] = normalPlane(2);
  param[3] = -normalPlane.dot(pointPlane);

  output->GetPointData()->AddArray(inliersArray.Get());
}

//----------------------------------------------------------------------------
void ConvertPolyDataToEigenVector(std::vector<Eigen::Matrix<double, 3, 1> >& Points, vtkPolyData* input)
{
  Eigen::Matrix<double, 3, 1> tempEigenVector;
  double currPt[3];
  for (unsigned int k = 0; k < input->GetNumberOfPoints(); ++k)
  {
    input->GetPoint(k, currPt);
    tempEigenVector << currPt[0], currPt[1], currPt[2];
    Points.push_back(tempEigenVector);
  }

  return;
}

//----------------------------------------------------------------------------
unsigned int ComputeNumberOfInlier(std::vector<Eigen::Matrix<double, 3, 1> >& Points, Eigen::Matrix<double, 3, 1> planePoint,
                                   Eigen::Matrix<double, 3, 1> planeNormal, double threshold)
{
  unsigned int nInliers = 0;
  for (unsigned int k = 0; k < Points.size(); ++k)
  {
    if (std::abs((Points[k] - planePoint).dot(planeNormal)) < threshold)
    {
      nInliers++;
    }
  }

  return nInliers;
}

// Implementation of the New function
vtkStandardNewMacro(vtkRansacPlaneModel);

//----------------------------------------------------------------------------
vtkRansacPlaneModel::vtkRansacPlaneModel()
{
  // 500 maximal ransac iteration
  this->MaxRansacIteration = 500;

  // 50 cm distance to plane threshold
  this->Threshold = 0.5;

  // 30% of inliers required
  this->RatioInliersRequired = 0.30;

  // fill params with 0 values
  std::fill(this->Param, this->Param + 4, 0);
}

//----------------------------------------------------------------------------
vtkRansacPlaneModel::~vtkRansacPlaneModel()
{
}

//-----------------------------------------------------------------------------
void vtkRansacPlaneModel::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
int vtkRansacPlaneModel::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get the input
  vtkPolyData * input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  // Get the output
  vtkPolyData *output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(input);

  // Convert the point cloud in Eigen data structure point cloud
  std::vector<Eigen::Matrix<double, 3, 1> > Points;
  ConvertPolyDataToEigenVector(Points, input);

  // Create a random order of points index
  std::vector<double> randomIndex(input->GetNumberOfPoints(), 0);
  for (unsigned int k = 0; k < randomIndex.size(); ++k)
  {
    randomIndex[k] = k;
  }
  std::random_shuffle(randomIndex.begin(), randomIndex.end());

  // information variable about ransac iterations
  bool shouldStopRansac = false;
  unsigned int iterationMade = 0;
  unsigned int iterationPointer = 0;

  // keep information of iterations
  std::vector<RansacSampleInfo> samplesInfo;
  unsigned int maxInliers = 0;
  unsigned int indexMaxInliers = 0;

  // Affine plane parameters
  Eigen::Matrix<double, 3, 1> planeNormal, planePoint;

  // indicate if ransac has "converged"
  bool hasConverged = false;

  // Ransac loop
  while (!shouldStopRansac)
  {
    // if we went throught the all random index we need
    // to random shuffle again
    if ((iterationPointer + 2) >= randomIndex.size())
    {
      std::random_shuffle(randomIndex.begin(), randomIndex.end());
      iterationPointer = 0;
    }

    // Compute current sample plane parameters
    planePoint = Points[randomIndex[iterationPointer]];
    planeNormal = (Points[randomIndex[iterationPointer + 2]] - planePoint).cross(Points[randomIndex[iterationPointer + 1]] - planePoint);
    planeNormal.normalize();

    // Compute the number of inliers / outliers
    unsigned int nInliers = ComputeNumberOfInlier(Points, planePoint, planeNormal, this->Threshold);

    // keep info
    RansacSampleInfo info(nInliers, randomIndex[iterationPointer], randomIndex[iterationPointer + 1], randomIndex[iterationPointer + 2]);
    samplesInfo.push_back(info);

    if (nInliers > maxInliers)
    {
      maxInliers = nInliers;
      indexMaxInliers = iterationMade;
    }

    // Check that the number of inliers is enought to
    // break the ransac algorithm loop
    if (nInliers > input->GetNumberOfPoints() * this->RatioInliersRequired)
    {
      shouldStopRansac = true;
      hasConverged = true;
    }

    // check if the maximum iteration has been reached
    if (iterationMade > this->MaxRansacIteration)
    {
      shouldStopRansac = true;
    }

    // Updates iterations index
    iterationMade++;
    iterationPointer += 3; 
  }

  // Now refine using all inliers
  RefineRansac(Points, output, samplesInfo[indexMaxInliers], this->Threshold, this->Param);

  // output info
  std::cout << "ransac algorithm has converged: " << hasConverged << std::endl;
  std::cout << "number of iteration made: " << iterationMade << std::endl;
  std::cout << "number of inliers: " << maxInliers << ", " << samplesInfo[indexMaxInliers].NInliers << std::endl;
  std::cout << "plane params: [" << this->Param[0] << "," << this->Param[1] << "," << this->Param[2] << "," << this->Param[3] << std::endl;

  return 1;
}

//-----------------------------------------------------------------------------
void vtkRansacPlaneModel::SetMaximumIteration(unsigned int maxIt)
{
  this->MaxRansacIteration = maxIt;
}

//-----------------------------------------------------------------------------
void vtkRansacPlaneModel::SetThreshold(double thresh)
{
  this->Threshold = thresh;
}

//-----------------------------------------------------------------------------
void vtkRansacPlaneModel::SetRatioInlierRequired(double ratio)
{
  this->RatioInliersRequired = ratio;
}

//-----------------------------------------------------------------------------
void vtkRansacPlaneModel::GetPlaneParam(double param[4])
{
  std::copy(this->Param, this->Param + 4, param);
  return;
}