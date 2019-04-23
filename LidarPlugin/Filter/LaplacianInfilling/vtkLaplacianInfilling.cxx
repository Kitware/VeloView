/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLaplacianInfilling.cxx
  Author: Pierre Guilbert

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// LOCAL
#include "vtkLaplacianInfilling.h"

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

// VTK
#include <vtkObjectFactory.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkXMLImageDataWriter.h>

// BOOST
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Sparse>

// Implementation of the New function
vtkStandardNewMacro(vtkLaplacianInfilling)

//-----------------------------------------------------------------------------
int vtkLaplacianInfilling::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  // Get the input
  vtkImageData * inputImage = vtkImageData::GetData(inputVector[0]->GetInformationObject(0));

  // Get the output
  vtkImageData* outputImage = vtkImageData::GetData(outputVector->GetInformationObject(0));
  outputImage->ShallowCopy(inputImage);

  int xBound = outputImage->GetDimensions()[0];
  int yBound = outputImage->GetDimensions()[1];
  int nParams = xBound * yBound;

  Eigen::SparseMatrix<double> Laplacian(nParams, nParams);
  Eigen::VectorXd Y(nParams); // The values of the laplacian required

  // Triplet of value: row, column and value
  std::vector<Eigen::Triplet<double> > nonZeroCoefficient;
  for (int x = 0; x < xBound; ++x)
  {
    for (int y = 0; y < yBound; ++y)
    {
      int flattenIndex = x + xBound * y;

      // check if the current pixel has a value
      double value = inputImage->GetScalarComponentAsDouble(x, y, 0, 0);
      if ((std::abs(value) > std::numeric_limits<double>::epsilon()))
      {
        // we don't want this value to be modified
        // contraint: xi = yi
        nonZeroCoefficient.push_back(Eigen::Triplet<double>(flattenIndex, flattenIndex, 1.0));
        Y(flattenIndex) = value;
      }
      else
      {
        // else fill it solving the laplace equation
        // using finite difference scheme and Dirichlet
        // boundary
        int validNeigh = 0;

        // Laplacian constraints matrix:
        // Neighbors contraint
        if (x != 0)
        {
          nonZeroCoefficient.push_back(Eigen::Triplet<double>(flattenIndex, flattenIndex - 1, 1));
          validNeigh++;
        }
        if (x != xBound - 1)
        {
          nonZeroCoefficient.push_back(Eigen::Triplet<double>(flattenIndex, flattenIndex + 1, 1));
          validNeigh++;
        }
        if (y != 0)
        {
          nonZeroCoefficient.push_back(Eigen::Triplet<double>(flattenIndex, flattenIndex - xBound, 1));
          validNeigh++;
        }
        if (y != yBound - 1)
        {
          nonZeroCoefficient.push_back(Eigen::Triplet<double>(flattenIndex, flattenIndex + xBound, 1));
          validNeigh++;
        }
        // Diagonal constraint
        nonZeroCoefficient.push_back(Eigen::Triplet<double>(flattenIndex, flattenIndex, -1.0 * static_cast<double>(validNeigh)));
        // We want the laplacian to be null
        Y(flattenIndex) = 0;
      }
    }
  }

  // Fill Laplacian constraints matrix
  Laplacian.setFromTriplets(nonZeroCoefficient.begin(), nonZeroCoefficient.end());

  // Solving:
  Eigen::SparseLU< Eigen::SparseMatrix<double> > solver(Laplacian);
  Eigen::MatrixXd X = solver.solve(Y);

  // X contains the Dirichlet solution function
  // values i.e: 0-values pixel are filled with
  // laplacian
  for (int x = 0; x < xBound; ++x)
  {
    for (int y = 0; y < yBound; ++y)
    {
      int flattendIndex = x + xBound * y;
      outputImage->SetScalarComponentFromDouble(x, y, 0, 0, X(flattendIndex));
    }
  }

  return 1;
}
