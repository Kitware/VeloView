/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vvtkPCLRansacModel.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// local includes
#include "vtkPCLRansacModel.h"
#include "vtkPCLConversions.h"

// vtk includes
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDataSet.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>

// pcl includes
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// Implementation of the New function
vtkStandardNewMacro(vtkPCLRansacModel);

class vtkPCLRansacModel::vtkInternal
{
public:

  vtkInternal()
  {
    this->DistanceThreshold = 0.35; // 20cm
  }

  ~vtkInternal()
  {
  }

  // Threshold to determine if
  // a point is an inlier or outlier
  double DistanceThreshold;
};

//----------------------------------------------------------------------------
vtkPCLRansacModel::vtkPCLRansacModel()
{
  this->Internal = new vtkInternal;
}

//----------------------------------------------------------------------------
vtkPCLRansacModel::~vtkPCLRansacModel()
{
}

//-----------------------------------------------------------------------------
void vtkPCLRansacModel::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
int vtkPCLRansacModel::RequestData(vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{

  // Get the input
  vtkPolyData * input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));
  
  // Convert input data in pcl format
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = vtkPCLConversions::PointCloudFromPolyData(input);

  // inliers's index according to the model and threshold
  std::vector<int> inliers;

  // instanciate the model
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    modelPlane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pointCloud));

  // Instanciate and launch the random consensus
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (modelPlane);
  ransac.setDistanceThreshold (this->Internal->DistanceThreshold);
  ransac.computeModel(); // compute ransac
  ransac.getInliers(inliers); // get inlier list
  std::cout << "Inliers size : " << inliers.size() << std::endl;
  // Add inlier / outlier array information to vtkPolyData input
  vtkSmartPointer<vtkUnsignedIntArray> InlierOutlierArray = vtkSmartPointer<vtkUnsignedIntArray>::New();
  InlierOutlierArray->Allocate(input->GetNumberOfPoints());
  InlierOutlierArray->SetName("inliers");
  InlierOutlierArray->SetNumberOfTuples(0);

  // populate the new array
  for (int k = 0; k < input->GetNumberOfPoints(); ++k)
  {
    InlierOutlierArray->InsertNextValue(0);
  }
  for (int k = 0; k < inliers.size(); ++k)
  {
    InlierOutlierArray->SetValue(inliers[k], 255);
  }

  // Get the output
  vtkPolyData *output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(input);

  // Add the array
  output->GetPointData()->AddArray(InlierOutlierArray);

  return 1;
}