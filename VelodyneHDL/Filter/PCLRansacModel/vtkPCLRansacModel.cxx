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
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// Implementation of the New function
vtkStandardNewMacro(vtkPCLRansacModel);

//----------------------------------------------------------------------------
vtkPCLRansacModel::vtkPCLRansacModel()
{

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

  // instantiate the model
  pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr RANSACModel;

  switch (ModelType) {

    case vtkPCLRansacModel::Circle2D:
      RANSACModel = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
            new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (pointCloud));
      break;

    case vtkPCLRansacModel::Circle3D:
      RANSACModel = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
            new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ> (pointCloud));
      break;

// The Cone Model and the Cylinder Model require to compute the normal, so this is not wrap
//    case vtkPCLRansacModel::Cone:
//      RANSACModel = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
//            new pcl::SampleConsensusModelCone<pcl::PointXYZ, pcl::Normal> (pointCloud));
//      break;

//    case vtkPCLRansacModel::Cylinder:
//      RANSACModel = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
//            new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> (pointCloud));
//      break;

    case vtkPCLRansacModel::Shpere:
      RANSACModel = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
            new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (pointCloud));
      break;

    case vtkPCLRansacModel::Line:
      RANSACModel = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
            new pcl::SampleConsensusModelLine<pcl::PointXYZ> (pointCloud));
      break;

    case vtkPCLRansacModel::Plane:
      RANSACModel = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
            new pcl::SampleConsensusModelLine<pcl::PointXYZ> (pointCloud));
      break;

    default:
      cerr << "No model : " << ModelType << endl;
      break;
  }

  // Instanciate and launch the random consensus
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (RANSACModel);
  ransac.setDistanceThreshold (this->DistanceThreshold);
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
