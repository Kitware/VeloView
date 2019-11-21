//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (pierre.guilbert@kitware.com)
// Date: 2019-07-03
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
//=========================================================================

// LOCAL
#include "MIDHOG.h"
#include "vtkPCLConversions.h"
#include "CameraCalibration.h"
// STD
#include <iostream>
// VTK
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkSmartPointer.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OPENCV
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  return 0;

  // load the image
  cv::Mat img;

  // load the pointcloud
  vtkNew<vtkXMLPolyDataReader> reader;
  reader->SetFileName("");
  reader->Update();
  vtkSmartPointer<vtkPolyData> vtkCloud = reader->GetOutput();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = vtkPCLConversions::PointCloudFromPolyDataWithIntensity(vtkCloud);

  // first estimation of K
  double fov = 90.0 / 180.0 * vtkMath::Pi();
  double f = static_cast<double>(img.cols) / (2.0 * std::tan(fov / 2.0));
  double x0 = static_cast<double>(img.rows / 2.0);
  double y0 = static_cast<double>(img.cols / 2.0);
  double skew = 0;

  // first guess, level-arm not too high
  double tx = 0;
  double ty = 0;
  double tz = 0;

  // Orientation
  double rx = -0.78666;
  double ry = 1.45414;
  double rz = -0.450068;

  // Initial parameters
  Eigen::Matrix<double, 17, 1> W;

  // Initial guess close to the solution
  W << rx, ry, rz, tx, ty, tz, f, -f, x0, y0, skew, 0, 0, 0, 0, 0, 0;

  // expected solution
  //W << 1.50171,0.0864216,1.88192,-0.0384615,0.464753,-0.124066,-862.402,871.102,949.188,577.101,-3.33209,-0.301486,0.0950394,-8.47634e-05,5.45807e-05,44.5193,-32.7421;

  // Instanciate
  MIDHOGCalibration automaticCalib(img, cloud, W, 4);
  automaticCalib.CreateSyntheticImage();

  return 0;
}
