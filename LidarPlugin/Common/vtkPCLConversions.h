//=========================================================================
//
// Copyright 2012,2013,2014 Kitware, Inc.
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
//
// .NAME vtkPCLConversions - collection of pointcloud library routines
//
// .SECTION Description
//

#ifndef __vtkPCLConversions_h
#define __vtkPCLConversions_h

#include <vtkObject.h>
#include <vtkSmartPointer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

class vtkPolyData;
class vtkCellArray;
class vtkIntArray;

class VTK_EXPORT vtkPCLConversions : public vtkObject
{
public:

  static vtkPCLConversions* New();

  vtkTypeMacro(vtkPCLConversions, vtkObject);

  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkSmartPointer<vtkPolyData> PolyDataFromPCDFile(const std::string& filename);

  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(
    pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud);

  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);

  static pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFromPolyData(
    vtkPolyData* polyData);

  static vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);

  static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length);
  static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length);
  static vtkSmartPointer<vtkIntArray> NewLabelsArray(const std::vector<pcl::PointIndices>& indices, vtkIdType length);

  static void PerformPointCloudConversionBenchmark(vtkPolyData* polyData);

protected:

  vtkPCLConversions();
  ~vtkPCLConversions();

private:

  vtkPCLConversions(const vtkPCLConversions&); // Not implemented
  void operator=(const vtkPCLConversions&); // Not implemented
};

#endif
