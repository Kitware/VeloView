//=========================================================================
//
// Copyright 2018 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
//         Laurenson Nick (nlaurenson5@gmail.com)
// Date: 03-27-2018
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
#ifndef SpinningSensorKeypointExtractor_H
#define SpinningSensorKeypointExtractor_H

#include <vector>
#include <unordered_map>

#include <pcl/point_cloud.h>

#include "LidarPoint.h"

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }


class SpinningSensorKeypointExtractor
{
public:
  using Point = PointXYZTIId;

  GetMacro(NeighborWidth, int)
  SetMacro(NeighborWidth, int)

  GetMacro(MinDistanceToSensor, double)
  SetMacro(MinDistanceToSensor, double)

  GetMacro(EdgeSinAngleThreshold, double)
  SetMacro(EdgeSinAngleThreshold, double)

  GetMacro(PlaneSinAngleThreshold, double)
  SetMacro(PlaneSinAngleThreshold, double)

  GetMacro(EdgeDepthGapThreshold, double)
  SetMacro(EdgeDepthGapThreshold, double)

  GetMacro(AngleResolution, double)
  SetMacro(AngleResolution, double)

  GetMacro(SaillancyThreshold, double)
  SetMacro(SaillancyThreshold, double)

  GetMacro(FarestKeypointDist, double)

  GetMacro(NLasers, int)

  pcl::PointCloud<Point>::Ptr GetEdgePoints() { return this->EdgesPoints; }
  pcl::PointCloud<Point>::Ptr GetPlanarPoints() { return this->PlanarsPoints; }
  pcl::PointCloud<Point>::Ptr GetBlobPoints() { return this->BlobsPoints; }

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature
  void ComputeKeyPoints(pcl::PointCloud<Point>::Ptr pc, std::vector<size_t> laserIdMapping);

  // Function to enable to have some inside on why a given point was detected as a keypoint
  std::unordered_map<std::string, std::vector<double>> GetDebugArray();
private:

  // Reset all mumbers variables that are
  // used during the process of a frame.
  void PrepareDataForNextFrame();

  // Convert the input vtk-format pointcloud
  // into a pcl-pointcloud format. scan lines
  // will also be sorted by their vertical angles
  void ConvertAndSortScanLines();

  // Compute the curvature of the scan lines
  // The curvature is not the one of the surface
  // that intersected the lines but the curvature
  // of the scan lines taken in an isolated way
  void ComputeCurvature();

  // Invalid the points with bad criteria from
  // the list of possible future keypoints.
  // This points correspond to planar surface
  // roughtly parallel to laser beam and points
  // close to a gap created by occlusion
  void InvalidPointWithBadCriteria();

  // Labelizes point to be a keypoints or not
  void SetKeyPointsLabels();

  // with of the neighbor used to compute discrete
  // differential operators
  int NeighborWidth = 4;

  // minimal point/sensor sensor to consider a point as valid
  double MinDistanceToSensor = 3.0;

  // Sharpness threshold to select a point
  double EdgeSinAngleThreshold = 0.86; // 60 degrees
  double PlaneSinAngleThreshold = 0.5; // 30 degrees
  double EdgeDepthGapThreshold = 0.15;
  double DistToLineThreshold = 0.20;

  // maximal angle resolution of the lidar
  // azimutal resolution of the VLP-16. We add an extra 20 %
  double AngleResolution = 0.00698; // 0.4 degree

  // Threshold upon sphricity of a neighborhood
  // to select a blob point
  double SphericityThreshold = 0.35;

  // Threshold upon saillancy of a neighborhood
  // to select an edge keypoint
  double SaillancyThreshold = 2.25;

  // Coef to apply to the incertitude
  // radius of the blob neighborhood
  double IncertitudeCoef = 3.0;

  // Mapping of the lasers id
  std::vector<size_t> LaserIdMapping;

  // Number of lasers scan lines composing the pointcloud
  unsigned int NLasers = 0;

  // norm of the farest keypoints
  double FarestKeypointDist = 0;

  // Curvature and over differntial operations
  // scan by scan; point by point
  std::vector<std::vector<double>> Angles;
  std::vector<std::vector<double>> DepthGap;
  std::vector<std::vector<double>> SaillantPoint;
  std::vector<std::vector<double>> IntensityGap;
  std::vector<std::vector<double>> IsPointValid;
  std::vector<std::vector<double>> Label;

  // Mapping between keypoints and their corresponding
  // index in the vtk input frame
  std::vector<std::pair<int, int>> EdgesIndex;
  std::vector<std::pair<int, int>> PlanarIndex;
  std::vector<std::pair<int, int>> BlobIndex;

  pcl::PointCloud<Point>::Ptr EdgesPoints;
  pcl::PointCloud<Point>::Ptr PlanarsPoints;
  pcl::PointCloud<Point>::Ptr BlobsPoints;

  // Current point cloud stored in two differents formats
  pcl::PointCloud<Point>::Ptr pclCurrentFrame;
  std::vector<pcl::PointCloud<Point>::Ptr> pclCurrentFrameByScan;
};

#endif // SpinningSensorKeypointExtractor_H
