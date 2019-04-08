//=========================================================================
//
// Copyright 2018 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
//         Laurenson Nick (nlaurenson5@gmail.com)
// Data: 03-27-2018
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
#include "vtkRotatingKeyPointsExtractor.h"

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkTable.h>

namespace {
//-----------------------------------------------------------------------------
template <typename T>
std::vector<size_t> sortIdx(const std::vector<T> &v)
{
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

  return idx;
}

//-----------------------------------------------------------------------------
class LineFitting
{
public:
  // Fitting using PCA
  bool FitPCA(std::vector<Eigen::Vector3d >& points);

  // Futting using very local line and
  // check if this local line is consistent
  // in a more global neighborhood
  bool FitPCAAndCheckConsistency(std::vector<Eigen::Vector3d >& points);

  // Poor but fast fitting using
  // extremities of the distribution
  void FitFast(std::vector<Eigen::Vector3d >& points);

  // Direction and position
  Eigen::Vector3d Direction;
  Eigen::Vector3d Position;
  Eigen::Matrix3d SemiDist;
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  double MaxDistance = 0.02;
  double MaxSinAngle = 0.65;
};

//-----------------------------------------------------------------------------
bool LineFitting::FitPCA(std::vector<Eigen::Vector3d >& points)
{
  // Compute PCA to determine best line approximation
  // of the points distribution
  Eigen::MatrixXd data(points.size(), 3);

  for (unsigned int k = 0; k < points.size(); k++)
  {
    data.row(k) = points[k];
  }

  Eigen::Vector3d mean = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - mean.transpose();
  Eigen::MatrixXd cov = centered.transpose() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);

  // Eigen values
  Eigen::MatrixXd D(1,3);
  // Eigen vectors
  Eigen::MatrixXd V(3,3);

  D = eig.eigenvalues();
  V = eig.eigenvectors();

  // Direction
  this->Direction = V.col(2).normalized();

  // Position
  this->Position = mean;

  // Semi distance matrix
  // (polar form associated to
  // a bilineare symmetric positive
  // semi-definite matrix)
  this->SemiDist = (this->I3 - this->Direction * this->Direction.transpose());
  this->SemiDist = this->SemiDist.transpose() * this->SemiDist;

  bool isLineFittingAccurate = true;

  // if a point of the neighborhood is too far from
  // the fitting line we considere the neighborhood as
  // non flat
  for (unsigned int k = 0; k < points.size(); k++)
  {
    double d = std::sqrt((points[k] - this->Position).transpose() * this->SemiDist * (points[k] - this->Position));
    if (d > this->MaxDistance)
    {
      isLineFittingAccurate = false;
    }
  }

  return isLineFittingAccurate;
}

//-----------------------------------------------------------------------------
bool LineFitting::FitPCAAndCheckConsistency(std::vector<Eigen::Vector3d >& points)
{
  bool isLineFittingAccurate = true;

  // first check if the neighborhood is straight
  Eigen::Vector3d U, V;
  U = (points[1] - points[0]).normalized();
  for (unsigned int index = 1; index < points.size() - 1; index++)
  {
    V = (points[index + 1] - points[index]).normalized();
    double sinAngle = (U.cross(V)).norm();
    if (sinAngle > this->MaxSinAngle)
    {
      isLineFittingAccurate = false;
    }
  }

  // Then fit with PCA
  isLineFittingAccurate &= this->FitPCA(points);
  return isLineFittingAccurate;
}

//-----------------------------------------------------------------------------
void LineFitting::FitFast(std::vector<Eigen::Vector3d >& points)
{
  // Take the two extrems points of the neighborhood
  // i.e the farest and the closest to the current point
  Eigen::Vector3d U = points[0];
  Eigen::Vector3d V = points[points.size() - 1];

  // direction
  this->Direction = (V - U).normalized();

  // position
  this->Position = U;

  // Semi distance matrix
  // (polar form associated to
  // a bilineare symmetric positive
  // semi-definite matrix)
  this->SemiDist = (this->I3 - this->Direction * this->Direction.transpose());
  this->SemiDist = this->SemiDist.transpose() * this->SemiDist;
}
}

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkRotatingKeyPointsExtractor)

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::PrepareDataForNextFrame()
{
  // Reset the pcl format pointcloud to store the new frame
  this->pclCurrentFrame.reset(new pcl::PointCloud<Point>());
  this->pclCurrentFrameByScan.resize(this->NLasers);
  for (unsigned int k = 0; k < this->NLasers; ++k)
  {
    this->pclCurrentFrameByScan[k].reset(new pcl::PointCloud<Point>());
  }

  this->EdgesPoints.reset(new pcl::PointCloud<Point>());
  this->PlanarsPoints.reset(new pcl::PointCloud<Point>());
  this->BlobsPoints.reset(new pcl::PointCloud<Point>());

  // reset vtk <-> pcl id mapping
  this->FromVTKtoPCLMapping.clear();
  this->FromVTKtoPCLMapping.resize(0);
  this->FromPCLtoVTKMapping.clear();
  this->FromPCLtoVTKMapping.resize(this->NLasers);
  this->Angles.clear();
  this->Angles.resize(this->NLasers);
  this->LengthResolution.clear();
  this->LengthResolution.resize(this->NLasers);
  this->SaillantPoint.clear();
  this->SaillantPoint.resize(this->NLasers);
  this->DepthGap.clear();
  this->DepthGap.resize(this->NLasers);
  this->IntensityGap.clear();
  this->IntensityGap.resize(this->NLasers);
  this->BlobScore.clear();
  this->BlobScore.resize(this->NLasers);
  this->IsPointValid.clear();
  this->IsPointValid.resize(this->NLasers);
  this->Label.clear();
  this->Label.resize(this->NLasers);
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::ConvertAndSortScanLines()
{
  // temp var
  double xL[3]; // in {L}
  Point yL; // in {L}

  // Get informations about input pointcloud
  vtkDataArray* lasersId = this->Frame->GetPointData()->GetArray("laser_id");
  vtkDataArray* time = this->Frame->GetPointData()->GetArray("timestamp");
  vtkDataArray* reflectivity = this->Frame->GetPointData()->GetArray("intensity");
  vtkPoints* Points = this->Frame->GetPoints();
  unsigned int Npts = this->Frame->GetNumberOfPoints();
  double t0 = static_cast<double>(time->GetTuple1(0));
  double t1 = static_cast<double>(time->GetTuple1(Npts - 1));
  this->FromVTKtoPCLMapping.resize(Npts);


  for (unsigned int index = 0; index < Npts; ++index)
  {
    // Get information about current point
    Points->GetPoint(index, xL);
    yL.x = xL[0]; yL.y = xL[1]; yL.z = xL[2];

    double relAdv = (static_cast<double>(time->GetTuple1(index)) - t0) / (t1 - t0);
    unsigned int id = static_cast<int>(lasersId->GetTuple1(index));
    double reflec = static_cast<double>(reflectivity->GetTuple1(index));
    id = this->LaserIdMapping[id];
    yL.intensity = relAdv;
    yL.normal_y = id;
    yL.normal_z = reflec;

    // add the current point to its corresponding laser scan
    this->pclCurrentFrame->push_back(yL);
    this->pclCurrentFrameByScan[id]->push_back(yL);
    this->FromVTKtoPCLMapping[index] = std::pair<int, int>(id, this->pclCurrentFrameByScan[id]->size() - 1);
    this->FromPCLtoVTKMapping[id].push_back(index);
  }
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::UpdateLaserIdMapping(vtkTable *calib)
{
  this->NLasers = calib->GetNumberOfRows();
  auto array = vtkDataArray::SafeDownCast(calib->GetColumnByName("verticalCorrection"));
  if (array)
  {
    std::vector<double> verticalCorrection;
    verticalCorrection.resize(array->GetNumberOfTuples());
    for (int i =0; i < array->GetNumberOfTuples(); ++i)
    {
      verticalCorrection[i] = array->GetTuple1(i);
    }
    this->LaserIdMapping = sortIdx(verticalCorrection);
  }
  else
  {
    vtkErrorMacro("<< The calibration data has no colomn named 'verticalCorrection'");
  }
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::ComputeKeyPoints(vtkPolyData* input, vtkTable* calib)
{
  this->Frame = input;
  if (this->LaserIdMapping.empty())
  {
    this->UpdateLaserIdMapping(calib);
  }
  this->PrepareDataForNextFrame();
  this->ConvertAndSortScanLines();
  // Initialize the vectors with the correct length
  for (unsigned int k = 0; k < this->NLasers; ++k)
  {
    this->IsPointValid[k].resize(this->pclCurrentFrameByScan[k]->size(), 1);
    this->Label[k].resize(this->pclCurrentFrameByScan[k]->size(), 0);
    this->Angles[k].resize(this->pclCurrentFrameByScan[k]->size(),0);
    this->LengthResolution[k].resize(this->pclCurrentFrameByScan[k]->size(),0);
    this->SaillantPoint[k].resize(this->pclCurrentFrameByScan[k]->size(),0);
    this->DepthGap[k].resize(this->pclCurrentFrameByScan[k]->size(), 0);
    this->IntensityGap[k].resize(this->pclCurrentFrameByScan[k]->size(), 0);
    this->BlobScore[k].resize(this->pclCurrentFrameByScan[k]->size(), 0);
  }

  // compute keypoints scores
  this->ComputeCurvature();

  // Invalid points with bad criteria
  this->InvalidPointWithBadCriteria();

  // labelize keypoints
  this->SetKeyPointsLabels();
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::AddDisplayInformation(vtkSmartPointer<vtkPolyData> input)
{
  this->DisplayLaserIdMapping(input);
  this->DisplayRelAdv(input);
  AddVectorToPolydataPoints<double, vtkDoubleArray>(this->Angles, "angles_line", input);
  AddVectorToPolydataPoints<double, vtkDoubleArray>(this->LengthResolution, "length_resolution", input);
  AddVectorToPolydataPoints<double, vtkDoubleArray>(this->SaillantPoint, "saillant_point", input);
  AddVectorToPolydataPoints<double, vtkDoubleArray>(this->DepthGap, "depth_gap", input);
  AddVectorToPolydataPoints<double, vtkDoubleArray>(this->IntensityGap, "intensity_gap", input);
  AddVectorToPolydataPoints<double, vtkDoubleArray>(this->BlobScore, "blob_score", input);
  AddVectorToPolydataPoints<int, vtkIntArray>(this->IsPointValid, "is_point_valid", input);
  AddVectorToPolydataPoints<int, vtkIntArray>(this->Label, "keypoint_label", input);
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::ComputeCurvature()
{
  Point currentPoint, nextPoint, previousPoint;
  Eigen::Vector3d X, centralPoint;
  LineFitting leftLine, rightLine, farNeighborsLine;

  // loop over scans lines
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    // loop over points in the current scan line
    int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (Npts < 2 * this->NeighborWidth + 1)
    {
      continue;
    }

    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // central point
      currentPoint = this->pclCurrentFrameByScan[scanLine]->points[index];
      centralPoint << currentPoint.x, currentPoint.y, currentPoint.z;

      // compute intensity gap
      nextPoint = this->pclCurrentFrameByScan[scanLine]->points[index + 1];
      previousPoint = this->pclCurrentFrameByScan[scanLine]->points[index - 1];
      this->IntensityGap[scanLine][index] = std::abs(nextPoint.normal_z - previousPoint.normal_z);
      // We will compute the line that fit the neighbors located
      // previously the current. We will do the same for the
      // neighbors located after the current points. We will then
      // compute the angle between these two lines as an approximation
      // of the "sharpness" of the current point.
      std::vector<Eigen::Vector3d > leftNeighbor;
      std::vector<Eigen::Vector3d > rightNeighbor;
      std::vector<Eigen::Vector3d > farNeighbors;

      // Fill right and left neighborhood
      // /!\ The way the neighbors are added
      // to the vectors matters. Especially when
      // computing the saillancy
      for (int j = index - this->NeighborWidth; j <= index + this->NeighborWidth; ++j)
      {
        currentPoint = this->pclCurrentFrameByScan[scanLine]->points[j];
        X << currentPoint.x, currentPoint.y, currentPoint.z;
        if (j < index)
          leftNeighbor.push_back(X);
        if (j > index)
          rightNeighbor.push_back(X);
      }

      // Fit line on the neighborhood and
      // Indicate if the left and right side
      // neighborhood of the current point is flat or not
      bool leftFlat = leftLine.FitPCAAndCheckConsistency(leftNeighbor);
      bool rightFlat = rightLine.FitPCAAndCheckConsistency(rightNeighbor);

      // Measurement of the gap
      double dist1 = 0; double dist2 = 0;

      // if both neighborhood are flat we can compute
      // the angle between them as an approximation of the
      // sharpness of the current point
      if (rightFlat && leftFlat)
      {
        // We check that the current point is not too far from its
        // neighborhood lines. This is because we don't want a point
        // to be considered as a angles point if it is due to gap
        dist1 = std::sqrt((centralPoint - leftLine.Position).transpose() * leftLine.SemiDist * (centralPoint - leftLine.Position));
        dist2 = std::sqrt((centralPoint - rightLine.Position).transpose() * rightLine.SemiDist * (centralPoint - rightLine.Position));

        if ((dist1 < this->DistToLineThreshold) && (dist2 < this->DistToLineThreshold))
          this->Angles[scanLine][index] = std::abs((leftLine.Direction.cross(rightLine.Direction)).norm()); // sin of angle actually
      }
      // Here one side of the neighborhood is non flat
      // Hence it is not worth to estimate the sharpness.
      // Only the gap will be considered here.
      else if (rightFlat && !leftFlat)
      {
        dist1 = 1000.0;
        for (unsigned int neighIndex = 0; neighIndex < leftNeighbor.size(); ++neighIndex)
        {
          dist1 = std::min(dist1,
                  std::sqrt((leftNeighbor[neighIndex] - rightLine.Position).transpose() * rightLine.SemiDist * (leftNeighbor[neighIndex] - rightLine.Position)));
        }
        dist1 = 0.5 * dist1;
      }
      else if (!rightFlat && leftFlat)
      {
        dist2 = 1000.0;
        for (unsigned int neighIndex = 0; neighIndex < leftNeighbor.size(); ++neighIndex)
        {
          dist2 = std::min(dist2,
                  std::sqrt((rightNeighbor[neighIndex] - leftLine.Position).transpose() * leftLine.SemiDist * (rightNeighbor[neighIndex] - leftLine.Position)));
        }
        dist2 = 0.5 * dist2;
      }
      else
      {
        // Compute saillant point score
        double currDepth = centralPoint.norm();
        unsigned int diffDepth = 0;
        bool canLeftBeAdded = true; bool hasLeftEncounteredDepthGap = false;
        bool canRightBeAdded = true; bool hasRightEncounteredDepthGap = false;

        // The saillant point score is the distance between the current point
        // and the points that have a depth gap with the current point
        for (unsigned int neighIndex = 0; neighIndex < leftNeighbor.size(); ++neighIndex)
        {
          // Left neighborhood depth gap computation
          if ((std::abs(leftNeighbor[leftNeighbor.size() - 1 - neighIndex].norm() - currDepth) > 1.5) && canLeftBeAdded)
          {
            hasLeftEncounteredDepthGap = true;
            diffDepth++;
            farNeighbors.push_back(leftNeighbor[neighIndex]);
          }
          else
          {
            if (hasLeftEncounteredDepthGap)
            {
              canLeftBeAdded = false;
            }
          }
          // Right neigborhood depth gap computation
          if ((std::abs(rightNeighbor[neighIndex].norm() - currDepth) > 1.5) && canRightBeAdded)
          {
            hasRightEncounteredDepthGap = true;
            diffDepth++;
            farNeighbors.push_back(rightNeighbor[neighIndex]);
          }
          else
          {
            if (hasRightEncounteredDepthGap)
            {
              canRightBeAdded = false;
            }
          }
        }

        // If there is enought neighbors with a big depth gap
        // we propose to compute the saillancy of the current
        // as the distance between the line that fits the neighbors
        // with a depth gap and the current point
        if (static_cast<double>(diffDepth) / (2.0 * this->NeighborWidth) > 0.5)
        {
          farNeighborsLine.FitPCA(farNeighbors);
          this->SaillantPoint[scanLine][index] = std::sqrt(
            (centralPoint - farNeighborsLine.Position).transpose() * farNeighborsLine.SemiDist * (centralPoint - farNeighborsLine.Position));
        }

        this->BlobScore[scanLine][index] = 1;
      }

      this->DepthGap[scanLine][index] = std::max(dist1, dist2);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::InvalidPointWithBadCriteria()
{
  // Temporary variables used in the next loop
  Eigen::Vector3d dX, X, Xn, Xp, Xproj, dXproj;
  Eigen::Vector3d Y, Yn, Yp, dY;
  double L, Ln, expectedLength, dLn, dLp;
  Point currentPoint, nextPoint, previousPoint;
  Point temp;

  // loop over scan lines
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (Npts < 3 * this->NeighborWidth)
    {
      continue;
    }
    // invalidate first and last points
    for (int index = 0; index <= this->NeighborWidth; ++index)
    {
      this->IsPointValid[scanLine][index] = 0;
    }
    for (int index = Npts - 1 - this->NeighborWidth - 1; index < Npts; ++index)
    {
      this->IsPointValid[scanLine][index] = 0;
    }

    // loop over points into the scan line
    for (int index = this->NeighborWidth; index <  Npts - this->NeighborWidth - 1; ++index)
    {
      currentPoint = this->pclCurrentFrameByScan[scanLine]->points[index];
      nextPoint = this->pclCurrentFrameByScan[scanLine]->points[index + 1];
      previousPoint = this->pclCurrentFrameByScan[scanLine]->points[index - 1];
      X << currentPoint.x, currentPoint.y, currentPoint.z;
      Xn << nextPoint.x, nextPoint.y, nextPoint.z;
      Xp << previousPoint.x, previousPoint.y, previousPoint.z;
      dX = Xn - X;
      L = X.norm();
      Ln = Xn.norm();
      dLn = dX.norm();

      // the expected length between two firing of the same laser
      // depend on the distance and the angular resolution of the
      // sensor.
      expectedLength = 2.0 *  std::tan(this->AngleResolution / 2.0) * L;
      double ratioExpectedLength = 10.0;

      // if the length between the two firing
      // is more than n-th the expected length
      // it means that there is a gap. We now must
      // determine if the gap is due to the geometry of
      // the scene or if the gap is due to an occluded area
      if (dLn > ratioExpectedLength * expectedLength)
      {
        // Project the next point onto the
        // sphere of center 0 and radius =
        // norm of the current point. If the
        // gap has disappeared it means that
        // the gap was due to an occlusion
        Xproj = L / Ln * Xn;
        dXproj = Xproj - X;
        // it is a depth gap, invalidate the part which belong
        // to the occluded area (farest)
        // invalid next part
        if (L < Ln)
        {
          for (int i = index + 1; i <= index + this->NeighborWidth; ++i)
          {
            if (i > index + 1)
            {
              temp = this->pclCurrentFrameByScan[scanLine]->points[i - 1];
              Yp << temp.x, temp.y, temp.z;
              temp = this->pclCurrentFrameByScan[scanLine]->points[i];
              Y << temp.x, temp.y, temp.z;
              dY = Y - Yp;
              // if there is a gap in the neihborhood
              // we do not invalidate the rest of neihborhood
              if (dY.norm() > ratioExpectedLength * expectedLength)
              {
                break;
              }
            }
            this->IsPointValid[scanLine][i] = 0;
          }
        }
        // invalid previous part
        else
        {
          for (int i = index - this->NeighborWidth; i <= index; ++i)
          {
            if (i < index)
            {
              temp = this->pclCurrentFrameByScan[scanLine]->points[i + 1];
              Yn << temp.x, temp.y, temp.z;
              temp = this->pclCurrentFrameByScan[scanLine]->points[i];
              Y << temp.x, temp.y, temp.z;
              dY = Yn - Y;
              // if there is a gap in the neihborhood
              // we do not invalidate the rest of neihborhood
              if (dY.norm() > ratioExpectedLength * expectedLength)
              {
                break;
              }
            }
            this->IsPointValid[scanLine][i] = 0;
          }
        }
      }
      // Invalid points which are too close from the sensor
      if (L < this->MinDistanceToSensor)
      {
        this->IsPointValid[scanLine][index] = 0;
      }

      // Invalid points which are on a planar
      // surface nearly parallel to the laser
      // beam direction
      dLp = (X - Xp).norm();
      if ((dLp > 1 / 4.0 * ratioExpectedLength * expectedLength) && (dLn > 1 / 4.0 * ratioExpectedLength * expectedLength))
      {
        this->IsPointValid[scanLine][index] = 0;
      }
    }
  }
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::SetKeyPointsLabels()
{
  this->EdgesIndex.clear(); this->EdgesIndex.resize(0);
  this->PlanarIndex.clear(); this->PlanarIndex.resize(0);
  this->BlobIndex.clear(); this->BlobIndex.resize(0);

  // loop over the scan lines
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    int Npts = this->pclCurrentFrameByScan[scanLine]->size();
    unsigned int nbrEdgePicked = 0;
    unsigned int nbrPlanarPicked = 0;

    // We split the validity of points between the edges
    // keypoints and planar keypoints. This allows to take
    // some points as planar keypoints even if they are close
    // to an edge keypoint.
    std::vector<int> IsPointValidForPlanar = this->IsPointValid[scanLine];

    // if the line is almost empty, skip it
    if (Npts < 3 * this->NeighborWidth)
    {
      continue;
    }

    // Sort the curvature score in a decreasing order
    std::vector<size_t> sortedDepthGapIdx = sortIdx<double>(this->DepthGap[scanLine]);
    std::vector<size_t> sortedAnglesIdx = sortIdx<double>(this->Angles[scanLine]);
    std::vector<size_t> sortedSaillancyIdx = sortIdx<double>(this->SaillantPoint[scanLine]);
    std::vector<size_t> sortedIntensityGap = sortIdx<double>(this->IntensityGap[scanLine]);

    double depthGap, sinAngle, saillancy, intensity;
    int index = 0;

    // Edges using depth gap
    for (int k = 0; k < Npts; ++k)
    {
      index = sortedDepthGapIdx[k];
      depthGap = this->DepthGap[scanLine][index];

      // thresh
      if (depthGap < this->EdgeDepthGapThreshold)
      {
        break;
      }

      // if the point is invalid continue
      if (this->IsPointValid[scanLine][index] == 0)
      {
        continue;
      }

      // else indicate that the point is an edge
      this->Label[scanLine][index] = 4;
      this->EdgesIndex.push_back(std::pair<int, int>(scanLine, index));
      nbrEdgePicked++;
      //IsPointValidForPlanar[index] = 0;

      // invalid its neighborhod
      int indexBegin = index - this->NeighborWidth + 1;
      int indexEnd = index + this->NeighborWidth - 1;
      indexBegin = std::max(0, indexBegin);
      indexEnd = std::min(Npts - 1, indexEnd);
      for (int j = indexBegin; j <= indexEnd; ++j)
      {
        this->IsPointValid[scanLine][j] = 0;
      }
    }

    // Edges using angles
    for (int k = 0; k < Npts; ++k)
    {
      index = sortedAnglesIdx[k];
      sinAngle = this->Angles[scanLine][index];

      // thresh
      if (sinAngle < this->EdgeSinAngleThreshold)
      {
        break;
      }

      // if the point is invalid continue
      if (this->IsPointValid[scanLine][index] == 0)
      {
        continue;
      }

      // else indicate that the point is an edge
      this->Label[scanLine][index] = 4;
      this->EdgesIndex.push_back(std::pair<int, int>(scanLine, index));
      nbrEdgePicked++;
      //IsPointValidForPlanar[index] = 0;

      // invalid its neighborhod
      int indexBegin = index - this->NeighborWidth;
      int indexEnd = index + this->NeighborWidth;
      indexBegin = std::max(0, indexBegin);
      indexEnd = std::min(Npts - 1, indexEnd);
      for (int j = indexBegin; j <= indexEnd; ++j)
      {
        this->IsPointValid[scanLine][j] = 0;
      }
    }

    // Edges using saillancy
    for (int k = 0; k < Npts; ++k)
    {
      index = sortedSaillancyIdx[k];
      saillancy = this->SaillantPoint[scanLine][index];

      // thresh
      if (saillancy < 1.5)
      {
        break;
      }

      // if the point is invalid continue
      if (this->IsPointValid[scanLine][index] == 0)
      {
        continue;
      }

      // else indicate that the point is an edge
      this->Label[scanLine][index] = 4;
      this->EdgesIndex.push_back(std::pair<int, int>(scanLine, index));
      nbrEdgePicked++;
      //IsPointValidForPlanar[index] = 0;

      // invalid its neighborhod
      int indexBegin = index - this->NeighborWidth + 1;
      int indexEnd = index + this->NeighborWidth - 1;
      indexBegin = std::max(0, indexBegin);
      indexEnd = std::min(Npts - 1, indexEnd);
      for (int j = indexBegin; j <= indexEnd; ++j)
      {
        this->IsPointValid[scanLine][j] = 0;
      }
    }

    // Edges using intensity
    for (int k = 0; k < Npts; ++k)
    {
      index = sortedIntensityGap[k];
      intensity = this->IntensityGap[scanLine][index];

      // thresh
      if (intensity < 50.0)
      {
        break;
      }

      // if the point is invalid continue
      if (this->IsPointValid[scanLine][index] == 0)
      {
        continue;
      }

      // else indicate that the point is an edge
      this->Label[scanLine][index] = 4;
      this->EdgesIndex.push_back(std::pair<int, int>(scanLine, index));
      nbrEdgePicked++;
      //IsPointValidForPlanar[index] = 0;

      // invalid its neighborhood
      int indexBegin = index - 1;
      int indexEnd = index + 1;
      indexBegin = std::max(0, indexBegin);
      indexEnd = std::min(Npts - 1, indexEnd);
      for (int j = indexBegin; j <= indexEnd; ++j)
      {
        this->IsPointValid[scanLine][j] = 0;
      }
    }

    // Blobs Points
//    if (!this->FastSlam)
//    {
      for (int k = 0; k < Npts; k = k + 3)
      {
        this->BlobIndex.push_back(std::pair<int, int>(scanLine, k));
      }
//    }

    // Planes
    for (int k = Npts - 1; k >= 0; --k)
    {
      index = sortedAnglesIdx[k];
      sinAngle = this->Angles[scanLine][index];

      // thresh
      if (sinAngle > this->PlaneSinAngleThreshold)
      {
        break;
      }

      // if the point is invalid continue
      if (IsPointValidForPlanar[index] == 0)
      {
        continue;
      }

      // else indicate that the point is a planar one
      if ((this->Label[scanLine][index] != 4) && (this->Label[scanLine][index] != 3))
        this->Label[scanLine][index] = 2;
      this->PlanarIndex.push_back(std::pair<int, int>(scanLine, index));
      IsPointValidForPlanar[index] = 0;
      this->IsPointValid[scanLine][index] = 0;

      // Invalid its neighbor so that we don't have too
      // many planar keypoints in the same region. This is
      // required because of the k-nearest search + plane
      // approximation realized in the odometry part. Indeed,
      // if all the planar points are on the same scan line the
      // problem is degenerated since all the points are distributed
      // on a line.
      int indexBegin = index - 4;
      int indexEnd = index + 4;
      indexBegin = std::max(0, indexBegin);
      indexEnd = std::min(Npts - 1, indexEnd);
      for (int j = indexBegin; j <= indexEnd; ++j)
      {
        IsPointValidForPlanar[j] = 0;
      }
      nbrPlanarPicked++;
    }
  }

  // add keypoints in increasing scan id order
  std::sort(this->EdgesIndex.begin(), this->EdgesIndex.end());
  std::sort(this->PlanarIndex.begin(), this->PlanarIndex.end());
  std::sort(this->BlobIndex.begin(), this->BlobIndex.end());

  // fill the keypoints vectors and compute the max dist keypoints
  this->FarestKeypointDist = 0.0;
  Point p;
  for (unsigned int k = 0; k < this->EdgesIndex.size(); ++k)
  {
    p = this->pclCurrentFrameByScan[this->EdgesIndex[k].first]->points[this->EdgesIndex[k].second];
    this->EdgesPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2))));
  }
  for (unsigned int k = 0; k < this->PlanarIndex.size(); ++k)
  {
    p = this->pclCurrentFrameByScan[this->PlanarIndex[k].first]->points[this->PlanarIndex[k].second];
    this->PlanarsPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2))));
  }
  for (unsigned int k = 0; k < this->BlobIndex.size();  ++k)
  {
    p = this->pclCurrentFrameByScan[this->BlobIndex[k].first]->points[this->BlobIndex[k].second];
    this->BlobsPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2))));
  }

  // keypoints extraction informations
  std::cout << "Extracted Edges: " << this->EdgesPoints->size() << " Planars: "
            << this->PlanarsPoints->size() << " Blobs: "
            << this->BlobsPoints->size() << std::endl;
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::DisplayUsedKeypoints(vtkSmartPointer<vtkPolyData> input,
                                                         const std::vector<int>& EdgePointRejectionEgoMotion,
                                                         const std::vector<int>& EdgePointRejectionMapping,
                                                         const std::vector<int>& PlanarPointRejectionEgoMotion,
                                                         const std::vector<int>& PlanarPointRejectionMapping)
{
  vtkSmartPointer<vtkIntArray> edgeUsedEgoMotion = vtkSmartPointer<vtkIntArray>::New();
  edgeUsedEgoMotion->Allocate(input->GetNumberOfPoints());
  edgeUsedEgoMotion->SetName("Edges_Used_EgoMotion");

  vtkSmartPointer<vtkIntArray> edgeUsedMapping = vtkSmartPointer<vtkIntArray>::New();
  edgeUsedMapping->Allocate(input->GetNumberOfPoints());
  edgeUsedMapping->SetName("Edges_Used_Mapping");

  vtkSmartPointer<vtkIntArray> planarUsedEgoMotion = vtkSmartPointer<vtkIntArray>::New();
  planarUsedEgoMotion->Allocate(input->GetNumberOfPoints());
  planarUsedEgoMotion->SetName("Planes_Used_EgoMotion");

  vtkSmartPointer<vtkIntArray> planarUsedMapping = vtkSmartPointer<vtkIntArray>::New();
  planarUsedMapping->Allocate(input->GetNumberOfPoints());
  planarUsedMapping->SetName("Planes_Used_Mapping");

  // fill with -1 for points that are not keypoints
  for (unsigned int k = 0; k < input->GetNumberOfPoints(); ++k)
  {
    edgeUsedEgoMotion->InsertNextTuple1(-1);
    edgeUsedMapping->InsertNextTuple1(-1);
    planarUsedEgoMotion->InsertNextTuple1(-1);
    planarUsedMapping->InsertNextTuple1(-1);
  }

  // fill with 1 if the point is a keypoint
  // fill with 2 if the point is a used keypoint
  for (unsigned int k = 0; k < this->EdgesIndex.size(); ++k)
  {
    unsigned int scan = this->EdgesIndex[k].first;
    unsigned int index = this->EdgesIndex[k].second;

    edgeUsedEgoMotion->SetTuple1(this->FromPCLtoVTKMapping[scan][index], EdgePointRejectionEgoMotion[k]);
    edgeUsedMapping->SetTuple1(this->FromPCLtoVTKMapping[scan][index], EdgePointRejectionMapping[k]);
  }
  for (unsigned int k = 0; k < this->PlanarIndex.size(); ++k)
  {
    unsigned int scan = this->PlanarIndex[k].first;
    unsigned int index = this->PlanarIndex[k].second;

    planarUsedEgoMotion->SetTuple1(this->FromPCLtoVTKMapping[scan][index], PlanarPointRejectionEgoMotion[k]);
    planarUsedMapping->SetTuple1(this->FromPCLtoVTKMapping[scan][index], PlanarPointRejectionMapping[k]);
  }

  input->GetPointData()->AddArray(edgeUsedEgoMotion);
  input->GetPointData()->AddArray(edgeUsedMapping);
  input->GetPointData()->AddArray(planarUsedEgoMotion);
  input->GetPointData()->AddArray(planarUsedMapping);
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::DisplayLaserIdMapping(vtkSmartPointer<vtkPolyData> input)
{
  vtkDataArray* idsArray = input->GetPointData()->GetArray("laser_id");
  vtkSmartPointer<vtkIntArray> laserMappingArray = vtkSmartPointer<vtkIntArray>::New();
  laserMappingArray->Allocate(input->GetNumberOfPoints());
  laserMappingArray->SetName("laser_mapping");
  for (unsigned int k = 0; k < input->GetNumberOfPoints(); ++k)
  {
    int id = static_cast<int>(idsArray->GetTuple1(k));
    id = this->LaserIdMapping[id];
    laserMappingArray->InsertNextTuple1(id);
  }
  input->GetPointData()->AddArray(laserMappingArray);
}

//-----------------------------------------------------------------------------
void vtkRotatingKeyPointsExtractor::DisplayRelAdv(vtkSmartPointer<vtkPolyData> input)
{
  vtkSmartPointer<vtkDoubleArray> relAdvArray = vtkSmartPointer<vtkDoubleArray>::New();
  relAdvArray->Allocate(input->GetNumberOfPoints());
  relAdvArray->SetName("relative_adv");
  for (unsigned int k = 0; k < input->GetNumberOfPoints(); ++k)
  {
    unsigned int scan = this->FromVTKtoPCLMapping[k].first;
    unsigned int index = this->FromVTKtoPCLMapping[k].second;
    relAdvArray->InsertNextTuple1(this->pclCurrentFrameByScan[scan]->points[index].intensity);
  }
  input->GetPointData()->AddArray(relAdvArray);
}

//-----------------------------------------------------------------------------
template <typename T, typename Tvtk>
void vtkRotatingKeyPointsExtractor::AddVectorToPolydataPoints(const std::vector<std::vector<T>>& vec, const char* name, vtkPolyData* pd)
{
  vtkSmartPointer<Tvtk> array = vtkSmartPointer<Tvtk>::New();
  array->Allocate(pd->GetNumberOfPoints());
  array->SetName(name);
  for (unsigned int k = 0; k < pd->GetNumberOfPoints(); ++k)
  {
    unsigned int scan = this->FromVTKtoPCLMapping[k].first;
    unsigned int index = this->FromVTKtoPCLMapping[k].second;
    array->InsertNextTuple1(vec[scan][index]);
  }
  pd->GetPointData()->AddArray(array);
}
