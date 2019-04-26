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
#include "SpinningSensorKeypointExtractor.h"

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Geometry>

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
  // Position
  this->Position = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - this->Position.transpose();
  Eigen::Matrix3d varianceCovariance = centered.transpose() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(varianceCovariance);

  // Direction
  this->Direction = eig.eigenvectors().col(2).normalized();

  // Semi distance matrix
  // (polar form associated to
  // a bilineare symmetric positive
  // semi-definite matrix)
  this->SemiDist = (this->I3 - this->Direction * this->Direction.transpose());

  bool isLineFittingAccurate = true;

  // if a point of the neighborhood is too far from
  // the fitting line we considere the neighborhood as
  // non flat
  double squaredMaxDistance = std::pow(this->MaxDistance, 2);
  for (unsigned int k = 0; k < points.size(); k++)
  {
    double d = (points[k] - this->Position).transpose() * this->SemiDist * (points[k] - this->Position);
    if (d > squaredMaxDistance)
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
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Reset the pcl format pointcloud to store the new frame
  this->pclCurrentFrameByScan.resize(this->NLasers);
  for (unsigned int k = 0; k < this->NLasers; ++k)
  {
    this->pclCurrentFrameByScan[k].reset(new pcl::PointCloud<Point>());
  }

  this->EdgesPoints.reset(new pcl::PointCloud<Point>());
  this->PlanarsPoints.reset(new pcl::PointCloud<Point>());
  this->BlobsPoints.reset(new pcl::PointCloud<Point>());

  this->Angles.clear();
  this->Angles.resize(this->NLasers);
  this->SaillantPoint.clear();
  this->SaillantPoint.resize(this->NLasers);
  this->DepthGap.clear();
  this->DepthGap.resize(this->NLasers);
  this->IntensityGap.clear();
  this->IntensityGap.resize(this->NLasers);
  this->IsPointValid.clear();
  this->IsPointValid.resize(this->NLasers);
  this->Label.clear();
  this->Label.resize(this->NLasers);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  int nbPoints = this->pclCurrentFrame->size();
  double frameStartTime = this->pclCurrentFrame->points[0].time;
  double frameDuration = this->pclCurrentFrame->points[nbPoints-1].time - frameStartTime;

  for (size_t index = 0; index < nbPoints; ++index)
  {
    const Point& oldPoint = this->pclCurrentFrame->points[index];
    int id = this->LaserIdMapping[oldPoint.laserId];
    // modify the point so that:
    // - laserId is corrected with the laserIdMapping
    // - time become a relative advancement time (between 0 and 1)
    Point newPoint(oldPoint);
    newPoint.laserId = id;
    newPoint.time = (oldPoint.time - frameStartTime) / frameDuration;

    // add the current point to its corresponding laser scan
    this->pclCurrentFrameByScan[id]->push_back(newPoint);
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(pcl::PointCloud<Point>::Ptr pc, std::vector<size_t> laserIdMapping)
{
  if (this->LaserIdMapping.empty())
  {
    this->NLasers = laserIdMapping.size();
    this->LaserIdMapping = laserIdMapping;
  }
  this->pclCurrentFrame = pc;
  this->PrepareDataForNextFrame();
  this->ConvertAndSortScanLines();
  // Initialize the vectors with the correct length
  for (unsigned int k = 0; k < this->NLasers; ++k)
  {
    size_t nbPoint = this->pclCurrentFrameByScan[k]->size();
    this->IsPointValid[k].resize(nbPoint, 1);
    this->Label[k].resize(nbPoint, 0);
    this->Angles[k].resize(nbPoint, 0);
    this->SaillantPoint[k].resize(nbPoint, 0);
    this->DepthGap[k].resize(nbPoint, 0);
    this->IntensityGap[k].resize(nbPoint, 0);
  }

  // Invalid points with bad criteria
  this->InvalidPointWithBadCriteria();

  // compute keypoints scores
  this->ComputeCurvature();

  // labelize keypoints
  this->SetKeyPointsLabels();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  double squaredDistToLineThreshold = std::pow(this->DistToLineThreshold, 2);
  double squaredDepthDistCoeff = 0.25;
  // loop over scans lines
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    Point currentPoint, nextPoint, previousPoint;
    Eigen::Vector3d X, centralPoint;
    LineFitting leftLine, rightLine, farNeighborsLine;

    // We will compute the line that fit the neighbors located
    // previously the current. We will do the same for the
    // neighbors located after the current points. We will then
    // compute the angle between these two lines as an approximation
    // of the "sharpness" of the current point.
    std::vector<Eigen::Vector3d> leftNeighbor(this->NeighborWidth);
    std::vector<Eigen::Vector3d> rightNeighbor(this->NeighborWidth);
    std::vector<Eigen::Vector3d> farNeighbors;
    farNeighbors.reserve(3 * this->NeighborWidth);

    // loop over points in the current scan line
    int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (Npts < 2 * this->NeighborWidth + 1)
    {
      continue;
    }

    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // Skip curvature computation for invalid points
      if (this->IsPointValid[scanLine][index] == 0)
      {
        continue;
      }

      // central point
      currentPoint = this->pclCurrentFrameByScan[scanLine]->points[index];
      centralPoint << currentPoint.x, currentPoint.y, currentPoint.z;

      // compute intensity gap
      nextPoint = this->pclCurrentFrameByScan[scanLine]->points[index + 1];
      previousPoint = this->pclCurrentFrameByScan[scanLine]->points[index - 1];
      this->IntensityGap[scanLine][index] = std::abs(nextPoint.intensity - previousPoint.intensity);

      // Fill right and left neighborhood
      // /!\ The way the neighbors are added
      // to the vectors matters. Especially when
      // computing the saillancy
      for (int j = index - this->NeighborWidth; j < index; ++j)
      {
        currentPoint = this->pclCurrentFrameByScan[scanLine]->points[j];
        leftNeighbor[j -index + this->NeighborWidth] << currentPoint.x, currentPoint.y, currentPoint.z;
      }
      for (int j = index + 1; j <= index + this->NeighborWidth; ++j)
      {
        currentPoint = this->pclCurrentFrameByScan[scanLine]->points[j];
        rightNeighbor[j - index - 1] << currentPoint.x, currentPoint.y, currentPoint.z;
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
        dist1 = (centralPoint - leftLine.Position).transpose() * leftLine.SemiDist * (centralPoint - leftLine.Position);
        dist2 = (centralPoint - rightLine.Position).transpose() * rightLine.SemiDist * (centralPoint - rightLine.Position);

        if ((dist1 < squaredDistToLineThreshold) && (dist2 < squaredDistToLineThreshold))
          this->Angles[scanLine][index] = std::abs((leftLine.Direction.cross(rightLine.Direction)).norm()); // sin of angle actually
      }
      // Here one side of the neighborhood is non flat
      // Hence it is not worth to estimate the sharpness.
      // Only the gap will be considered here.
      else if (rightFlat && !leftFlat)
      {
        dist1 = std::numeric_limits<double>::max();
        for (unsigned int neighIndex = 0; neighIndex < leftNeighbor.size(); ++neighIndex)
        {
          dist1 = std::min(dist1,
                  ((leftNeighbor[neighIndex] - rightLine.Position).transpose() * rightLine.SemiDist * (leftNeighbor[neighIndex] - rightLine.Position))(0));
        }
        dist1 = squaredDepthDistCoeff * dist1;
      }
      else if (!rightFlat && leftFlat)
      {
        dist2 = std::numeric_limits<double>::max();
        for (unsigned int neighIndex = 0; neighIndex < leftNeighbor.size(); ++neighIndex)
        {
          dist2 = std::min(dist2,
                  ((rightNeighbor[neighIndex] - leftLine.Position).transpose() * leftLine.SemiDist * (rightNeighbor[neighIndex] - leftLine.Position))(0));
        }
        dist2 = squaredDepthDistCoeff * dist2;
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
        farNeighbors.resize(0);
        for (unsigned int neighIndex = 0; neighIndex < leftNeighbor.size(); ++neighIndex)
        {
          // Left neighborhood depth gap computation
          if ((std::abs(leftNeighbor[leftNeighbor.size() - 1 - neighIndex].norm() - currDepth) > 1.5) && canLeftBeAdded)
          {
            hasLeftEncounteredDepthGap = true;
            diffDepth++;
            farNeighbors.emplace_back(leftNeighbor[neighIndex]);
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
            farNeighbors.emplace_back(rightNeighbor[neighIndex]);
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
          this->SaillantPoint[scanLine][index] =
            (centralPoint - farNeighborsLine.Position).transpose() * farNeighborsLine.SemiDist * (centralPoint - farNeighborsLine.Position);
        }
      }
      this->DepthGap[scanLine][index] = std::max(dist1, dist2);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidPointWithBadCriteria()
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
void SpinningSensorKeypointExtractor::SetKeyPointsLabels()
{
  this->EdgesIndex.clear(); this->EdgesIndex.resize(0);
  this->PlanarIndex.clear(); this->PlanarIndex.resize(0);
  this->BlobIndex.clear(); this->BlobIndex.resize(0);
  double squaredEdgeDepthGapThreshold = std::pow(this->EdgeDepthGapThreshold, 2);

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
    std::vector<double> IsPointValidForPlanar = this->IsPointValid[scanLine];

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
      if (depthGap < squaredEdgeDepthGapThreshold)
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
      if (saillancy < this->SaillancyThreshold)
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
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2)));
  }
  for (unsigned int k = 0; k < this->PlanarIndex.size(); ++k)
  {
    p = this->pclCurrentFrameByScan[this->PlanarIndex[k].first]->points[this->PlanarIndex[k].second];
    this->PlanarsPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2)));
  }
  for (unsigned int k = 0; k < this->BlobIndex.size();  ++k)
  {
    p = this->pclCurrentFrameByScan[this->BlobIndex[k].first]->points[this->BlobIndex[k].second];
    this->BlobsPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2)));
  }
  this->FarestKeypointDist = std::sqrt(this->FarestKeypointDist);

  // keypoints extraction informations
  std::cout << "Extracted Edges: " << this->EdgesPoints->size() << " Planars: "
            << this->PlanarsPoints->size() << " Blobs: "
            << this->BlobsPoints->size() << std::endl;
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<double> >
SpinningSensorKeypointExtractor::GetDebugArray()
{
  auto get1DVector =  [this](std::vector<std::vector<double>> array) {
    std::vector<double> v (this->pclCurrentFrame->size());
    std::vector<int> indexPerByScanLine(this->NLasers, 0);
    for (int i = 0; i < this->pclCurrentFrame->size(); i++)
    {
      double laserId = this->LaserIdMapping[this->pclCurrentFrame->points[i].laserId];
      v[i] = array[laserId][indexPerByScanLine[laserId]];
      indexPerByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<double> > map;
  map["angles_line"]    = get1DVector(this->Angles);
  map["saillant_point"] = get1DVector(this->SaillantPoint);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["is_point_valid"] = get1DVector(this->IsPointValid);
  map["keypoint_label"] = get1DVector(this->Label);
  return map;
}
