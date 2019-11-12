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

#include "MIDHOG.h"

// LOCAL
#include "vtkPCLConversions.h"
#include "CameraProjection.h"
#include "vtkEigenTools.h"

// OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

//----------------------------------------------------------------------------
double ComputeMutualInformation(cv::Mat syntheticImg, cv::Mat realImg, int dx = 0)
{
  const int nbrBin = 256;
  // Compute the histograms
  std::vector<double> h1(nbrBin, 0);
  std::vector<double> h2(nbrBin, 0);
  std::vector<double> h12(nbrBin * nbrBin, 0);

  int H = realImg.rows;
  int W = realImg.cols;

  double countValue = 0;
  // loop over pixels
  for (int i = 0; i < H; ++i)
  {
    for (int j = 0; j < W; ++j)
    {
      int itilde = i + dx;
      if (itilde < 0 || itilde > H - 1)
      {
        continue;
      }
      int value1 = syntheticImg.at<uchar>(itilde, j);
      int value2 = realImg.at<uchar>(i, j);

      // value not available
      if (value1 == 255)
      {
        continue;
      }

      countValue++;
      h1[value1] += 1.0;
      h2[value2] += 1.0;
      h12[value1 + nbrBin * value2] += 1.0;
    }
  }

  for (int i = 0; i < nbrBin; ++i)
  {
    h1[i] /= countValue;
    h2[i] /= countValue;
    for (int j = 0; j < nbrBin; ++j)
    {
      h12[i + nbrBin * j] /= countValue;
    }
  }

  double entropy1 = 0;
  double entropy2 = 0;
  double entropy12 = 0;

  const double minProbability = 1e-10;
  for (int i = 0; i < nbrBin; ++i)
  {
    if (h1[i] > minProbability)
    {
      entropy1 += -h1[i] * std::log(h1[i]);
    }
    if (h2[i] > minProbability)
    {
      entropy2 += -h2[i] * std::log(h2[i]);
    }
    for (int j = 0; j < nbrBin; ++j)
    {
      if (h12[i + nbrBin * j] > minProbability)
      {
        entropy12 += - h12[i + nbrBin * j] * std::log(h12[i + nbrBin * j]);
      }
    }
  }

  return ((entropy1 + entropy2) / entropy12);
}

//----------------------------------------------------------------------------
void ProjectAllPoints(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                      const Eigen::Matrix<double, 17, 1>& cameraParams,
                      std::vector<Eigen::Vector3d>& projectionMatching,
                      std::vector<double>& projectionIntensity,
                      std::vector<double>& projectionDepth,
                      std::vector<double>& projectionNormalAngle,
                      std::vector<bool>& hasPointProjected,
                      unsigned int H,
                      unsigned int W)
{
  // Get the cneter of the camera
  Eigen::Vector3d C(cameraParams[3], cameraParams[4], cameraParams[5]);

  // Project all the points on the image. If two differents points
  // are projected on the same pixel, we keep the closest one (according to
  // the center of the camera)
  for (int i = 0; i < cloud->size(); ++i)
  {
    // Project the point into the image plan
    pcl::PointXYZINormal pt = cloud->points[i];
    Eigen::Vector3d X(pt.x, pt.y, pt.z);
    Eigen::Vector2d y0 = BrownConradyPinholeProjection(cameraParams, X, true);
    Eigen::Vector2d y(H - 1 - y0(1), y0(0));
    if (y(1) < 0 || y(1) >= W ||
        y(0) < 0 || y(0) >= H)
    {
      continue;
    }
    int u = static_cast<int>(std::floor(y(0)));
    int v = static_cast<int>(std::floor(y(1)));

    // Compute the normal angle value
    Eigen::Vector3d XC = (C - X).normalized();
    Eigen::Vector3d N = Eigen::Vector3d(pt.normal_x, pt.normal_y, pt.normal_z).normalized();
    double normalAngle = std::abs(N.dot(XC));

    // If it is the first time a point is projected
    // on this pixel. Just add the point
    if (!hasPointProjected[u + H * v])
    {
      hasPointProjected[u + H * v] = true;
      projectionMatching[u + H * v] = X;
      projectionIntensity[u + H * v] = pt.intensity;
      projectionDepth[u + H * v] = (X - C).norm();
      projectionNormalAngle[u + H * v] = normalAngle;
    }
    // It is not the first point projected on this pixel
    // keep the closest one from the camera center
    else
    {
      double currentD = (projectionMatching[u + H * v] - C).norm();
      double candidateD = (X - C).norm();
      if (candidateD < currentD)
      {
        projectionMatching[u + H * v] = X;
        projectionIntensity[u + H * v] = pt.intensity;
        projectionDepth[u + H * v] = (X - C).norm();
        projectionNormalAngle[u + H * v] = normalAngle;
      }
    }
  }
}

//----------------------------------------------------------------------------
void ComputePointVisibility(std::vector<bool>& isPointVisible,
                            const std::vector<Eigen::Vector3d>& projectionMatching,
                            const std::vector<bool>& hasPointProjected,
                            unsigned int H,
                            unsigned int W,
                            int L)
{
  // Now, handle occultation using heuristic method.
  // For each point, we will estimate the visibility
  for (int i = 0; i < H; ++i)
  {
    for (int j = 0; j < W; ++j)
    {
      if (hasPointProjected[i + H * j])
      {
        // Get current 3D point and its corresponding pixel
        Eigen::Vector2d Xpx(i, j);
        Eigen::Vector3d P = projectionMatching[i + H * j];

        int umin = std::max(0, i - L); int vmin = std::max(0, j - L);
        int umax = std::min((int)(H) - 1, i + L); int vmax = std::min((int)(W) - 1, j + L);
        std::vector<double> solidAnglesPerSector(8, std::numeric_limits<double>::max());
        std::vector<bool> isSectorEmpty(8, true);

        // Loop over points that has been projected
        // in a pixel belonging to the neighborhood
        // of the current pixel we are computing
        for (int u = umin; u <= umax; ++u)
        {
          for (int v = vmin; v <= vmax; ++v)
          {
            if (u == i && v == j)
            {
              continue;
            }

            if (hasPointProjected[u + H * v])
            {
              // compute the image polar angle of the current
              // neighbor pixel to determine the sector it is
              // belonging to
              Eigen::Vector2d Ypx = Eigen::Vector2d(u, v) - Xpx;
              double polar2DAngle = std::atan2(Ypx(1), Ypx(0)) + 3.14159265359;
              int sectorIdx = std::floor(4.0 * polar2DAngle / 3.14159265359);

              // Now, compute the visibility angle
              Eigen::Vector3d Q = projectionMatching[u + H * v];
              Eigen::Vector3d PQ = Q - P;
              Eigen::Vector3d PO = -1.0 * P;
              double angle = std::abs(SignedAngle(PO, PQ));

              if (angle < solidAnglesPerSector[sectorIdx])
              {
                solidAnglesPerSector[sectorIdx] = angle;
                isSectorEmpty[sectorIdx] = false;
              }
            }
          }
        }

        // Now, compute the sum of the minimal angle visibility
        double sumAngle = 0;
        unsigned int nbrNonEmptySector = 0;
        unsigned int nbrSectorNonVisible = 0;
        for (int sectorIdx = 0; sectorIdx < 8; ++sectorIdx)
        {
          if (!isSectorEmpty[sectorIdx])
          {
            sumAngle += solidAnglesPerSector[sectorIdx];
            nbrNonEmptySector++;
            if (solidAnglesPerSector[sectorIdx] < 0.25)
            {
              nbrSectorNonVisible++;
            }
          }
        }

        // remove this points
        /*if (nbrSectorNonVisible >= 5)*/
        if (sumAngle < 2.0)
        {
          isPointVisible[i + H * j] = false;
        }

        /*if (nbrNonEmptySector <= 7)
        {
          isPointVisible[i + H * j] = true;
        }*/
      }
    }
  }
}

//----------------------------------------------------------------------------
int GetSectorId(int u, int v)
{
  int sectorId = -1;
  if (u < 0 && v > 0)
  {
    sectorId = 0;
  }
  else if (u > 0 && v > 0)
  {
    sectorId = 1;
  }
  else if (u > 0 && v < 0)
  {
    sectorId = 2;
  }
  else if (u < 0 && v < 0)
  {
    sectorId = 3;
  }
  return sectorId;
}

//----------------------------------------------------------------------------
double QuadraticInterpolatation(const std::vector<Eigen::Vector2d>& points, const std::vector<double>& values)
{
  if (points.size() < 4 || values.size() < 4)
  {
    return 0;
  }
  Eigen::Matrix<double, 4, 4> M;
  Eigen::Vector4d Y;
  for (int i = 0; i < 4; ++i)
  {
    M(i, 0) = points[i](0) * points[i](1);
    M(i, 1) = points[i](0);
    M(i, 2) = points[i](1);
    M(i, 3) = 1;

    Y(i) = values[i];
  }

  Eigen::Matrix<double, 4, 1> X = M.inverse() * Y;
  return X(3);
}

//----------------------------------------------------------------------------
void ComputeInterpolation(std::vector<cv::Mat> img, int L,
                          const std::vector<Eigen::Vector3d>& projectionMatching,
                          Eigen::Vector3d cameraCenter,
                          unsigned int H, unsigned int W)
{
  for (int imgIndx = 0; imgIndx < img.size(); ++imgIndx)
  {
    // First, copy the input image
    cv::Mat rawImg;
    img[imgIndx].copyTo(rawImg);

    int H = img[imgIndx].rows;
    int W = img[imgIndx].cols;

    // loop over pixels
    for (int i = L; i < H - L; ++i)
    {
      for (int j = L; j < W - L; ++j)
      {
        // Check if the value is already available
        double currentValue = rawImg.at<uchar>(i, j);
        if (currentValue != 255)
        {
          continue;
        }

        Eigen::Vector2d Xpx(i, j);

        // anchor used for interpolation
        std::vector<Eigen::Vector2d> interpolationAnchor(4, Eigen::Vector2d(0, 0));
        std::vector<bool> anchorAvailable(4, false);
        std::vector<double> anchorValue(4, 0);
        std::vector<double> sectorDist(4, std::numeric_limits<double>::max());

        // Loop over the neighborhood. The goal is to compute
        // per sector the anchor point used to perform the
        // quadratic interpolation
        for (int u = i - L; u <= i + L; ++u)
        {
          for (int v = j - L; v <= j + L; ++v)
          {
            if (u == i && v == j)
            {
              continue;
            }

            // Get pixel value and check if data is available
            int value = rawImg.at<uchar>(u, v);
            if (value == 255)
            {
              continue;
            }

            // compute sector id
            int sectorId = GetSectorId(u - i, v - j);

            if (sectorId == -1)
            {
              continue;
            }

            Eigen::Vector2d Ypx(u, v);
            Eigen::Vector3d Y = projectionMatching[u + H * v];

            if (!anchorAvailable[sectorId])
            {
              interpolationAnchor[sectorId] = Ypx - Xpx;
              anchorAvailable[sectorId] = true;
              anchorValue[sectorId] = value;
              sectorDist[sectorId] = (Y - cameraCenter).norm();
            }
            else
            {
              // 2d-euclidean based distance
              /*double currentD = interpolationAnchor[sectorId].norm();
              double candidateD = (Ypx - Xpx).norm();*/

              // 3d-euclidean based distance to cneter of camera
              double candidateD = (Y - cameraCenter).norm();
              if (candidateD < sectorDist[sectorId])
              {
                interpolationAnchor[sectorId] = Ypx - Xpx;
                anchorValue[sectorId] = value;
                sectorDist[sectorId] = candidateD;
              }
            }
          }
        }

        // Fill anchor and values that will be used for the
        // quadratic interpolation
        std::vector<Eigen::Vector2d> points;
        std::vector<double> values;
        for (int sectorId = 0; sectorId < 4; ++sectorId)
        {
          if (anchorAvailable[sectorId])
          {
            points.push_back(interpolationAnchor[sectorId]);
            values.push_back(anchorValue[sectorId]);
          }
        }


        double interpolatedValue = 255;
        // Linear interpolation using a plan
        //if (points.size() == 3)
        //{
        //
        //}
        // Quadratic interpolation using a quadric
        if (points.size() == 4)
        {
         interpolatedValue = QuadraticInterpolatation(points, values);
        }
        img[imgIndx].at<uchar>(i, j) = interpolatedValue;
      }
    }
  }
}

//----------------------------------------------------------------------------
void ComputeMedianFilter(std::vector<cv::Mat> img, int L)
{
  for (int imgIndx = 0; imgIndx < img.size(); ++imgIndx)
  {
    // First, copy the input image
    cv::Mat rawImg;
    img[imgIndx].copyTo(rawImg);

    int H = img[imgIndx].rows;
    int W = img[imgIndx].cols;

    // loop over pixels
    for (int i = L; i < H - L; ++i)
    {
      for (int j = L; j < W - L; ++j)
      {
        // Check if the value is available
        double currentValue = rawImg.at<uchar>(i, j);
        if (currentValue == 255)
        {
          continue;
        }

        std::vector<int> neighborhoodValues;

        // Loop over the neighborhood
        for (int u = i - L; u <= i + L; ++u)
        {
          for (int v = j - L; v <= j + L; ++v)
          {
            // Get pixel value and check if data is available
            int value = rawImg.at<uchar>(u, v);
            if (value == 255)
            {
              continue;
            }
            neighborhoodValues.push_back(value);
          }
        }

        // sort the values
        std::sort(neighborhoodValues.begin(), neighborhoodValues.end());

        int idx = std::floor(((double)neighborhoodValues.size() - 1.0) / 2.0);
        img[imgIndx].at<uchar>(i, j) = neighborhoodValues[idx];
      }
    }
  }
}

//----------------------------------------------------------------------------
void ComputeImageGradient(cv::Mat img, std::vector<Eigen::Vector2d>& gradients, bool manageEmptyData = false)
{
  cv::GaussianBlur(img, img, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

  int H = img.rows;
  int W = img.cols;
  double maxG = 0.0;

  gradients = std::vector<Eigen::Vector2d>(H * W, Eigen::Vector2d(0, 0));
  std::vector<double> gradientNorm(H * W, 0.0);
  for (int i = 0; i < H; ++i)
  {
    for (int j = 0; j < W; ++j)
    {
      int imin = std::max(i - 1, 0);
      int imax = std::min(H - 1, i + 1);

      int jmin = std::max(j - 1, 0);
      int jmax = std::min(W - 1, j + 1);

      uchar currentValue = img.at<uchar>(i, j);

      // Estimate partial derivation using central finite difference
      double gX = ((double)(img.at<uchar>(i, jmax)) - (double)(img.at<uchar>(i, jmin))) / 2.0;
      double gY = ((double)(img.at<uchar>(imax, j)) - (double)(img.at<uchar>(imin, j))) / 2.0;
      double normG = std::sqrt(gX * gX + gY * gY);
      maxG = std::max(maxG, normG);
      gradientNorm[i + H * j] = normG;

      // set gradient value to zero to preserve empty data
      if (manageEmptyData && currentValue == 255)
      {
        gradientNorm[i + H * j] = 0;
      }

      // store the gradient vector
      gradients[i + H * j] = Eigen::Vector2d(gX, gY);
    }
  }

  for (int i = 0; i < H; ++i)
  {
    for (int j = 0; j < W; ++j)
    {
      img.at<uchar>(i, j) = static_cast<uchar>(254.0 * gradientNorm[i + H * j] / maxG);
    }
  }
}

//----------------------------------------------------------------------------
void ComputeHOGDescriptor(const std::vector<Eigen::Vector2d>& gradient,
                          std::vector<std::vector<double>>& HOG,
                          int cellSize, int H, int W)
{
  // To avoid computing multiple time magnitude and angle of gradient,
  // preprocess it
  std::vector<Eigen::Vector2d> polarGradient(H * W);
  for (int k = 0; k < H * W; ++k)
  {
    double gradNorm = gradient[k].norm();
    if (gradNorm < 1e-5)
    {
      polarGradient[k] = Eigen::Vector2d(0, 0);
    }
    else
    {
      double angle = std::abs(atan2(gradient[k](1),  gradient[k](0)));
      polarGradient[k] = Eigen::Vector2d(gradNorm, angle);
    }
  }

  HOG = std::vector<std::vector<double>>(H * W, std::vector<double>(9, 0));

  // loop over pixels
  for (int i = 0; i < H; ++i)
  {
    for (int j = 0; j < W; ++j)
    {
      int umin = std::max(i - cellSize, 0);
      int umax = std::min(H - 1, i + cellSize);

      int vmin = std::max(j - cellSize, 0);
      int vmax = std::min(W - 1, j + cellSize);

      // loop over the cell
      double sumGradNorm = 0;
      for (int u = umin; u <= umax; ++u)
      {
        for (int v = vmin; v <= vmax; ++v)
        {
          int bin = std::floor(9.0 * polarGradient[u + H * v](1) / (vtkMath::Pi() + 0.0001));
          HOG[i + H * j][bin] += polarGradient[u + H * v](0);
          sumGradNorm += polarGradient[u + H * v](0);
        }
      }

      // normalized the histogram
      for (int k = 0; k < 9; ++k)
      {
        HOG[i + H * j][k] /= sumGradNorm;
      }
    }
  }
}

//----------------------------------------------------------------------------
void ComputeHarrisKeypoints(cv::Mat img)
{
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  cv::Mat dst = cv::Mat::zeros(img.size(), CV_32FC1);
  cornerHarris(img, dst, blockSize, apertureSize, k);
  cv::Mat dst_norm, dst_norm_scaled;
  normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
  convertScaleAbs(dst_norm, dst_norm_scaled);
  for( int i = 0; i < dst_norm.rows ; i++ )
  {
      for( int j = 0; j < dst_norm.cols; j++ )
      {
          if( (int) dst_norm.at<float>(i,j) > 200)
          {
              circle(dst_norm_scaled, cv::Point(j,i), 5, cv::Scalar(0), 2, 8, 0 );
          }
      }
  }
  cv::namedWindow("Corners");
  cv::imshow("Corners", dst_norm_scaled);
}

//----------------------------------------------------------------------------
MIDHOGCalibration::MIDHOGCalibration(const cv::Mat& image,
                                     pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                                     const Eigen::Matrix<double, 17, 1>& cameraParams,
                                     int neighborRadius)
{
  this->Cloud = cloud;
  this->CameraParams = cameraParams;
  this->Image = image.clone();
  this->NeighborRadius = neighborRadius;

  this->ComputeCloudNormals();
}

//----------------------------------------------------------------------------
void MIDHOGCalibration::ComputeCloudNormals()
{
  if (!this->Cloud)
  {
    std::cout << "Cloud is empty" << std::endl;
    return;
  }

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZINormal, pcl::PointXYZINormal> normalEstimator;
  normalEstimator.setInputCloud(this->Cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZINormal>());
  normalEstimator.setSearchMethod(kdTree);

  // Use all neighbors in a sphere of radius 25 cm
  normalEstimator.setRadiusSearch (0.30);

  // Compute the features
  normalEstimator.compute(*this->Cloud);
}

//----------------------------------------------------------------------------
void MIDHOGCalibration::CreateSyntheticImage()
{
  // create a white image
  this->SyntheticImage.push_back(255.0 * cv::Mat::ones(this->Image.size(), CV_8UC1));
  this->SyntheticImage.push_back(255.0 * cv::Mat::ones(this->Image.size(), CV_8UC1));
  this->SyntheticImage.push_back(255.0 * cv::Mat::ones(this->Image.size(), CV_8UC1));
  unsigned int H = this->Image.rows;
  unsigned int W = this->Image.cols;

  // Create the synthetic image
  // First, project all the points on the image. If two differents points
  // are projected on the same pixel, we keep the closest one (according to
  // the center of the camera)
  std::vector<Eigen::Vector3d> projectionMatching(this->Image.cols * this->Image.rows, Eigen::Vector3d::Zero());
  std::vector<double> projectionIntensity(this->Image.cols * this->Image.rows, 0);
  std::vector<double> projectionDepth(this->Image.cols * this->Image.rows, 0);
  std::vector<double> projectionNormalAngle(this->Image.cols * this->Image.rows, 0);
  std::vector<bool> hasPointProjected(this->Image.cols * this->Image.rows, false);
  ProjectAllPoints(this->Cloud, this->CameraParams, projectionMatching,
                   projectionIntensity, projectionDepth, projectionNormalAngle,
                   hasPointProjected, H, W);

  // Now, handle occultation using heuristic method.
  // For each point, we will estimate the visibility
  std::vector<bool> isPointVisible(this->Image.cols * this->Image.rows, true);
  ComputePointVisibility(isPointVisible, projectionMatching,
                         hasPointProjected, H, W, this->NeighborRadius);

  // get the max intensity
  double maxD = 0;
  for (int i = 0; i < projectionIntensity.size(); ++i)
  {
    maxD = std::max(projectionIntensity[i], maxD);
  }

  // Once the visibility has been computed, we can create the image of
  // visible projected 3D points
  Eigen::Vector3d C(this->CameraParams[3], this->CameraParams[4], this->CameraParams[5]);
  for (int i = 0; i < H; ++i)
  {
    for (int j = 0; j < W; ++j)
    {
      if (hasPointProjected[i + H * j] && isPointVisible[i + H * j])
      {
        this->SyntheticImage[0].at<uchar>(i, j) = projectionIntensity[i + H * j];
        this->SyntheticImage[1].at<uchar>(i, j) = static_cast<uchar>(254.0 -  254.0 * projectionDepth[i + H * j] / maxD);
        if (!std::isnan(projectionNormalAngle[i + H * j]))
        {
          this->SyntheticImage[2].at<uchar>(i, j) = static_cast<uchar>(254.0 * projectionNormalAngle[i + H * j]);
        }
      }
    }
  }

  // Finally, we will interpole missing data
  ComputeInterpolation(this->SyntheticImage, this->NeighborRadiusInterpolation, projectionMatching, C, H, W);

  // Compute median filter to remove salt noise
  ComputeMedianFilter(this->SyntheticImage, this->NeighborRadiusMedianFilter);

  std::vector<std::vector<Eigen::Vector2d>> syntheticImgGradients(3);
  std::vector<Eigen::Vector2d> realImgGradient;

  // Compute gradient image
  for (int i = 0; i < 3; ++i)
  {
    ComputeImageGradient(this->SyntheticImage[i], syntheticImgGradients[i], true);
  }
  ComputeImageGradient(this->Image, realImgGradient);

  std::vector<std::vector<std::vector<double>>> syntheticImgHOG(3);
  std::vector<std::vector<double>> realImgHOG;

  for (int i = 0; i < 3; ++i)
  {
    ComputeHOGDescriptor(syntheticImgGradients[i], syntheticImgHOG[i], 8, H, W);
  }
  ComputeHOGDescriptor(realImgGradient, realImgHOG, 8, H, W);

  // extract keypoints using harris corner to then, compare
  // their HOG descriptor with the synthetic image ones
  ComputeHarrisKeypoints(this->Image);
}

//----------------------------------------------------------------------------
cv::Mat MIDHOGCalibration::GetSyntheticImage(int idx)
{
  return this->SyntheticImage[idx].clone();
}
