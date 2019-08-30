//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Date: 08-02-2019
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
#include "vtkPCLConversions.h"
#include "vtkTemporalTransforms.h"
#include "vtkCustomTransformInterpolator.h"
#include "CameraModel.h"
#include "vtkBoundingBoxReader.h"
#include "vtkBoundingBox.h"
#include "CameraProjection.h"
#include "BoundingBox.h"
#include "vtkEigenTools.h"

// STD
#include <iostream>
#include <sstream>

// BOOST
#include <boost/filesystem.hpp>

// YAML
#include <yaml-cpp/yaml.h>

// VTK
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkImageData.h>
#include <vtkXMLImageDataReader.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkTransform.h>
#include <vtkJPEGReader.h>
#include <vtkPNGReader.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkFieldData.h>
#include <vtk_jsoncpp.h>

class SemanticCentroid
{
public:
  Eigen::Vector3d center;
  int class_id;
  std::string type;
};

//------------------------------------------------------------------------------
size_t GetNumberOfClouds(std::string cloudFrameSeries)
{
  YAML::Node series = YAML::LoadFile(cloudFrameSeries);
  return series["files"].size();
}

void ReadFromSeries(std::string fileSeries, size_t index, std::string& path, double& time)
{
  YAML::Node series = YAML::LoadFile(fileSeries);
  YAML::Node files = series["files"];

  // compute absolute file paths from the relative one present in the .series
  boost::filesystem::path dirname = boost::filesystem::path(fileSeries).parent_path();
  boost::filesystem::path basename = boost::filesystem::path(files[index]["name"].as<std::string>());
  path = (dirname / basename).string();

  time = files[index]["time"].as<double>();
}

//------------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> ReadCloudFrame(std::string pathToVTP)
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(pathToVTP.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyData> cloud = reader->GetOutput();

  return cloud;
}

//------------------------------------------------------------------------------
std::pair<Eigen::Matrix3d, Eigen::Vector3d> GetRTFromTime(vtkSmartPointer<vtkCustomTransformInterpolator> interpolator,
                                                          double time)
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  interpolator->InterpolateTransform(time, transform);
  vtkSmartPointer<vtkMatrix4x4> M0 = vtkSmartPointer<vtkMatrix4x4>::New();
  transform->GetMatrix(M0);
  Eigen::Matrix3d R0;
  Eigen::Vector3d T0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      R0(i, j) = M0->Element[i][j];
    }
    T0(i) = M0->Element[i][3];
  }

  return std::pair<Eigen::Matrix3d, Eigen::Vector3d>(R0, T0);
}

//------------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> ReferenceFrameChange(vtkSmartPointer<vtkPolyData> cloud,
                                                  vtkSmartPointer<vtkCustomTransformInterpolator> interpolator,
                                                  double time)
{
  vtkSmartPointer<vtkPolyData> transformedCloud = vtkSmartPointer<vtkPolyData>::New();
  transformedCloud->DeepCopy(cloud);
  vtkSmartPointer<vtkDataArray> timeArray = transformedCloud->GetPointData()->GetArray("adjustedtime");

  // Get the target reference frame pose
  std::pair<Eigen::Matrix3d, Eigen::Vector3d> H0 = GetRTFromTime(interpolator, time);

  // transforms the points
  for (int pointIdx = 0; pointIdx < transformedCloud->GetNumberOfPoints(); ++pointIdx)
  {
    // Get the point and its time
    double pt[3];
    transformedCloud->GetPoint(pointIdx, pt);
    double ptTime = 1e-6 * static_cast<double>(timeArray->GetTuple1(pointIdx));
    Eigen::Vector3d X(pt[0], pt[1], pt[2]);

    // Get the transformed associated to the current time
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> H1 = GetRTFromTime(interpolator, ptTime);

    // Reference frame changing
    Eigen::Vector3d Y = H0.first.transpose() * (H1.first * X + H1.second - H0.second);
    double newPt[3] = {Y(0), Y(1), Y(2)};
    transformedCloud->GetPoints()->SetPoint(pointIdx, newPt);
  }

  return transformedCloud;
}

//------------------------------------------------------------------------------
std::vector<OrientedBoundingBox<2>> Create2DBBFromPolyData(vtkSmartPointer<vtkMultiBlockDataSet> vtkBBs)
{
  std::vector<OrientedBoundingBox<2>> bbList;
  for (unsigned int bbIdx = 0; bbIdx < vtkBBs->GetNumberOfBlocks(); ++bbIdx)
  {
    vtkSmartPointer<vtkPolyData> vtkBB = vtkPolyData::SafeDownCast(vtkBBs->GetBlock(bbIdx));
    double minCorner[3], maxCorner[3];
    vtkBB->GetPoint(0, minCorner);
    vtkBB->GetPoint(2, maxCorner);

    OrientedBoundingBox<2> bb(Eigen::Vector2d(minCorner[0], minCorner[1]), Eigen::Vector2d(maxCorner[0], maxCorner[1]));
    bb.Type = vtkBB->GetFieldData()->GetAbstractArray(0)->GetVariantValue(0).ToString();
    bbList.push_back(bb);
  }
  return bbList;
}

//------------------------------------------------------------------------------
std::vector<Eigen::VectorXd> RemovePercentilFarthest(std::vector<Eigen::VectorXd> inputList, double percentil)
{
  std::vector<Eigen::VectorXd> closestPoints(0);
  std::vector<std::pair<double, size_t>> toSortWithDist(inputList.size());
  for (size_t i = 0; i < inputList.size(); ++i)
  {
    toSortWithDist[i].first = inputList[i].norm();
    toSortWithDist[i].second = i;
  }

  std::sort(toSortWithDist.begin(), toSortWithDist.end());

  size_t maxIndex = static_cast<size_t>(std::ceil(static_cast<double>(inputList.size() - 1) * percentil));
  for (size_t i = 0; i <= maxIndex; ++i)
  {
    closestPoints.push_back(inputList[toSortWithDist[i].second]);
  }

  return closestPoints;
}

//------------------------------------------------------------------------------
bool CheckYoloPspConsistency(std::string type, int r, int g, int b)
{
  // 4 wheels Vehicle
  bool carConsistency = (type == "car") && (r == 0) && (g == 0) && (b == 142);
  bool truckConsistency = (type == "truck") && (r == 0) && (g == 0) && (b == 70);
  bool busConsistency = (type == "bus") && (r == 0) && (g == 60) && (b == 100);

  // 2 wheels vehicle
  bool bicycleConsistence = (type == "bicycle") && (r == 119) && (g == 11) && (b == 32);
  bool mototbikeConsistence = (type == "motorbike") && (r == 0) && (g == 0) && (b == 230);

  // signalization
  bool signConsistency = (type == "stop sign") && (r == 220) && (g == 220) && (b == 0);
  bool trafficLightConsistency = (type == "traffic light") && (r == 220) && (g == 220) && (b == 0);

  // persons
  bool personConsistency = (type == "person") && (r == 220) && (g == 20) && (b == 60);

  return carConsistency || truckConsistency || personConsistency || busConsistency ||
         bicycleConsistence || mototbikeConsistence || signConsistency || trafficLightConsistency;
}

//------------------------------------------------------------------------------
std::vector<bool> ComputeBBPSPNetConsistency(std::vector<OrientedBoundingBox<2>> bbList,
                                             vtkSmartPointer<vtkImageData> pspMsk)
{
  std::vector<bool> isConsistent(bbList.size(), true);
  for (size_t bbIdx = 0; bbIdx < bbList.size(); ++bbIdx)
  {
    double pixelCount = 0;
    double pixelConsistentCount = 0;
    for (int col = 0; col < pspMsk->GetDimensions()[0]; ++col)
    {
      for (int row = 0; row < pspMsk->GetDimensions()[1]; ++row)
      {
        if (bbList[bbIdx].IsPointInside(Eigen::Vector2d(col, row)))
        {
          pixelCount += 1.0;
          int r = static_cast<int>(std::round(pspMsk->GetScalarComponentAsDouble(col, row, 0, 0)));
          int g = static_cast<int>(std::round(pspMsk->GetScalarComponentAsDouble(col, row, 0, 1)));
          int b = static_cast<int>(std::round(pspMsk->GetScalarComponentAsDouble(col, row, 0, 2)));

          if (CheckYoloPspConsistency(bbList[bbIdx].Type, r, g, b))
          {
            pixelConsistentCount += 1.0;
          }
        }
      }
    }

    // compute area fraction of consistent pixel
    double consistentFraction = pixelConsistentCount / pixelCount;
    if (consistentFraction < 0.175)
    {
      isConsistent[bbIdx] = false;
    }
  }
  return isConsistent;
}

//------------------------------------------------------------------------------
std::vector<SemanticCentroid> DetectAndComputeCentroid(vtkSmartPointer<vtkPolyData> cloud,
                                                       vtkSmartPointer<vtkImageData> pspMsk,
                                                       vtkSmartPointer<vtkImageData> img,
                                                       vtkSmartPointer<vtkMultiBlockDataSet> bbs,
                                                       Eigen::VectorXd W)
{
  // Convert the polydata to 2D boundingbox structure
  std::vector<OrientedBoundingBox<2>> bbList = Create2DBBFromPolyData(bbs);
  std::vector<std::vector<Eigen::VectorXd>> pointsInBB(bbList.size());

  // Discard a potential bounding box if there is no
  // consistency between psp-net and yolo
  std::vector<bool> isBBConsistent = ComputeBBPSPNetConsistency(bbList, pspMsk);

  // loop over points of the pointcloud
  for (unsigned int pointIdx = 0; pointIdx < cloud->GetNumberOfPoints(); ++pointIdx)
  {
    double pt[3];
    cloud->GetPoint(pointIdx, pt);
    Eigen::Vector3d X(pt[0], pt[1], pt[2]);

    // Project the 3D point onto the image
    Eigen::Vector2d y = BrownConradyPinholeProjection(W, X, true);

    // y represents the pixel coordinates using opencv convention, we need to
    // go back to vtkImageData pixel convention
    int vtkRaw = static_cast<int>(std::round(y(1)));
    int vtkCol = static_cast<int>(std::round(y(0)));

    // Check if the projected point is in the region of interest of the image
    if ((vtkRaw < 0) || (vtkRaw >= img->GetDimensions()[1]) ||
        (vtkCol < 0) || (vtkCol >= img->GetDimensions()[0]))
    {
      continue;
    }

    // loop over the bounding box
    for (unsigned int bbIdx = 0; bbIdx < bbList.size(); ++bbIdx)
    {
      OrientedBoundingBox<2> bb = bbList[bbIdx];
      std::string type = bb.Type;

      // reject over kind of object
      if (type == "car" || type == "truck" || type == "person" ||
          type == "bicycle" || type == "motorbike" || type == "bus" ||
          type == "traffic light" || type == "stop sign")
      {
        // check that the projected point is inside the bounding box
        if (bb.IsPointInside(y) && isBBConsistent[bbIdx])
        {
          // if it is, we still need to check the consistency with
          // the psp-net semantic segmentation
          int r = static_cast<int>(std::round(pspMsk->GetScalarComponentAsDouble(vtkCol, vtkRaw, 0, 0)));
          int g = static_cast<int>(std::round(pspMsk->GetScalarComponentAsDouble(vtkCol, vtkRaw, 0, 1)));
          int b = static_cast<int>(std::round(pspMsk->GetScalarComponentAsDouble(vtkCol, vtkRaw, 0, 2)));

          if (CheckYoloPspConsistency(type, r, g, b))
          {
            // Add the 3D points to the list since it is in the bb frustrum
            pointsInBB[bbIdx].push_back(X);
          }
        }
      }
    }
  }

  // compute the centroid
  std::vector<SemanticCentroid> positions;
  for (unsigned int objectIdx = 0; objectIdx < pointsInBB.size(); ++objectIdx)
  {
    if (pointsInBB[objectIdx].size() > 0)
    {
      // first, only keep the 50 percent points closest to the camera center
      std::vector<Eigen::VectorXd> closestPoints = RemovePercentilFarthest(pointsInBB[objectIdx], 0.30);

      // Then compute the centroid of the remaining points
      SemanticCentroid centroid;
      centroid.center = MultivariateMedian(closestPoints, 1e-6);
      centroid.type = bbList[objectIdx].Type;
      positions.push_back(centroid);
      std::cout << "Centroid of " << centroid.type << " is: " << centroid.center.transpose() << std::endl;
    }
  }

  return positions;
}

//------------------------------------------------------------------------------
std::vector<SemanticCentroid> LaunchDetectionBackProjection(vtkSmartPointer<vtkPolyData> cloud,
                                vtkSmartPointer<vtkCustomTransformInterpolator> interpolator,
                                double timeshift, std::string imageFolder,
                                std::string pspnetFolder, std::string yoloFolder,
                                std::string calibFilename)
{
  // Get the time of the cloud
  // Define the time of the cloud as being the middle time
  vtkDataArray* timeArray = cloud->GetPointData()->GetArray("adjustedtime");
  double time = 1e-6 * static_cast<double>((timeArray->GetTuple1(0) + timeArray->GetTuple1(cloud->GetNumberOfPoints() - 1))) / 2.0 - timeshift;

  // Get the name of the temporally closest image
  std::string filename = imageFolder + "/image.jpg.series";
  YAML::Node imageInfo = YAML::LoadFile(filename);

  double maxTemporalDist = std::numeric_limits<double>::max();
  double closestImgTime = 0;
  int closestImgIndex = -1;
  std::string closestImgName;
  for (unsigned int imgIndex = 0; imgIndex < imageInfo["files"].size(); ++imgIndex)
  {
    double imgTime = imageInfo["files"][imgIndex]["time"].as<double>();
    if (std::abs(imgTime - time) < maxTemporalDist)
    {
      maxTemporalDist = std::abs(imgTime - time);
      closestImgTime = imgTime;
      closestImgIndex = static_cast<int>(imgIndex);
      closestImgName = imageInfo["files"][imgIndex]["name"].as<std::string>();
    }
  }

  if (maxTemporalDist > 1.0) {
      std::cout << "image closest to lidar frame is too far in time. Are exports complete ? You could try running on less lidar frames" << std::endl;
      exit(1);
  }

  // Express the closest image time in the lidar time clock system
  closestImgTime = closestImgTime + timeshift;

  // Now, express the pointcloud in the vehicle reference frame corresponding
  // to the camera timestamp
  vtkSmartPointer<vtkPolyData> transformedCloud = ReferenceFrameChange(cloud, interpolator, closestImgTime);

  // Load the calibration
  CameraModel Model;
  Model.LoadParamsFromFile(calibFilename);

  // load the image
  std::stringstream ss; ss << std::setw(4) << std::setfill('0') << closestImgIndex;
  std::string imgFilename = imageFolder + "/" + ss.str() + ".jpg";
  vtkSmartPointer<vtkJPEGReader> imgReader0 = vtkSmartPointer<vtkJPEGReader>::New();
  imgReader0->SetFileName(imgFilename.c_str());
  imgReader0->Update();
  vtkSmartPointer<vtkImageData> img = imgReader0->GetOutput();

  // load corresponding pspnet
  std::string pspNetMaskFilename = pspnetFolder + "/" + ss.str() + ".png";
  vtkSmartPointer<vtkPNGReader> imgReader = vtkSmartPointer<vtkPNGReader>::New();
  imgReader->SetFileName(pspNetMaskFilename.c_str());
  imgReader->Update();
  vtkSmartPointer<vtkImageData> pspMask = imgReader->GetOutput();

  // load yolo
  std::string yoloBBFilename = yoloFolder + "/" + ss.str() + ".yml";
  vtkSmartPointer<vtkBoundingBoxReader> bbReader = vtkSmartPointer<vtkBoundingBoxReader>::New();
  bbReader->SetFileName(yoloBBFilename);
  bbReader->SetImageHeight(img->GetDimensions()[1]);
  bbReader->Update();
  vtkSmartPointer<vtkMultiBlockDataSet> bbs = bbReader->GetOutput();

  // Launch 3D median center computation
  Eigen::VectorXd W = Model.GetParametersVector();
  std::vector<SemanticCentroid> results = DetectAndComputeCentroid(transformedCloud, pspMask, img, bbs, W);

  return results;
}

//------------------------------------------------------------------------------
void Export3DBBAsYaml(std::vector<std::vector<SemanticCentroid>> objects,
                      std::string outputYamlFile)
{
  YAML::Node ymlFile;
  ymlFile["meta"] = YAML::Node();
  ymlFile["objects"] = YAML::Node();

  ymlFile["meta"]["date"] = std::string("N.A.");
  ymlFile["meta"]["source"] = std::string("N.A.");

  std::vector<SemanticCentroid> allObjects;
  for (size_t i = 0; i < objects.size(); ++i)
  {
    for (size_t j = 0; j < objects[i].size(); ++j)
    {
      allObjects.push_back(objects[i][j]);
    }
  }

  for (size_t i = 0; i < allObjects.size(); ++i)
  {
    YAML::Node currentBB;
    std::string type = allObjects[i].type;
    if (type == "car" || type == "truck" || type == "bus")
    {
      currentBB["label"] = "vehicle";
    }
    else if (type == "bicycle" || type == "motorbike")
    {
      currentBB["label"] = "bike";
    }
    else if (type == "stop sign" || type == "traffic light")
    {
      currentBB["label"] = "sign";
    }
    else
    {
      currentBB["label"] = allObjects[i].type;
    }

    currentBB["custom"] = YAML::Node();
    currentBB["custom"]["confidence"] = 271828;
    currentBB["custom"]["algo"] = "Yolo_PSP_backprojected";

    currentBB["selector"] = YAML::Node();
    currentBB["selector"]["center"].push_back(allObjects[i].center(0));
    currentBB["selector"]["center"].push_back(allObjects[i].center(1));
    currentBB["selector"]["center"].push_back(allObjects[i].center(2));

    currentBB["selector"]["dimensions"].push_back(2.0);
    currentBB["selector"]["dimensions"].push_back(2.0);
    currentBB["selector"]["dimensions"].push_back(2.0);

    currentBB["selector"]["type"] = "3D bounding box";

    ymlFile["objects"].push_back(currentBB);
  }

  std::ofstream fout(outputYamlFile.c_str());
  fout << ymlFile;
}

//------------------------------------------------------------------------------
// negative numbers (python like indexes) are accepted for {first,last}LidarFrameToProcess
// so to run on all frames pass respectively 0 and -1
int main(int argc, char* argv[])
{
  // Check if there is the minimal number of inputs
  if (argc < 8)
  {
    std::cout << "Not enough program inputs" << std::endl;
    return EXIT_FAILURE;
  }

  int firstLidarFrameToProcess = stoi(std::string(argv[1]));
  int lastLidarFrameToProcess = stoi(std::string(argv[2]));
  std::string cloudFrameSeries(argv[3]);
  std::string trajectoryFilename(argv[4]);
  std::string export3DBBFolder(argv[5]);
  double timeshift = std::atof(argv[6]);
  unsigned int nbrCameras = static_cast<unsigned int>(std::atoi(argv[7]));


  // Check if there is the expected number of inputs
  unsigned int expectedNbrInput = 8 + nbrCameras * 4;
  if (static_cast<unsigned int>(argc) != expectedNbrInput)
  {
    std::cout << "Unexpected nbr of inputs" << std::endl;
    std::cout << "Got: " << argc << " inputs" << std::endl;
    std::cout << "expected: " << expectedNbrInput << std::endl;
    return EXIT_FAILURE;
  }

  // Get the folder of the images and detections
  std::vector<std::string> imageFolders(0);
  std::vector<std::string> pspnetFolders(0);
  std::vector<std::string> yoloFolders(0);
  std::vector<std::string> calibFilenames(0);
  for (unsigned int cameraIndex = 0; cameraIndex < nbrCameras; ++cameraIndex)
  {
    imageFolders.push_back(std::string(argv[8 + cameraIndex]));
    pspnetFolders.push_back(std::string(argv[8 + nbrCameras + cameraIndex]));
    yoloFolders.push_back(std::string(argv[8 + 2 * nbrCameras + cameraIndex]));
    calibFilenames.push_back(std::string(argv[8 + 3 * nbrCameras + cameraIndex]));

    std::cout << cameraIndex << std::endl;
    std::cout << std::string(argv[8 + cameraIndex]) << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::string(argv[8 + nbrCameras + cameraIndex]) << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::string(argv[8 + 2 * nbrCameras + cameraIndex]) << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::string(argv[8 + 3 * nbrCameras + cameraIndex]) << std::endl;
    std::cout << std::endl;
  }

  // Load trajectory file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(trajectoryFilename.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyData> polyTraj = reader->GetOutput();
  vtkSmartPointer<vtkTemporalTransforms> trajectory = vtkTemporalTransforms::CreateFromPolyData(polyTraj);
  vtkSmartPointer<vtkCustomTransformInterpolator> interpolator = trajectory->CreateInterpolator();
  interpolator->SetInterpolationTypeToLinear();

  size_t nbrClouds = GetNumberOfClouds(cloudFrameSeries);

  // allow negative (python like) indexes
  if (firstLidarFrameToProcess < 0) {
      firstLidarFrameToProcess += nbrClouds;
  }
  if (lastLidarFrameToProcess < 0) {
      lastLidarFrameToProcess += nbrClouds;
  }

  Json::Value outputSeries;
  Json::Value files(Json::arrayValue);

  // For each lidar frame, launch the detection and tracking process
  for (size_t cloudIndex = static_cast<size_t>(firstLidarFrameToProcess); cloudIndex < static_cast<size_t>(lastLidarFrameToProcess + 1); ++cloudIndex)
  {
    // read cloud
    std::string vtpPath = "";
    double vtpPipelineTime = 0.0; // network time, not used for projection (points have their own lidar time)
    ReadFromSeries(cloudFrameSeries, cloudIndex, vtpPath, vtpPipelineTime);
    vtkSmartPointer<vtkPolyData> cloud = ReadCloudFrame(vtpPath);

    // object detected
    std::vector<std::vector<SemanticCentroid>> objects;

    // for each image, launch detection back projection
    for (unsigned int cameraIndex = 0; cameraIndex < imageFolders.size(); ++cameraIndex)
    {
      std::vector<SemanticCentroid> currentCamObjects = LaunchDetectionBackProjection(cloud, interpolator, timeshift,
                                                                                      imageFolders[cameraIndex],
                                                                                      pspnetFolders[cameraIndex],
                                                                                      yoloFolders[cameraIndex],
                                                                                      calibFilenames[cameraIndex]);
      std::cout << "Added: " << currentCamObjects.size() << " objects for camera: " << cameraIndex << std::endl;
      objects.push_back(currentCamObjects);
    }

    // Export as yaml
    std::string yamlOutput = (boost::filesystem::path(export3DBBFolder) / boost::filesystem::path(vtpPath).stem()).string() + ".yml";
    Export3DBBAsYaml(objects, yamlOutput);
    //std::vector<std::string>["files"] = YAML::NodeType.
    Json::Value node;
    node["name"] = boost::filesystem::path(yamlOutput).filename().string();
    node["time"] = vtpPipelineTime;
    files.append(node);
  }

  outputSeries["files"] = files;
  outputSeries["file-series-version"] = "1.0";

  std::ofstream outputSeriesStream((boost::filesystem::path(export3DBBFolder) / boost::filesystem::path("detections.yml.series")).c_str());
  outputSeriesStream << outputSeries;
  outputSeriesStream.close();

  return EXIT_SUCCESS;
}
