// Copyright 2019 Kitware Inc.
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

#include <vtkAbstractArray.h>
#include <vtkAlgorithm.h>
#include <vtkDataArray.h>
#include <vtkLidarReader.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkSlam.h>
#include <vtkSmartPointer.h>
#include <vtkVelodynePacketInterpreter.h>
#include <vtkXMLPolyDataReader.h>

#include "TestHelpers.h"
#include "Slam.h"

//-----------------------------------------------------------------------------
std::vector<size_t> ComputeLaserMapping(vtkTable* calib)
{
    std::vector<size_t> laserIdMapping;
    vtkAbstractArray * aa = calib->GetColumnByName("verticalCorrection");
    auto array = vtkDataArray::SafeDownCast(aa);

    if (array)
    {
      std::vector<double> verticalCorrection;
      verticalCorrection.resize(array->GetNumberOfTuples());
      for (int i =0; i < array->GetNumberOfTuples(); ++i)
      {
        verticalCorrection[i] = array->GetTuple1(i);
      }
      laserIdMapping = sortIdx(verticalCorrection);
    }
    return laserIdMapping;
}

//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
{  
  if (argc != 4 && argc != 5)
  {
    std::cerr << "Wrong number of arguments. Usage: "
              << "TestSlam <pcapFileName> <referenceFileName> <correctionFileName> "
              << "<max distance between the two trajectories"
              << "(optional, set to 0.5 otherwise)>" << std::endl;
    return 1;
  }

  std::string pcapFileName = argv[1];
  std::string refName = argv[2];
  const char * referenceFileName = refName.c_str();
  std::string  correctionFileName = argv[3];

  double eps = 0.5;
  if(argc == 5){
    std::string epsilon = argv[4];
    eps = std::stod(epsilon.c_str());
  }

  // return value indicate if the test passed
  int retVal = 0;

  std::cout << "--------------------------------------------------------" << std::endl
            << "Pcap :\t" << pcapFileName << std::endl
            << "Baseline:\t" << referenceFileName << std::endl
            << "Corrections :\t" << correctionFileName << std::endl
            << "--------------------------------------------------------" << std::endl;

  vtkNew<vtkXMLPolyDataReader> reader;
  reader->SetFileName(referenceFileName);
  reader->Update();
  vtkSmartPointer<vtkPolyData> expectedTraj = reader->GetOutput();

  // Instantiate a Velodyne HDL reader
  vtkNew<vtkLidarReader> HDLReader;
  auto interp = vtkSmartPointer<vtkVelodynePacketInterpreter>::New();
  HDLReader->SetInterpreter(interp);
  HDLReader->SetFileName(pcapFileName);
  HDLReader->SetCalibrationFileName(correctionFileName);
  HDLReader->Update();

  // Check if we can read the PCAP file
  if (HDLReader->GetNumberOfFrames() == 0)
  {
    std::cout << "ERROR, the reader ouput 0 frame" << std::endl
              << "PLEASE CHECK YOUR PCAP FILE OR FILEPATH" << std::endl;
    return 1;
  }

  Slam slam = Slam();

  for (int idFrame = 0; idFrame < expectedTraj->GetNumberOfPoints(); ++idFrame)
  {
    vtkPolyData* currentFrame = GetCurrentFrame(HDLReader.Get(), idFrame+1);

    // Prepare inputs for Slam Algo
    vtkDataObject* data = HDLReader->GetOutputDataObject(1);
    vtkTable* calib = vtkTable::SafeDownCast(data);
    std::vector<size_t> laserIdMapping = ComputeLaserMapping(calib);
    pcl::PointCloud<Slam::Point>::Ptr pc (new pcl::PointCloud<Slam::Point>);
    PointCloudFromPolyData(currentFrame, pc);

    // Compute the slam algorithm with the new frame
    slam.AddFrame(pc, laserIdMapping);
    Transform t = slam.GetWorldTransform();
    double resSlam[3] = {t.x, t.y, t.z};

    // Get the reference trajectory
    double pointsRef[3];
    expectedTraj->GetPoint(idFrame, pointsRef);

    if(!compare(pointsRef, resSlam, 3, eps))
    {
      retVal +=1;
    }
  }
  return retVal;
}

