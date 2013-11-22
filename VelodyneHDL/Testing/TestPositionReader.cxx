// Copyright 2013 Velodyne Acoustics, Inc.
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


#include "vtkVelodyneHDLPositionReader.h"
#include "vtkTestingMacros.h"

#include <vtkNew.h>

#include <string>
#include <iostream>
#include <cstdlib>

int main(int argc, char* argv[])
{
  if (argc < 2)
    {
    std::cout << "Usage: " << argv[0] << " <pcap file> [TEST_SPECS]" << std::endl;
    return 1;
    }

  std::string filename = argv[1];

  vtkNew<vtkVelodyneHDLPositionReader> reader;

  reader->SetFileName(filename);
  reader->Update();

  vtkSmartPointer<vtkPolyData> poly = reader->GetOutput();
  std::cout << "Number of samples: " << poly->GetNumberOfPoints() << std::endl;

  // Validate arguments
  if(argc > 2)
    {
    ASSERT_EQUALS(poly->GetNumberOfPoints(), atoi(argv[2]));
    }

  return 0;
}
