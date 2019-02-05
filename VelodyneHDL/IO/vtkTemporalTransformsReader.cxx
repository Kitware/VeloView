// Copyright 2019 Kitware SAS.
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

#include "vtkTemporalTransformsReader.h"

#include <vtkAbstractArray.h>
#include <vtkDelimitedTextReader.h>
#include <vtkDoubleArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>

#include <Eigen/Geometry>

#include <unordered_map>

#include "vtkTemporalTransforms.h"

namespace {
//-----------------------------------------------------------------------------
vtkDoubleArray* foundArray(vtkTable* table, std::vector<std::string> potentialName)
{
  // try to find the right array
  for (const auto& name: potentialName)
  {
    vtkDoubleArray* tmp = vtkDoubleArray::SafeDownCast(table->GetColumnByName(name.c_str()));
    if (tmp)
    {
      return tmp;
    }
  }
  // throw an exception when no matching could be founded
  std::string errorMessage = "Could not find of the following colum: ";
  for (const auto& name: potentialName)
  {
    errorMessage += name +" , ";
  }
  throw errorMessage;
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, vtkDoubleArray*> createArrayIndex(vtkTable* table)
{
  std::unordered_map<std::string, vtkDoubleArray*> array;
  std::vector<std::string> potentialName;

  std::string time[] = {"Time", "time", "Timestamp", "timestamp"};
  potentialName = std::vector<std::string>(time, time + sizeof(time) / sizeof(time[0]));
  array["time"] = foundArray(table, potentialName);

  std::string roll[] = {"Rx(Roll)", "Roll", "roll", "Rx", "rx"};
  potentialName = std::vector<std::string>(roll, roll + sizeof(roll) / sizeof(roll[0]));
  array["roll"] = foundArray(table, potentialName);

  std::string pitch[] = {"Ry(Pitch)", "Pitch", "pitch", "Ry", "ry"};
  potentialName = std::vector<std::string>(pitch, pitch + sizeof(pitch) / sizeof(pitch[0]));
  array["pitch"] = foundArray(table, potentialName);

  std::string yaw[] = {"Rz(Yaw)", "Yaw", "yaw", "Rz", "rz"};
  potentialName = std::vector<std::string>(yaw, yaw + sizeof(yaw) / sizeof(yaw[0]));
  array["yaw"] = foundArray(table, potentialName);

  std::string x[] = {"X", "x"};
  potentialName = std::vector<std::string>(x, x + sizeof(x) / sizeof(x[0]));
  array["X"] = foundArray(table, potentialName);

  std::string y[] = {"Y", "y"};
  potentialName = std::vector<std::string>(y, y + sizeof(y) / sizeof(y[0]));
  array["Y"] = foundArray(table, potentialName);

  std::string z[] = {"Z", "z"};
  potentialName = std::vector<std::string>(z, z + sizeof(z) / sizeof(z[0]));
  array["Z"] = foundArray(table, potentialName);

  return array;
}
}

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkTemporalTransformsReader)

//-----------------------------------------------------------------------------
vtkTemporalTransformsReader::vtkTemporalTransformsReader()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
int vtkTemporalTransformsReader::RequestData(vtkInformation* vtkNotUsed(request),
                        vtkInformationVector** vtkNotUsed(inputVector),
                        vtkInformationVector* outputVector)
{
  if (!this->FileName)
  {
    vtkErrorMacro(<< "Please select the file to read")
    return 1;
  }

  // Read the data from the file
  vtkNew<vtkDelimitedTextReader> csvReader;
  csvReader->SetFileName(this->FileName);
  csvReader->DetectNumericColumnsOn();
  csvReader->SetFieldDelimiterCharacters(",");
  csvReader->ForceDoubleOn();
  csvReader->SetHaveHeaders(true);
  csvReader->Update();

  vtkTable* table = csvReader->GetOutput();

  if (table->GetNumberOfColumns() < 7)
  {
    vtkErrorMacro( << "The file you try to read has only " << table->GetNumberOfColumns() << " colums."
                   << "This reader needs to have a CVS file with the following colum:"
                   << "time, roll, pitch, yaw, X, Y, Z")
  }

  auto translation = vtkSmartPointer<vtkDoubleArray>::New();
  translation->SetNumberOfComponents(3);
  // The rotation will be store in an axis-angle representation (w, x, y, z) so that
  // - axis = (x, y, z) and norm(axis) = 1
  // - angle = w in radian
  auto axisAngle = vtkSmartPointer<vtkDoubleArray>::New();
  axisAngle->SetNumberOfComponents(4);
  auto timstamp = vtkSmartPointer<vtkDoubleArray>::New();

  // create map for idx
  std::unordered_map<std::string, vtkDoubleArray*> array;
  try {
    array = createArrayIndex(table);
  } catch (std::string e) {
    vtkErrorMacro(<< e)
    return 1;
  }

  // Read the data from the csv table
  for (vtkIdType i = 0; i < table->GetNumberOfRows(); i++)
  {
    translation->InsertNextTuple3(array["X"]->GetTuple1(i),
                                  array["Y"]->GetTuple1(i),
                                  array["Z"]->GetTuple1(i));

    timstamp->InsertNextValue(array["time"]->GetTuple1(i) + this->TimeOffset);

    // Assumption: roll, pitch and yaw are in degree
    double roll  = array["roll"]->GetTuple1(i);
    double pitch = array["pitch"]->GetTuple1(i);
    double yaw   = array["yaw"]->GetTuple1(i);

    auto currentAxisAngleRotation = Eigen::AngleAxisd(
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

    axisAngle->InsertNextTuple4(currentAxisAngleRotation.axis()[0],
                                currentAxisAngleRotation.axis()[1],
                                currentAxisAngleRotation.axis()[2],
                                currentAxisAngleRotation.angle());
  }

  // Create the cell to be able to visualize the data.
  auto trajectory = vtkSmartPointer<vtkTemporalTransforms>::New();
  trajectory->SetTranslationArray(translation);
  trajectory->SetTimeArray(timstamp);
  trajectory->SetOrientationArray(axisAngle);

  // Set the filter output
  vtkPolyData* output = vtkPolyData::GetData(outputVector);
  output->ShallowCopy(trajectory);

  return 1;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransformsReader::OpenTemporalTransforms(const std::string& filename)
{
  auto reader1 = vtkSmartPointer<vtkTemporalTransformsReader>::New();
  reader1->SetFileName(filename.c_str());
  reader1->Update();
  auto temporalTransform1 = reader1->GetOutput();
  return vtkTemporalTransforms::CreateFromPolyData(temporalTransform1);
}
