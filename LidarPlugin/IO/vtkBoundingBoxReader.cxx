#include "vtkBoundingBoxReader.h"

#include <yaml-cpp/yaml.h>

#include <vtkInformationVector.h>
#include <vtkFieldData.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkStringArray.h>

#include "vtkHelper.h"

#define vtkAssertWarningMacro(test, text) if (test) {vtkWarningMacro(text); return VTK_ERROR;}
using namespace std;

namespace {
  /**
 * @brief CreateBoundingBox2D create a polyData with a polyline representing the 2D bounding box.
 * As everything is 3D in vtk, the z coordinate is set to 0.
 * @param x upper left corner x coordinate
 * @param y upper left corner y coordinate
 * @param w width (must be strictly positive)
 * @param h height (must be strictly positive)
 */
vtkSmartPointer<vtkPolyData> CreateBoundingBox2D(double x, double y, double w, double h)
{
  assert(w > 0 && "width must be strictly positive");
  assert(h > 0 && "height must be strictly positive");

  auto bb = vtkSmartPointer<vtkPolyData>::New();

  // Fill points
  auto pts = vtkSmartPointer<vtkPoints>::New();
  bb->SetPoints(pts);
  pts->InsertNextPoint(x, y, 0);
  pts->InsertNextPoint(x+w, y, 0);
  pts->InsertNextPoint(x+w, y+h, 0);
  pts->InsertNextPoint(x, y+h, 0);

  // Fill cells
  auto cell = vtkSmartPointer<vtkCellArray>::New();
  auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(5);
  polyLine->GetPointIds()->SetId(0, 0);
  polyLine->GetPointIds()->SetId(1, 1);
  polyLine->GetPointIds()->SetId(2, 2);
  polyLine->GetPointIds()->SetId(3, 3);
  polyLine->GetPointIds()->SetId(4, 0);
  cell->InsertNextCell(polyLine.Get());
  bb->SetLines(cell);

  return bb;
}
}
//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkBoundingBoxReader)

//-----------------------------------------------------------------------------
vtkBoundingBoxReader::vtkBoundingBoxReader()
{
  this->SetNumberOfInputPorts(0);
}

//-----------------------------------------------------------------------------
int vtkBoundingBoxReader::RequestData(vtkInformation *, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  if (this->FileName.empty())
  {
    vtkErrorMacro("Please specify a file name")
    return VTK_ERROR;
  }
  auto *output = vtkMultiBlockDataSet::GetData(outputVector->GetInformationObject(0));

  YAML::Node node = YAML::LoadFile(this->FileName);
  YAML::Node objects = node["objects"];
  vtkAssertWarningMacro(!objects, "No 'objects' key at level 0!")

  output->SetNumberOfBlocks(objects.size());
  for (int i = 0; i < objects.size(); ++i)
  {
    std::string type = objects[i]["selector"]["type"].as<std::string>();
    if (type == "2D bounding box")
    {

      std::string label = objects[i]["label"].as<std::string>();
      double x = objects[i]["selector"]["position"]["x"].as<double>();
      double y = this->ImageHeight - objects[i]["selector"]["position"]["y"].as<double>();
      double w = objects[i]["selector"]["dimension"]["width"].as<double>();
      double h = objects[i]["selector"]["dimension"]["height"].as<double>();

      auto bb = CreateBoundingBox2D(x - w/2., y - w/2., w, h);

      auto labelData = createArray<vtkStringArray>("Label", 1, 1);
      labelData->SetValue(0, label);
      bb->GetFieldData()->AddArray(labelData);

      output->SetBlock(i, bb);

    }
    else
    {
      vtkAssertWarningMacro(true, "Either the 'type' key is missing or this type of bounding box is not supported")
    }
  }

  return VTK_OK;
}
