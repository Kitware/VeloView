#include "vtkBoundingBoxReader.h"

#include <yaml-cpp/yaml.h>

#include <vtkInformationVector.h>
#include <vtkFieldData.h>
#include <vtkNew.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkStringArray.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <Eigen/Geometry>

#include "vtkEigenTools.h"
#include "vtkHelper.h"

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
  vtkNew<vtkPoints> pts;
  bb->SetPoints(pts);
  pts->InsertNextPoint(x, y, 0);
  pts->InsertNextPoint(x+w, y, 0);
  pts->InsertNextPoint(x+w, y+h, 0);
  pts->InsertNextPoint(x, y+h, 0);

  // Fill cells
  vtkNew<vtkCellArray> cell;
  vtkNew<vtkPolyLine> polyLine;
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

vtkSmartPointer<vtkPolyData> CreateBoundingBox3D(const Eigen::Isometry3d& p, const Eigen::Vector3d& d)
{
  assert(d(0) > 0 && "d1 must be strictly positive");
  assert(d(1) > 0 && "d2 must be strictly positive");
  assert(d(2) > 0 && "d3 must be strictly positive");

  auto bb = vtkSmartPointer<vtkPolyData>::New();

  // Fill points
  vtkNew<vtkPoints> pts;
  bb->SetPoints(pts);
  // top
  pts->InsertNextPoint(-d(0)/2, -d(1)/2, +d(2)/2);
  pts->InsertNextPoint(+d(0)/2, -d(1)/2, +d(2)/2);
  pts->InsertNextPoint(+d(0)/2, +d(1)/2, +d(2)/2);
  pts->InsertNextPoint(-d(0)/2, +d(1)/2, +d(2)/2);
  // bottom
  pts->InsertNextPoint(-d(0)/2, -d(1)/2, -d(2)/2);
  pts->InsertNextPoint(+d(0)/2, -d(1)/2, -d(2)/2);
  pts->InsertNextPoint(+d(0)/2, +d(1)/2, -d(2)/2);
  pts->InsertNextPoint(-d(0)/2, +d(1)/2, -d(2)/2);

  // Fill cells
  vtkNew<vtkCellArray> cell;
  bb->SetLines(cell);
  // top
  vtkNew<vtkPolyLine> polyLineBottom;
  polyLineBottom->GetPointIds()->SetNumberOfIds(5);
  polyLineBottom->GetPointIds()->SetId(0, 0);
  polyLineBottom->GetPointIds()->SetId(1, 1);
  polyLineBottom->GetPointIds()->SetId(2, 2);
  polyLineBottom->GetPointIds()->SetId(3, 3);
  polyLineBottom->GetPointIds()->SetId(4, 0);
  cell->InsertNextCell(polyLineBottom.Get());

  // bottom
  vtkNew<vtkPolyLine> polyLineTop;
  polyLineTop->GetPointIds()->SetNumberOfIds(5);
  polyLineTop->GetPointIds()->SetId(0, 4);
  polyLineTop->GetPointIds()->SetId(1, 5);
  polyLineTop->GetPointIds()->SetId(2, 6);
  polyLineTop->GetPointIds()->SetId(3, 7);
  polyLineTop->GetPointIds()->SetId(4, 4);
  cell->InsertNextCell(polyLineTop.Get());

  // vertical line
  for (int j = 0; j < 4; j++)
  {
    vtkNew<vtkPolyLine> polyLine;
    polyLine->GetPointIds()->SetNumberOfIds(2);
    polyLine->GetPointIds()->SetId(0, j % 8);
    polyLine->GetPointIds()->SetId(1, (j + 4) % 8);
    cell->InsertNextCell(polyLine.Get());
  }

  // construct the transform
  vtkNew<vtkMatrix4x4> m;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
    {
      m->SetElement(i, j, p(i, j));
    }
  vtkNew<vtkTransform> t;
  t->SetMatrix(m);

  // transform the bounding box
  vtkNew<vtkTransformPolyDataFilter> transformFilter;
  transformFilter->SetInputData(0, bb);
  transformFilter->SetTransform(t);
  transformFilter->Update();
  return transformFilter->GetOutput();
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
int vtkBoundingBoxReader::RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *outputVector)
{
  if (this->FileName.empty())
  {
    vtkErrorMacro("Please specify a file name")
    return VTK_ERROR;
  }
  auto *output = vtkMultiBlockDataSet::GetData(outputVector->GetInformationObject(0));

  YAML::Node node = YAML::LoadFile(this->FileName);
  YAML::Node objects = node["objects"];
  if (!objects)
    vtkErrorMacro("In yaml file, no 'objects' key at level 0!")

  output->SetNumberOfBlocks(objects.size());
  for (int i = 0; i < objects.size(); ++i)
  {
    try {
      std::string type = objects[i]["selector"]["type"].as<std::string>();
      vtkSmartPointer<vtkPolyData> bb = nullptr;
      if (type == "2D bounding box")
      {
        std::vector<double> center = objects[i]["selector"]["center"].as<std::vector<double>>();
        center[1] = this->ImageHeight - center[1]; // due to image processing vs vtk convention
        std::vector<double> dimension= objects[i]["selector"]["dimensions"].as<std::vector<double>>();

        bb = CreateBoundingBox2D(center[0] - dimension[0]/2.,center[1] - dimension[1]/2., dimension[0], dimension[1]);
      }
      else if (type == "3D bounding box")
      {
        std::vector<double> center = objects[i]["selector"]["center"].as<std::vector<double>>();
        std::vector<double> rotation = objects[i]["selector"]["rotation"].as<std::vector<double>>();

        Eigen::Matrix3d r = RollPitchYawInDegreeToMatrix(rotation[0], rotation[1], rotation[2]);
        Eigen::Translation3d t(center[0], center[1], center[2]);
        Eigen::Isometry3d pose(t*Eigen::Quaterniond(r));

        std::vector<double> dimension= objects[i]["selector"]["dimensions"].as<std::vector<double>>();
        Eigen::Vector3d d(dimension.data());

        bb = CreateBoundingBox3D(pose, d);
      }
      else
      {
        vtkErrorMacro("Either the 'type' key is missing or this type of bounding box is not supported")
      }

      // Add label
      std::string label = objects[i]["label"].as<std::string>();
      auto labelData = createArray<vtkStringArray>("Label", 1, 1);
      labelData->SetValue(0, label);
      bb->GetFieldData()->AddArray(labelData);

      output->SetBlock(i, bb);

    } catch (YAML::BadConversion& e) {
      vtkErrorMacro(<< "YAML::BadConversion : " << e.what() << "\nThe yaml file is ill-formed")
    }
  }

  return VTK_OK;
}
