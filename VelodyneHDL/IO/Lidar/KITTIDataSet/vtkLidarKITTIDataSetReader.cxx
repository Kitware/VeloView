#include "vtkLidarKITTIDataSetReader.h"

#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkPolyData.h>
#include <vtkMath.h>

#include <sstream>

# include <boost/filesystem.hpp>

namespace  {
//-----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts * 2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
  {
    ids[i * 2] = 1;
    ids[i * 2 + 1] = i;
  }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

//-----------------------------------------------------------------------------
template<typename T>
vtkSmartPointer<T> CreateDataArray(const char* name, vtkPolyData* pd)
{
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->SetName(name);
  pd->GetPointData()->AddArray(array);
  return array;
}

typedef struct point {
  float x;
  float y;
  float z;
  float intensity;
} point_t;
}

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLidarKITTIDataSetReader)

//----------------------------------------------------------------------------
void vtkLidarKITTIDataSetReader::SetFileName(const std::string &filename)
{
  if (filename == this->FileName)
  {
    return;
  }

  if (!boost::filesystem::exists(filename))
  {
    vtkErrorMacro(<< "Folder not be found! Contrary to what the name of this function implies, \
                    the input must be the folder containing all '.bin' files for a given sequence");
    return;
  }

  // count number of frames inside the folder
  this->NumberOfFrames = 0;
  boost::filesystem::path folder(filename);
  boost::filesystem::directory_iterator it(folder);
  while (it != boost::filesystem::directory_iterator())
  {
    this->NumberOfFrames++;
    *it++;
  }
  this->FileName = filename + "/";
  this->Modified();
}

//-----------------------------------------------------------------------------
std::string vtkLidarKITTIDataSetReader::GetSensorInformation()
{
  return "Velodyne HDL64 sensor playing back data from the KITTI dataset\n \
          The data are the .bin file contain in following folder: " + this->FileName;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkLidarKITTIDataSetReader::GetFrame(int frameNumber)
{
  // create a new empty frame
  vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(0);
  points->GetData()->SetName("Points_m_XYZ");
  poly->SetPoints(points.GetPointer());

  vtkSmartPointer<vtkDoubleArray> xArray = CreateDataArray<vtkDoubleArray>("X", poly);
  vtkSmartPointer<vtkDoubleArray> yArray = CreateDataArray<vtkDoubleArray>("Y", poly);
  vtkSmartPointer<vtkDoubleArray> zArray = CreateDataArray<vtkDoubleArray>("Z", poly);
  vtkSmartPointer<vtkDoubleArray> intensityArray = CreateDataArray<vtkDoubleArray>("intensity", poly);
  vtkSmartPointer<vtkDoubleArray> azimutArray = CreateDataArray<vtkDoubleArray>("azimuth", poly);
  vtkSmartPointer<vtkDoubleArray> elevationArray = CreateDataArray<vtkDoubleArray>("elevation", poly);
  vtkSmartPointer<vtkDoubleArray> radiusArray = CreateDataArray<vtkDoubleArray>("radius", poly);
  vtkSmartPointer<vtkDoubleArray> idArray = CreateDataArray<vtkDoubleArray>("laser_id", poly);
  vtkSmartPointer<vtkDoubleArray> timestamp = CreateDataArray<vtkDoubleArray>("timestamp", poly);
  vtkSmartPointer<vtkDoubleArray> adjustedTime = CreateDataArray<vtkDoubleArray>("adjustedtime", poly);

  int startFrame = std::max(0, frameNumber);
  for (int i = startFrame; i <= frameNumber; i++)
  {
    // produce path to the required .bin file
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << i;
    std::string filename = this->GetFileName() + ss.str() + ".bin";

    ifstream is;
    is.open(filename, ios::binary|ios::in);
    // get length of file:
    is.seekg(0, ios::end);
    int length = is.tellg();
    is.seekg(0, ios::beg);

    // variable used to detect a laser jump
    double old_thetaProj = 0;
    int laser_id = 0;

    // buffer used to read the points
    char buffer[16];
    const int nbPoints = length / 16;
    for (int i = 0; i < nbPoints; i++)
    {
      is.read(buffer, 16);
      point_t* pt;
      pt = reinterpret_cast<point_t*>(buffer);
      double x = pt->x;
      double y = pt->y;
      double z = pt->z;

      double radius = sqrt(x*x + y*y + z*z);
      double thetaProj = 180 / vtkMath::Pi() * std::atan2(pt->y, pt->x);
      double azimut = 180 / vtkMath::Pi() * std::atan2(pt->x, pt->y);

      if (azimut < 0)
        azimut = 360 + azimut;

      if(old_thetaProj < 0 && thetaProj >= 0)
      {
        laser_id++;
        if (laser_id >= this->NbrLaser)
        {
          vtkErrorMacro(<< "An error occur while parsing the frame, more than 64 laser where detected. The last point won't be processed")
          break;
        }
      }
      double projRadius = std::sqrt(pt->x * pt->x + pt->y * pt->y);
      double elevation = 180 / vtkMath::Pi() * std::atan2(pt->z, projRadius);

      double time = azimut / 360.0;

      // fill the polydata
      points->InsertNextPoint(pt->x, pt->y, pt->z);
      xArray->InsertNextValue(pt->x);
      yArray->InsertNextValue(pt->y);
      zArray->InsertNextValue(pt->z);
      radiusArray->InsertNextValue(radius);
      intensityArray->InsertNextValue(pt->intensity);
      azimutArray->InsertNextValue(azimut);
      idArray->InsertNextValue(laser_id);
      elevationArray->InsertNextValue(elevation);
      timestamp->InsertNextValue(time);
      adjustedTime->InsertNextValue(time);

      // update old azimut
      old_thetaProj = thetaProj;
    }
    is.close();
  }

  poly->SetVerts(NewVertexCells(poly->GetNumberOfPoints()));

  return poly;
}

//----------------------------------------------------------------------------
int vtkLidarKITTIDataSetReader::RequestData(vtkInformation* vtkNotUsed(request),
                                            vtkInformationVector** vtkNotUsed(inputVector),
                                            vtkInformationVector* outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector);
  vtkInformation* info = outputVector->GetInformationObject(0);

  int timestep = 0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    timestep = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
  }
  // Because no timestep are present in the .bin file we consider that timestep = frame number.
  if (timestep < 0 || timestep >= this->GetNumberOfFrames())
  {
    vtkErrorMacro("Cannot meet timestep request: " << timestep << ".  Have "
                                                   << this->GetNumberOfFrames() << " datasets.");
    return 0;
  }
  output->ShallowCopy(GetFrame(timestep));
  return 1;
}

//----------------------------------------------------------------------------
int vtkLidarKITTIDataSetReader::RequestInformation(vtkInformation* vtkNotUsed(request),
                                                   vtkInformationVector** vtkNotUsed(inputVector),
                                                   vtkInformationVector* outputVector)
{
  vtkInformation* info = outputVector->GetInformationObject(0);
  int numberOfTimesteps = this->NumberOfFrames;
  std::vector<double> timesteps;
  for (int i = 0; i < numberOfTimesteps; ++i)
  {
    timesteps.push_back(i);
  }

  if (numberOfTimesteps)
  {
    double timeRange[2] = { timesteps.front(), timesteps.back() };
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
  }
  else
  {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
  }
  return 1;
}
