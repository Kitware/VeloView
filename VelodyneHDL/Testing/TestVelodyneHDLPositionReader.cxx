#include "vtkVelodyneHDLPositionReader.h"
#include "vtkVelodyneTransformInterpolator.h"
#include "vtkPointData.h"
#include "vtkTransform.h"

#include <Eigen/Dense>

Eigen::Matrix3d RollPitchYawToMatrix(double roll, double pitch, double yaw)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

Eigen::Matrix3d RollPitchYawToMatrixDegree(double roll, double pitch, double yaw)
{
  return RollPitchYawToMatrix((vtkMath::Pi() / 180.0) * roll,
                              (vtkMath::Pi() / 180.0) * pitch,
                              (vtkMath::Pi() / 180.0) * yaw);
}

// defines an arbitrary order on the columns names
// this order is expected when passing arrays of double to test functions
// this order does not have to be the same as the one that is used inside the
// vtkPolydata, because we do not want to test that this order remain consistent
// (because the order is not documented and currently based on the behavior of // an iterator on a std::map)
const char* arrayNames[17] = { "lat",
                               "lon",
                               "gpstime",
                               "time",
                               "accel1x",
                               "accel1y",
                               "accel2x",
                               "accel2y",
                               "accel3x",
                               "accel3y",
                               "gyro1",
                               "gyro2",
                               "gyro3",
                               "heading",
                               "temp1",
                               "temp2",
                               "temp3" };

bool Equal(double a, double b, double epsilon = 1e-6)
{
  return std::abs(a - b) <= epsilon;
}

bool test_point_count(vtkSmartPointer<vtkVelodyneHDLPositionReader> reader,
                /* expected: */
                int count)
{
  std::cout << "Testing number of points" << std::endl;
  bool isvalid = (reader->GetOutput()->GetNumberOfPoints() == count);
  if (!isvalid)
  {
    std::cerr << "unexpected number of points" << std::endl;
  }
  return isvalid;
}

bool test_interpolator_time_range(
    vtkSmartPointer<vtkVelodyneHDLPositionReader> reader,
    /* expected: */
    double tmin,
    double tmax)
{
  std::cout << "Testing the interpolator time range" << std::endl;
  bool isvalid = (Equal(reader->GetInterpolator()->GetMinimumT(),tmin)
                  && Equal(reader->GetInterpolator()->GetMaximumT(), tmax));
  if (!isvalid)
  {
    std::cerr << "unexpected interpolator time range" << std::endl;
  }
  return isvalid;
}


// roll pitch and yaw angles in degrees
// Producing the matrix Rz(yaw) * Ry(pitch) * Rx(roll)
bool test_interpolator_transform(
    vtkSmartPointer<vtkVelodyneHDLPositionReader> reader,
    double t,
    /* expected: */
    double posRollPitchYaw[6])
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  reader->GetInterpolator()->InterpolateTransform(t, transform);

  vtkSmartPointer<vtkMatrix4x4> tmp = vtkSmartPointer<vtkMatrix4x4>::New();
  transform->GetMatrix(tmp);

  Eigen::Matrix3d M = RollPitchYawToMatrixDegree(posRollPitchYaw[3 + 0],
      posRollPitchYaw[3 + 1],
      posRollPitchYaw[3 + 2]);

  bool rotationValid = true;
  for (int i = 0; i < 3; i++)
  {
      for (int j = 0; j < 3; j++)
      {
        rotationValid &= Equal(tmp->GetElement(i,j), M(i,j));
      }
  }
  if (!rotationValid)
  {
    std::cerr << "Unexpected rotation in transform" << std::endl;
  }

  bool translationValid = true;
  for (int i = 0; i < 3; i++)
  {
        if (!Equal(tmp->GetElement(i,3), posRollPitchYaw[i]))
        {
          std::cerr << "For coord " << i
                    << std::setprecision(17)
                    << " got: " << tmp->GetElement(i,3)
                    << " expecting: " << posRollPitchYaw[i] << std::endl;
          translationValid = false;
        }
  }
  if (!translationValid)
  {
    std::cerr << "Unexpected translation in transform" << std::endl;
  }

  return (rotationValid && translationValid);
}


bool test_interpolator_transform_count(
                vtkSmartPointer<vtkVelodyneHDLPositionReader> reader,
                /* expected: */
                int count)
{
  std::cout << "Testing number of transforms in interpolator" << std::endl;
  bool isvalid = (reader->GetInterpolator()->GetNumberOfTransforms() == count);
  if (!isvalid)
  {
    std::cerr << "unexpected number of transforms" << std::endl;
  }
  return isvalid;
}

bool test_point_coords(vtkSmartPointer<vtkVelodyneHDLPositionReader> reader,
                int id,
                /* expected: */
                double coords[3])
{
  std::cout << "Testing coordinates of point: " << id << std::endl;
  bool isvalid = true;
  // check the point value in all 3 coordinates
  double coordsRead[3];
  reader->GetOutput()->GetPoint(id, coordsRead);
  for (int i = 0; i < 3; i++)
  {
    isvalid &= Equal(coordsRead[i], coords[i]);
  }
  if (!isvalid)
  {
    std::cerr << "unexpected value for coordinates of point " << id
              << std::endl;
  }
  return isvalid;
}

bool test_point_arrays(vtkSmartPointer<vtkVelodyneHDLPositionReader> reader,
                int id,
                /* expected: */
                double arrayValues[17])
{
  std::cout << "Testing arrays of point: " << id << std::endl;
  bool isvalid = true;
  // check the point value in all 17 arrays
  for (int i = 0; i < 17; i++)
  {
    if (!Equal(reader->GetOutput()->GetPointData()->GetArray(arrayNames[i])
               ->GetTuple1(id),
               arrayValues[i]))
    {
      std::cerr << "unexpected value for point " << id
                << " in array "
                << reader->GetOutput()->GetPointData()->GetArrayName(i)
                << std::endl;
      isvalid = false;
    }
  }

  if (!isvalid)
  {
    std::cout << "values read for point " << id << " are: " << std::endl;
    std::cout << "(in the order: ";
    for (int i = 0; i < 17; i++)
    {
      std::cout << arrayNames[i] << " ";
    }
    std::cout << ")" << std::endl;
    for (int i = 0; i < 17; i++)
    {
      std::cout << std::setprecision(17)
                << reader->GetOutput()->GetPointData()->GetArray(i)->GetTuple1(id);
      if (i < 16)
      {
        std::cout << ", ";
      }
    }
    std::cout << std::endl;
  }

  return isvalid;
}

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    std::cerr << "Wrong number of arguments. Usage: "
              << argv[0]
              << "<path to "
              << "\"HDL32-V2_R into Butterfield into Digital Drive.pcap\""
              << "(from data.kitware.com,"
              << "sha1sum: 1edb06c8c4312cb259dd0417a226c235945ff5b6) >"
              << std::endl;
    return 1;
  }

  std::string pathToPcap = std::string(argv[1]);

  vtkSmartPointer<vtkVelodyneHDLPositionReader> reader =
      vtkSmartPointer<vtkVelodyneHDLPositionReader>::New();
  reader->SetFileName(pathToPcap);
  reader->Update();
  // std::cout << *reader->GetOutput() << std::endl; // should you want to inspect the polydata produced

  double point0_arrays[17] =  { 37.139071666666, -121.657165, 78376, 2777073776, 0.9768, 0.108669, 0.970695, -0.272283, -0.068376, -0.272283, 3.12512, -7.42216, -32.52078, 40.6, 36.9146, 38.6582, 42.0001 };
  double point0_coords[3] =  { 0.0, 0.0, 0.0 };
  double point8947_arrays[17] = { 37.1387, -121.65492833333, 78421, 2822076651, 0.990231, 0.023199, 0.98901, -0.002442, 0.001221, -0.003663, -8.7894, -4.10172, -28.3214, 257.7, 36.9146, 38.8035, 42.0001 };
  double point8947_coords[3] =  { 199.2462615966796875,	-38.420330047607421875,	0.0 };
  int numberOfPoints = 8948;
  int numberOfTransforms = 46;
  double minTime = 78376.0;
  double maxTime = 78421.0;
  double transformMinTime[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 40.6 };
  double transformMaxTimeMinus1[6] = { 203.09254455566406, -38.180858612060547, 0.0, 0.0, 0.0, -92.3 };

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  reader->GetInterpolator()->InterpolateTransform(maxTime - 1.0, transform);

  bool isvalid = true;
  isvalid &= test_point_count(reader, numberOfPoints);
  isvalid &= test_point_coords(reader, 0, point0_coords);
  isvalid &= test_point_arrays(reader, 0, point0_arrays);
  isvalid &= test_point_coords(reader, 8947, point8947_coords);
  isvalid &= test_point_arrays(reader, 8947, point8947_arrays);

  isvalid &= test_interpolator_transform_count(reader, numberOfTransforms);
  isvalid &= test_interpolator_time_range(reader, minTime, maxTime);
  isvalid &= test_interpolator_transform(reader, minTime, transformMinTime);
  isvalid &= test_interpolator_transform(reader, maxTime - 1.0, transformMaxTimeMinus1);

  return  isvalid ? 0 : 1;
}
