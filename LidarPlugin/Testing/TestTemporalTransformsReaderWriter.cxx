#include "vtkTemporalTransformsReader.h"
#include "vtkTemporalTransformsWriter.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>

#include "vtkPolyData.h"

#include "TestHelpers.h"

const double epsilon = 1e-6;

bool file_exists(const char *path)
{
    std::ifstream infile(path);
    return infile.good();
}

bool check_mm04_orbslam2_no_loop_closure(char* path)
{
  auto reader = vtkSmartPointer<vtkTemporalTransformsReader>::New();
  reader->SetFileName(path);
  reader->Update();
  vtkPolyData* read = reader->GetOutput();

  bool allgood = true;
  allgood &= read->GetNumberOfCells() == 1;
  allgood &= read->GetNumberOfPoints() == 242;
  if (!allgood) {
    // no need to go further (else we can now ask for point 241)
    return 0;
  }

  vtkDoubleArray* points = vtkDoubleArray::SafeDownCast(read->GetPoints()->GetData());
  allgood &= points != nullptr;
  double* lastPointXYZ = points->GetTuple3(241);
  const double referenceLastPointXYZ[3] = { 0.1678862, -0.0361966, -0.0566818 };
  allgood &= compare(lastPointXYZ, referenceLastPointXYZ, 3, epsilon);

  vtkDoubleArray* time = vtkDoubleArray::SafeDownCast(read->GetPointData()->GetArray("Time"));
  allgood &= time != nullptr;
  double lastTime = time->GetTuple1(241);
  const double referenceTime = 136.457;
  allgood &= compare(&lastTime, &referenceTime, 1, epsilon);

  vtkDoubleArray* orientation = vtkDoubleArray::SafeDownCast(read->GetPointData()->GetArray("Orientation(AxisAngle)"));
  allgood &= orientation != nullptr;
  double* lastOrientation = orientation->GetTuple4(241);
  const double referenceLastOrientation[4] = { 0.711088177611299, 0.335114389446281, 0.618103510463650, 0.022307328124517 };
  allgood &= compare(lastOrientation, referenceLastOrientation, 4, epsilon);

  return allgood;
}


bool read_write_trajectory(char* to_read, char* to_write)
{
  auto reader = vtkSmartPointer<vtkTemporalTransformsReader>::New();
  reader->SetFileName(to_read);
  reader->Update();

  auto writer = vtkSmartPointer<vtkTemporalTransformsWriter>::New();
  writer->SetInputConnection(0, reader->GetOutputPort());
  writer->SetFileName(to_write);
  writer->Update();
  return 1;
}

int main(int argc, char* argv[])
{
  if (argc != 3)
  {
    std::cerr << "Wrong number of arguments. Usage: " << argv[0]
                 << " <path to mm04/orbslam2-no-loop-closure.csv>"
                 << " <path to a writable file that will be destroyed>"
              << std::endl;
    return 1;
  }

  char* referenceTrajectory = argv[1];
  char* temporaryFile = argv[2]; // ! will be deleted

  // First, check that the reader works:
  if (! check_mm04_orbslam2_no_loop_closure(referenceTrajectory))
  {
    std::cout << "Reading trajectory using vtkTemporalTransformsReader"
              << " does not seem to work" << std::endl;
    return 1;
  }

  // Then prepare to test the writer
  if (file_exists(temporaryFile))
  {
   const int result = std::remove(temporaryFile);
   if (result != 0)
   {
     std::cout << "Failed to delete existing file: " << temporaryFile
               << " cannot test writer" << std::endl;
     return 1;
   }
  }

  // Then test the writer
  read_write_trajectory(referenceTrajectory, temporaryFile);
  if (!file_exists(temporaryFile))
  {
    std::cout << "Failed to produce file using writer: " << temporaryFile << std::endl;
    return 1;
  }

  if (! check_mm04_orbslam2_no_loop_closure(temporaryFile))
  {
    std::cout << "Reading file written using vtkTemporalTransformsWriter"
                 " does not seem to work" << std::endl;
    return 1;
  }

  const int result = std::remove(temporaryFile);
  if (result != 0)
  {
    std::cout << "Failed to cleanup by deleting existing file: "
              << temporaryFile << std::endl;
    return 1;
  }

  return 0;
}
