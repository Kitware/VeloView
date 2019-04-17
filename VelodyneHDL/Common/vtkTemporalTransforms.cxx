#include "vtkTemporalTransforms.h"
#include "vtkEigenTools.h"
#include "vtkConversions.h"

#include <vtkCellData.h>
#include <vtkCell.h>
#include <vtkPolyLine.h>
#include <vtkLine.h>
#include <vtkTransform.h>

#include <cmath>

// Eigen
#include <Eigen/Dense>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkTemporalTransforms)

//-----------------------------------------------------------------------------
vtkTemporalTransforms::vtkTemporalTransforms()
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();

  auto timeArray = vtkSmartPointer<vtkDoubleArray>::New();
  timeArray->SetName(this->TimeArrayName);

  auto orientationArray = vtkSmartPointer<vtkDoubleArray>::New();
  orientationArray->SetName(this->OrientationArrayName);
  orientationArray->SetNumberOfComponents(4);

  this->SetPoints(points);
  this->GetPointData()->AddArray(timeArray);
  this->GetPointData()->AddArray(orientationArray);

  // create the cell in the same time for visualization
  auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(0);
  auto cell = vtkSmartPointer<vtkCellArray>::New();
  cell->InsertNextCell(polyLine);
  this->SetLines(cell);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::CreateFromPolyData(vtkPolyData *poly)
{
  if (!poly)
  {
    return nullptr;
  }
  // copy the data into a vtkTemporalTransforms before checking that it is well-formed
  auto temporalTransforms = vtkSmartPointer<vtkTemporalTransforms>::New();
  temporalTransforms->ShallowCopy(poly);

  bool isWellFormed = temporalTransforms->GetTimeArray() &&
                      temporalTransforms->GetTranslationArray() &&
                      temporalTransforms->GetOrientationArray();
  if(!isWellFormed)
  {
    return nullptr;
  }

  // create polyline if needed
  if (temporalTransforms->GetLines()->GetNumberOfCells() == 0)
  {
    // create the cell in the same time for visualization
    auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(temporalTransforms->GetNumberOfPoints());
    for (vtkIdType i = 0; i < temporalTransforms->GetNumberOfPoints(); i++)
    {
      polyLine->GetPointIds()->SetId(i,i);
    }
    auto cell = vtkSmartPointer<vtkCellArray>::New();
    cell->InsertNextCell(polyLine);
    temporalTransforms->SetLines(cell);
  }

  return temporalTransforms;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkTransform> vtkTemporalTransforms::GetTransform(unsigned int transformNumber)
{
  auto axisAngle = this->GetOrientationArray();
  auto transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();
  double x = axisAngle->GetTuple4(transformNumber)[0];
  double y = axisAngle->GetTuple4(transformNumber)[1];
  double z = axisAngle->GetTuple4(transformNumber)[2];
  double w = axisAngle->GetTuple4(transformNumber)[3]
      * 180 / vtkMath::Pi(); // vtk need degrees not radians
  transform->RotateWXYZ(w, x, y, z);
  transform->Translate(this->GetTranslationArray()->GetTuple(transformNumber));

  return transform;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkCustomTransformInterpolator> vtkTemporalTransforms::CreateInterpolator()
{
  auto interpolator = vtkSmartPointer<vtkCustomTransformInterpolator>::New();

  auto timestamp = this->GetTimeArray();

  for (unsigned int i = 0; i < this->GetNumberOfPoints(); i++)
  {
    // get timestamp
    double currentTimestamp = timestamp->GetTuple1(i);

    // create the tranform
    auto transform = this->GetTransform(i);

    // add the tranform to the interpolator
    if (!vtkMath::IsNan(currentTimestamp))
    {
      interpolator->AddTransform(currentTimestamp, transform);
    }
    else
    {
      vtkErrorMacro("Timestamp " << i << "is not a number")
    }
  }
  interpolator->Modified();
  return interpolator;
}

//-----------------------------------------------------------------------------
void vtkTemporalTransforms::SetOrientationArray(vtkDoubleArray *array)
{
  if (array->GetNumberOfComponents() != 4)
  {
    vtkErrorMacro(<< "The orientation array has " << array->GetNumberOfComponents()
                  << ". 4 components are expected")
  }
  array->SetName(this->OrientationArrayName);
  this->GetPointData()->AddArray(array);
}

//-----------------------------------------------------------------------------
void vtkTemporalTransforms::SetTranslationArray(vtkDataArray *array)
{
  if (array->GetNumberOfComponents() != 3)
  {
    vtkErrorMacro(<< "The translation array has " << array->GetNumberOfComponents()
                  << ". 3 components are expected")
  }
  auto points =  vtkSmartPointer<vtkPoints>::New();
  this->SetPoints(points);
  this->GetPoints()->SetData(array);

  // create the cell in the same time for visualization
  auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(this->GetNumberOfPoints());
  for (vtkIdType i = 0; i < array->GetNumberOfTuples(); i++)
  {
    polyLine->GetPointIds()->SetId(i,i);
  }
  auto cell = vtkSmartPointer<vtkCellArray>::New();
  cell->InsertNextCell(polyLine);
  this->SetLines(cell);
}

//-----------------------------------------------------------------------------
void vtkTemporalTransforms::SetTimeArray(vtkDoubleArray *array)
{
  if (array->GetNumberOfComponents() != 1)
  {
    vtkErrorMacro(<< "The time array has " << array->GetNumberOfComponents()
                  << ". 1 component is expected")
  }
  array->SetName(this->TimeArrayName);
  this->GetPointData()->AddArray(array);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::IsometricTransform(vtkSmartPointer<vtkTransform> H)
{
  vtkSmartPointer<vtkTemporalTransforms> outputPoses = vtkSmartPointer<vtkTemporalTransforms>::New();
  outputPoses->DeepCopy(this);

  // First, compute the rotation and translation from H 4x4 matrix
  std::pair<Eigen::Vector3d, Eigen::Vector3d> transParams = GetPoseParamsFromTransform(H);
  Eigen::Matrix3d R0 = RollPitchYawToMatrix(transParams.first);
  Eigen::Vector3d T0 = transParams.second;

  // We need to change the angle axis and position arrays
  vtkDoubleArray* xyzwArray = vtkDoubleArray::SafeDownCast(outputPoses->GetOrientationArray());
  vtkDoubleArray* xyzArray = vtkDoubleArray::SafeDownCast(outputPoses->GetTranslationArray());

  // Loop over the transforms points
  for (unsigned int transformIndex = 0; transformIndex < outputPoses->GetNumberOfPoints(); transformIndex++)
  {
    // Get the angle-axis representation
    double* xyzw = xyzwArray->GetTuple4(transformIndex);
    double* xyz = xyzArray->GetTuple3(transformIndex);

    // Compute the corresponding rotation matrix that
    // correspond to the angle axis representation
    Eigen::Vector3d T(xyz[0], xyz[1], xyz[2]);
    Eigen::AngleAxisd angleAxis(xyzw[3], Eigen::Vector3d(xyzw[0], xyzw[1], xyzw[2]));
    Eigen::Matrix3d R = angleAxis.toRotationMatrix();

    // Compute the new rotation and translation
    R = R0 * R;
    T = R0 * T + T0;

    // Get the axis angle representation of this new rotation
    Eigen::AngleAxisd newAngleAxis(R);
    double newXyzw[4] = {newAngleAxis.axis()(0), newAngleAxis.axis()(1),
                         newAngleAxis.axis()(2), newAngleAxis.angle()};

    // Replace it
    xyzwArray->SetTuple4(transformIndex, newXyzw[0], newXyzw[1], newXyzw[2], newXyzw[3]);
    xyzArray->SetTuple3(transformIndex, T(0), T(1), T(2));
  }
  return outputPoses;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::CycloidicTransform(vtkSmartPointer<vtkTransform> H)
{
  vtkSmartPointer<vtkTemporalTransforms> outputPoses = vtkSmartPointer<vtkTemporalTransforms>::New();
  outputPoses->DeepCopy(this);

  // First, compute the rotation and translation from H 4x4 matrix
  std::pair<Eigen::Vector3d, Eigen::Vector3d> transParams = GetPoseParamsFromTransform(H);
  Eigen::Matrix3d R0 = RollPitchYawToMatrix(transParams.first);
  Eigen::Vector3d T0 = transParams.second;

  // We need to change the angle axis and position arrays
  vtkDoubleArray* xyzwArray = vtkDoubleArray::SafeDownCast(this->GetOrientationArray());
  vtkDoubleArray* xyzArray = vtkDoubleArray::SafeDownCast(this->GetTranslationArray());

  // Loop over the transforms points
  for (unsigned int transformIndex = 0; transformIndex < this->GetNumberOfPoints(); transformIndex++)
  {
    // Get the angle-axis representation
    double* xyzw = xyzwArray->GetTuple4(transformIndex);
    double* xyz = xyzArray->GetTuple3(transformIndex);

    // Compute the corresponding rotation matrix that
    // correspond to the angle axis representation
    Eigen::Vector3d T(xyz[0], xyz[1], xyz[2]);
    Eigen::AngleAxisd angleAxis(xyzw[3], Eigen::Vector3d(xyzw[0], xyzw[1], xyzw[2]));
    Eigen::Matrix3d R = angleAxis.toRotationMatrix();

    // Compute the new rotation and translation
    T = R * T0 + T;
    R = R * R0;

    // Get the axis angle representation of this new rotation
    Eigen::AngleAxisd newAngleAxis(R);
    double newXyzw[4] = {newAngleAxis.axis()(0), newAngleAxis.axis()(1),
                         newAngleAxis.axis()(2), newAngleAxis.angle()};

    // Replace it
    xyzwArray->SetTuple4(transformIndex, newXyzw[0], newXyzw[1], newXyzw[2], newXyzw[3]);
    xyzArray->SetTuple3(transformIndex, T(0), T(1), T(2));
  }
  return outputPoses;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::MLSSmoothing(int polDeg, int kernelRadius)
{
  auto outputPoses = vtkSmartPointer<vtkTemporalTransforms>::New();
  outputPoses->DeepCopy(this);

  // We need to change the angle axis and position arrays
  vtkDoubleArray* anglesAxisArray = vtkDoubleArray::SafeDownCast(this->GetOrientationArray());
  vtkDoubleArray* xyzArray = vtkDoubleArray::SafeDownCast(this->GetTranslationArray());

  // First, convert the positions coordinates and the orientation
  // using Eigen container
  std::vector<Eigen::VectorXd> X, Y;
  std::vector<Eigen::VectorXd> Qin, Qout;
  for (int k = 0; k < outputPoses->GetNumberOfPoints(); ++k)
  {
    Eigen::VectorXd x(3, 1);
    Eigen::VectorXd q(4, 1);
    double currPos[3];
    // Positions
    outputPoses->GetPoint(k, currPos);
    x << currPos[0], currPos[1], currPos[2];
    X.push_back(x);

    // Orientations
    double* xyzw = anglesAxisArray->GetTuple4(k);
    Eigen::AngleAxisd angleAxis(xyzw[3], Eigen::Vector3d(xyzw[0], xyzw[1], xyzw[2]));
    Eigen::Quaterniond quat(angleAxis);
    q << quat.w(), quat.x(), quat.y(), quat.z();
    Qin.push_back(q);
  }

  // Smooth the trajectory
  EuclideanMLSSmoothing(X, Y, polDeg, kernelRadius);

  // Smooth the orientations using an euclidean
  // MLS algorithm and then reprojecting the points
  // onto the unit quaternion sphere
  EuclideanMLSSmoothing(Qin, Qout, polDeg, kernelRadius);

  // Set the points
  for (int k = 0; k < outputPoses->GetNumberOfPoints(); ++k)
  {
    // Positions
    double pos[3] = {Y[k](0), Y[k](1), Y[k](2)};
    outputPoses->GetPoints()->SetPoint(k, pos);
    xyzArray->SetTuple3(k, pos[0], pos[1], pos[2]);

    // Orientations
    Eigen::Quaterniond quat(Qout[k](0), Qout[k](1), Qout[k](2), Qout[k](3));
    quat.normalize();
    Eigen::AngleAxisd axisAngle(quat);
    double xyzw[4] = {axisAngle.axis()(0), axisAngle.axis()(1), axisAngle.axis()(2), axisAngle.angle()};
    anglesAxisArray->SetTuple4(k, xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
  }
  return outputPoses;
}

//-----------------------------------------------------------------------------
void vtkTemporalTransforms::PushBack(double time, const Eigen::AngleAxisd& orientation , const Eigen::Vector3d translation)
{
  this->GetTimeArray()->InsertNextTuple1(time);
  this->GetOrientationArray()->InsertNextTuple4(orientation.axis()[0],
                                                orientation.axis()[1],
                                                orientation.axis()[2],
                                                orientation.angle());
  this->GetTranslationArray()->InsertNextTuple(static_cast<const double*>(translation.data()));


  if (this->GetNumberOfPoints() == 1)
  {
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    this->SetLines(lines);
    return; // not enough points (1) to create a new line
  }

  // if control reaches this line, we have at least two points,
  // we append one small line between the new point and the previous one
  vtkSmartPointer<vtkCellArray> lines = this->GetLines();
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  line->GetPointIds()->SetId(0, this->GetNumberOfPoints() - 2);
  line->GetPointIds()->SetId(1, this->GetNumberOfPoints() - 1);
  lines->InsertNextCell(line);
}

vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::ExtractTimes(double tstart, double tend)
{
  auto extract = vtkSmartPointer<vtkTemporalTransforms>::New();

  for (unsigned int i = 0; i < this->GetNumberOfPoints(); i++)
  {
    // get timestamp
    double currentTimestamp = this->GetTimeArray()->GetTuple1(i);
    if (currentTimestamp < tstart || currentTimestamp > tend) {
      continue;
    }

    extract->GetTimeArray()->InsertNextTuple1(currentTimestamp);
    double* aa = this->GetOrientationArray()->GetTuple4(i);
    extract->GetOrientationArray()->InsertNextTuple(aa);
    double* xyz = this->GetTranslationArray()->GetTuple3(i);
    extract->GetTranslationArray()->InsertNextTuple(xyz);
  }

  return extract;
}

vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::Subsample(int N)
{

  auto extract = vtkSmartPointer<vtkTemporalTransforms>::New();

  for (unsigned int i = 0; i < this->GetNumberOfPoints(); i++)
  {
    if (i % N != 0)
    {
      continue;
    }
    // get timestamp
    double currentTimestamp = this->GetTimeArray()->GetTuple1(i);
    extract->GetTimeArray()->InsertNextTuple1(currentTimestamp);
    double* aa = this->GetOrientationArray()->GetTuple4(i);
    extract->GetOrientationArray()->InsertNextTuple(aa);
    double* xyz = this->GetTranslationArray()->GetTuple3(i);
    extract->GetTranslationArray()->InsertNextTuple(xyz);
  }

  auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(extract->GetNumberOfPoints());
  for (vtkIdType i = 0; i < extract->GetNumberOfPoints(); i++)
  {
    polyLine->GetPointIds()->SetId(i,i);
  }
  auto cell = vtkSmartPointer<vtkCellArray>::New();
  cell->InsertNextCell(polyLine);
  extract->SetLines(cell);

  return extract;
}

vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::ApplyTimeshift(double shift)
{
  auto timeshifted = vtkSmartPointer<vtkTemporalTransforms>::New();

  for (unsigned int i = 0; i < this->GetNumberOfPoints(); i++)
  {
    double currentTimestamp = this->GetTimeArray()->GetTuple1(i);
    timeshifted->GetTimeArray()->InsertNextTuple1(currentTimestamp + shift);
    double* aa = this->GetOrientationArray()->GetTuple4(i);
    timeshifted->GetOrientationArray()->InsertNextTuple(aa);
    double* xyz = this->GetTranslationArray()->GetTuple3(i);
    timeshifted->GetTranslationArray()->InsertNextTuple(xyz);
  }

  auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(timeshifted->GetNumberOfPoints());
  for (vtkIdType i = 0; i < timeshifted->GetNumberOfPoints(); i++)
  {
    polyLine->GetPointIds()->SetId(i,i);
  }
  auto cell = vtkSmartPointer<vtkCellArray>::New();
  cell->InsertNextCell(polyLine);
  timeshifted->SetLines(cell);

  bool isWellFormed = timeshifted->GetTimeArray() &&
                      timeshifted->GetTranslationArray() &&
                      timeshifted->GetOrientationArray();
  if(!isWellFormed)
  {
    std::cout << "warning ! timeshifted not well formed" << std::endl;
  }

  return timeshifted;
}


vtkSmartPointer<vtkTemporalTransforms> vtkTemporalTransforms::ApplyScale(double scale)
{
  auto scaled = vtkSmartPointer<vtkTemporalTransforms>::New();

  for (unsigned int i = 0; i < this->GetNumberOfPoints(); i++)
  {
    double currentTimestamp = this->GetTimeArray()->GetTuple1(i);
    scaled->GetTimeArray()->InsertNextTuple1(currentTimestamp);
    double* aa = this->GetOrientationArray()->GetTuple4(i);
    scaled->GetOrientationArray()->InsertNextTuple(aa);
    double* xyz = this->GetTranslationArray()->GetTuple3(i);
    xyz[0] *= scale;
    xyz[1] *= scale;
    xyz[2] *= scale;
    scaled->GetTranslationArray()->InsertNextTuple(xyz);
  }

  auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(scaled->GetNumberOfPoints());
  for (vtkIdType i = 0; i < scaled->GetNumberOfPoints(); i++)
  {
    polyLine->GetPointIds()->SetId(i,i);
  }
  auto cell = vtkSmartPointer<vtkCellArray>::New();
  cell->InsertNextCell(polyLine);
  scaled->SetLines(cell);

  bool isWellFormed = scaled->GetTimeArray() && scaled->GetTranslationArray() &&
                      scaled->GetOrientationArray();
  if(!isWellFormed)
  {
    std::cout << "warning ! scaled not well formed" << std::endl;
  }

  return scaled;
}
