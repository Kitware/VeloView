#include "vtkTemporalTransforms.h"
#include "vtkTemporalTransformsWriter.h"

#include "vtkTransform.h"
#include "vtkNew.h"
#include <vtkObjectFactory.h>
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include <iostream>

#include "vtkConversions.h"

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkTemporalTransformsWriter)

//-----------------------------------------------------------------------------
vtkTemporalTransformsWriter::~vtkTemporalTransformsWriter()
{
  if (this->FileName)
  {
    delete[] this->FileName;
  }
}

//-----------------------------------------------------------------------------
int vtkTemporalTransformsWriter::RequestData(vtkInformation *vtkNotUsed(request),
                                             vtkInformationVector **inputVector,
                                             vtkInformationVector *vtkNotUsed(outputVector))
{
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *polyData = vtkPolyData::SafeDownCast(
  inInfo->Get(vtkDataObject::DATA_OBJECT()));

  if (polyData == nullptr)
  {
    vtkErrorMacro("Input is not a vtkPolyData");
    return 0;
  }

  auto transforms = vtkTemporalTransforms::CreateFromPolyData(polyData);
  if (transforms == nullptr)
  {
    vtkErrorMacro("Input is not a vtkTemporalTransforms");
    return 0;
  }

  vtkDataArray* time = transforms->GetTimeArray();

  std::ofstream file(this->FileName);
  // TODO: when we support reading a CSV with commented line, add such comment:
  // # Pose trajectory format, time in s, angles in degree, position in meters
  // # Recompose the rotation part of the pose using:
  // # R = Rz(yaw) * Ry(pitch) * Rx(roll)
  // # a point exprimed in the Lidar reference frame can be exprimed in a fixed
  // # reference frame using: X_fixed = R(t) * X_lidar + [x(t), y(t), z(t)]^T
  file << "Time,Rx(Roll),Ry(Pitch),Rz(Yaw),X,Y,Z" << endl;

  for (unsigned int i = 0; i < transforms->GetNumberOfPoints(); i++)
  {
    double t = time->GetTuple1(i);
    auto transform = transforms->GetTransform(i);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> towrite = GetPoseParamsFromTransform(transform);
    // We *need* to take care than the user might want to export a trajectory
    // that was produced by projecting GPS coordinate into UTM, thus giving huge
    // coordinates (thousands of kilometers).
    // So make sure that you always show up to N decimals after the point.
    // (not just N significant digits because many digits could be before the
    // point).
    // Possible optimization: hide useless trailing zeros which take up space.
    // (at the cost of breaking the columns alignement that helps a human parse)
    file << std::fixed << std::setprecision(17)
         << t
         << "," << towrite.first[0]
         << "," << towrite.first[1]
         << "," << towrite.first[2]
         << "," << towrite.second[0]
         << "," << towrite.second[1]
         << "," << towrite.second[2]
         << std::endl;
  }

  file.close();
  return 1;
}
