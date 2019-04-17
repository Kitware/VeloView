#include "vtkTemporalTransformsRemapper.h"

#include <vtkTemporalTransforms.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkInformationVector.h>

#include "vtkGeometricCalibration.h"

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkTemporalTransformsRemapper)

//-----------------------------------------------------------------------------
vtkTemporalTransformsRemapper::vtkTemporalTransformsRemapper()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
int vtkTemporalTransformsRemapper::RequestData(vtkInformation *, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
{
  vtkPolyData* referencePoly = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));
  vtkPolyData* toAlignPoly = vtkPolyData::GetData(inputVector[1]->GetInformationObject(0));

  auto reference = vtkTemporalTransforms::CreateFromPolyData(referencePoly);
  auto toAlign = vtkTemporalTransforms::CreateFromPolyData(toAlignPoly);

  vtkSmartPointer<vtkTemporalTransforms> result = MatchTrajectoriesWithIsometryAndApply(reference, toAlign);

  auto *output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(result);
  return VTK_OK;
}
