#include "vtkTemporalTransforms.h"

#include <vtkCellData.h>
#include <vtkPolyLine.h>
#include <vtkTransform.h>

#include <cmath>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkTemporalTransforms)

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

  return isWellFormed ? temporalTransforms : nullptr;
}

vtkSmartPointer<vtkVelodyneTransformInterpolator> vtkTemporalTransforms::CreateInterpolator()
{
  auto interpolator = vtkSmartPointer<vtkVelodyneTransformInterpolator>::New();

  auto axisAngle = this->GetOrientationArray();
  auto timestamp = this->GetTimeArray();

  for (unsigned int i = 0; i < this->GetNumberOfPoints(); i++)
  {
    // get timestamp
    double currentTimestamp = timestamp->GetTuple1(i);

    // create the tranform
    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply();
    double x = axisAngle->GetTuple4(i)[0];
    double y = axisAngle->GetTuple4(i)[1];
    double z = axisAngle->GetTuple4(i)[2];
    double w = axisAngle->GetTuple4(i)[3] * 180 / vtkMath::Pi(); // vtk need degrees not radians
    transform->RotateWXYZ(w, x, y, z);
    transform->Translate(this->GetTranslationArray()->GetTuple(i));

    // add the tranform to the interpolator
    if (!vtkMath::IsNan(currentTimestamp))
    {
      interpolator->AddTransform(currentTimestamp, transform);
    }
    else
    {
      vtkErrorMacro(<< "Timestamp " << i << "is not a number")
    }
  }
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
