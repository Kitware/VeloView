#include "vtkLidarPacketInterpreter.h"

#include <vtkTransform.h>

namespace {
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
// Returns the value that is equal to x modulo mod, and that is inside to (0, mod(
// mod must be > 0.0
double place_in_interval(double x, double mod)
{
  if (x < 0.0)
  {
    return x + std::ceil(- x / mod) * mod; // not equal to std::fmod (always >= 0)
  } else {
    return std::fmod(x, mod);
  }
}

//-----------------------------------------------------------------------------
// Returns true if x is "inside" (a, b) modulo mod
bool inside_interval_mod(double x, double a, double b, double mod)
{
  // first step: place everything in [0.0, mod]
  x = place_in_interval(x, mod);
  a = place_in_interval(a, mod);
  b = place_in_interval(b, mod);
  if (a >= b)
  {
    // [ ...in...|b|...out...|a|...in...]
    return x >= a || x <= b;
  }
  else
  {
    // [ ...out...|a|...in...|b|...out...]
    return x >= a && x <= b;
  }
}
}

//-----------------------------------------------------------------------------
bool vtkLidarPacketInterpreter::SplitFrame(bool force)
{
  const vtkIdType nPtsOfCurrentDataset= this->CurrentFrame->GetNumberOfPoints();
  if (this->IgnoreEmptyFrames && (nPtsOfCurrentDataset == 0) && !force)
  {
    return false;
  }

  // add vertex to the polydata
  this->CurrentFrame->SetVerts(NewVertexCells(this->CurrentFrame->GetNumberOfPoints()));
  // split the frame
  this->Frames.push_back(this->CurrentFrame);
  // create a new frame
  this->CurrentFrame = this->CreateNewEmptyFrame(0, nPtsOfCurrentDataset);

  return true;
}


//-----------------------------------------------------------------------------
bool vtkLidarPacketInterpreter::shouldBeCroppedOut(double pos[3])
{
  bool pointInside = true;
  switch (this->CropMode)
  {
    case CROP_MODE::Cartesian:
    {
      pointInside = pos[0] >= this->CropRegion[0] && pos[0] <= this->CropRegion[1];
      pointInside &= pos[1] >= this->CropRegion[2] && pos[1] <= this->CropRegion[3];
      pointInside &= pos[2] >= this->CropRegion[4] && pos[2] <= this->CropRegion[5];
      break;
    }
    case CROP_MODE::Spherical:
    {
      double R = std::sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
      // azimuth in [0°, 360°]
      double azimuth = 180.0 + (180.0 / vtkMath::Pi()) * std::atan2(pos[1], pos[0]);
      // vertAngle in [-90°, 90°], increasing with z
      double vertAngle = 90.0 - (180.0 / vtkMath::Pi()) * std::acos(pos[2] / R);

      pointInside = inside_interval_mod(azimuth, this->CropRegion[0], this->CropRegion[1], 360.0);
      pointInside &= vertAngle >= this->CropRegion[2] && vertAngle <= this->CropRegion[3];
      pointInside &= R >= this->CropRegion[4] && R <= this->CropRegion[5];
      break;
    }
    case CROP_MODE::Cylindric:
    {
      // space holder for future implementation
    }
    case CROP_MODE::None:
      return false;
  }
  return !((pointInside && !this->CropOutside) || (!pointInside && this->CropOutside));
}

//-----------------------------------------------------------------------------
vtkCxxSetObjectMacro(vtkLidarPacketInterpreter, SensorTransform, vtkTransform)

//-----------------------------------------------------------------------------
vtkMTimeType vtkLidarPacketInterpreter::GetMTime()
{
  if (this->SensorTransform)
  {
    return std::max(this->Superclass::GetMTime(), this->SensorTransform->GetMTime());
  }
  return this->Superclass::GetMTime();
}
