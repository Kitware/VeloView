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
bool vtkLidarPacketInterpreter::shouldBeCroppedOut(double pos[3], double theta)
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
      double vertAngle = std::atan2(pos[2], std::sqrt(pos[0] * pos[0] + pos[1] * pos[1]));
      vertAngle *= 180.0 / vtkMath::Pi();

      pointInside = theta >= this->CropRegion[0] && theta <= this->CropRegion[1];
      pointInside &= theta >= this->CropRegion[0] && theta <= this->CropRegion[1];
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
