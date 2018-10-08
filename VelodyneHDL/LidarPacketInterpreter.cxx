#include "LidarPacketInterpreter.h"
#include "vtkLidarProvider.h"

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
LidarPacketInterpreter::LidarPacketInterpreter()
{
  this->NumberOfTrailingFrames = 0;

  this->CalibrationFileName = "";
  this->CalibrationReportedNumLasers = -1;
  this->IsCalibrated = false;

  this->Frequency = 0;
  this->DistanceResolutionM = 0;

  this->IgnoreZeroDistances = true;
  this->IgnoreEmptyFrames = true;

  this->ApplyTransform = false;

  // Cropping
  this->CropMode = 1/*vtkLidarProvider::Cartesian*/;
  this->CropReturns = false;
  this->CropOutside = false;
  this->CropRegion[0] = 0;
  this->CropRegion[1] = 0;
  this->CropRegion[2] = 0;
  this->CropRegion[3] = 0;
  this->CropRegion[4] = 0;
  this->CropRegion[5] = 0;
}

//-----------------------------------------------------------------------------
bool LidarPacketInterpreter::SplitFrame(bool force)
{
  if (this->IgnoreEmptyFrames && this->CurrentDataset->GetNumberOfPoints() == 0 && !force)
  {
    return false;
  }

  if (this->SplitCounter > 0 && !force)
  {
    this->SplitCounter--;
    return false;
  }
  // add vertex to the polydata
  this->CurrentDataset->SetVerts(NewVertexCells(this->CurrentDataset->GetNumberOfPoints()));
  // split the frame
  this->Datasets.push_back(this->CurrentDataset);
  // create a new frame
  this->CurrentDataset = this->CreateData(0);

  return true;
}

//-----------------------------------------------------------------------------
void LidarPacketInterpreter::SetCropRegion(double region[])
{
  for (int i = 0; i < 6; i++)
  {
    this->CropRegion[i] = region[i];
  }
}

//-----------------------------------------------------------------------------
bool LidarPacketInterpreter::shouldBeCroppedOut(double pos[3], double theta)
{
  // Test if point is cropped
  if (!this->CropReturns)
  {
    return false;
  }
  switch (this->CropMode)
  {
    case vtkLidarProvider::Cartesian: // Cartesian cropping mode
    {
      bool pointOutsideOfBox = pos[0] >= this->CropRegion[0] && pos[0] <= this->CropRegion[1] &&
        pos[1] >= this->CropRegion[2] && pos[1] <= this->CropRegion[3] &&
        pos[2] >= this->CropRegion[4] && pos[2] <= this->CropRegion[5];
      return (
        (pointOutsideOfBox && this->CropOutside) || (!pointOutsideOfBox && !this->CropOutside));
      break;
    }
    case vtkLidarProvider::Spherical:
      // Spherical mode
      {
        double R = std::sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
        double vertAngle = std::atan2(pos[2], std::sqrt(pos[0] * pos[0] + pos[1] * pos[1]));
        vertAngle *= 180.0 / vtkMath::Pi();
        bool pointInsideOfBounds;
        if (this->CropRegion[0] <= this->CropRegion[1]) // 0 is NOT in theta range
        {
          pointInsideOfBounds = theta >= this->CropRegion[0] && theta <= this->CropRegion[1] &&
            R >= this->CropRegion[4] && R <= this->CropRegion[5];
        }
        else // theta range includes 0
        {
          pointInsideOfBounds = (theta >= this->CropRegion[0] || theta <= this->CropRegion[1]) &&
            R >= this->CropRegion[4] && R <= this->CropRegion[5];
        }
        pointInsideOfBounds &= (vertAngle > this->CropRegion[2] && vertAngle < this->CropRegion[3]);
        return ((pointInsideOfBounds && this->CropOutside) ||
          (!pointInsideOfBounds && !this->CropOutside));
        break;
      }
    case vtkLidarProvider::Cylindric:
    {
      // space holder for future implementation
    }
  }
  return false;
}
