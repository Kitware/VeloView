#include "vtkLidarReaderInternal.h"

#include "vtkLidarReader.h"
#include "vtkPacketFileReader.h"

//-----------------------------------------------------------------------------
vtkLidarReaderInternal::vtkLidarReaderInternal(vtkLidarReader* obj)
  : vtkLidarProviderInternal(obj)
{
  this->Lidar = obj;
  this->Reader = nullptr;
  this->FileName = "";
  this->SplitCounter = 0;
}

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::Open()
{
  this->Close();
  this->Reader = new vtkPacketFileReader;
  if (!this->Reader->Open(this->FileName))
  {
    vtkErrorWithObjectMacro(this->Lidar, "Failed to open packet file: " << this->FileName << endl
                                                 << this->Reader->GetLastError());
    this->Close();
  }
}

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::Close()
{
  delete this->Reader;
  this->Reader = 0;
}

//-----------------------------------------------------------------------------
bool vtkLidarReaderInternal::shouldBeCroppedOut(double pos[3], double theta)
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
