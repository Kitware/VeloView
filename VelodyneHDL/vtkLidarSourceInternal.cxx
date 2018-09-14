#include "vtkLidarSourceInternal.h"

//-----------------------------------------------------------------------------
vtkLidarSourceInternal::vtkLidarSourceInternal()
{
//    this->RpmCalculator.Reset();
//    this->AlreadyWarnAboutCalibration = false;
  this->IgnoreZeroDistances = true;
  this->CropMode = Cartesian;
//    this->OutputPacketProcessingDebugInfo = false;
//    this->SensorPowerMode = 0;
    this->Skip = 0;
//    this->CurrentFrameState = new FramingState;
//    this->LastTimestamp = std::numeric_limits<unsigned int>::max();
//    this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
//    this->Reader = 0;
  this->SplitCounter = 0;
  this->NumberOfTrailingFrames = 0;
  this->ApplyTransform = false;
  this->CropReturns = false;
  this->CropOutside = false;
  this->CropRegion[0] = this->CropRegion[1] = 0.0;
  this->CropRegion[2] = this->CropRegion[3] = 0.0;
  this->CropRegion[4] = this->CropRegion[5] = 0.0;
  this->CorrectionsInitialized = false;
  this->currentRpm = 0;

//    std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);

  this->CalibrationReportedNumLasers = -1;
  this->IgnoreEmptyFrames = true;
//    this->distanceResolutionM = 0.002;
//    this->WantIntensityCorrection = false;
}

//-----------------------------------------------------------------------------
bool vtkLidarSourceInternal::shouldBeCroppedOut(double pos[3], double theta)
{
  // Test if point is cropped
  if (!this->CropReturns)
  {
    return false;
  }
  switch (this->CropMode)
  {
    case Cartesian: // Cartesian cropping mode
    {
      bool pointOutsideOfBox = pos[0] >= this->CropRegion[0] && pos[0] <= this->CropRegion[1] &&
        pos[1] >= this->CropRegion[2] && pos[1] <= this->CropRegion[3] &&
        pos[2] >= this->CropRegion[4] && pos[2] <= this->CropRegion[5];
      return (
        (pointOutsideOfBox && this->CropOutside) || (!pointOutsideOfBox && !this->CropOutside));
      break;
    }
    case Spherical:
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
    case Cylindric:
    {
      // space holder for future implementation
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkLidarSourceInternal::GetFrame(int frameNumber, int wantedNumberOfTrailingFrames)
{
  this->UnloadPerFrameData();
  if (!this->Reader)
  {
//    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
  }
  if (!this->CorrectionsInitialized)
  {
//    vtkErrorMacro("Corrections have not been set");
    return 0;
  }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  this->Reader->SetFilePosition(&this->FilePositions[frameNumber]);
  this->SplitCounter = wantedNumberOfTrailingFrames;

  while (this->Reader->NextPacket(data, dataLength, timeSinceStart))
  {
    this->ProcessPacket(const_cast<unsigned char*>(data), dataLength);

    if (this->Datasets.size())
    {
      this->SplitCounter = 0;
      return this->Datasets.back();
    }
  }

  this->SplitFrame(true);
  this->SplitCounter = 0;
  return this->Datasets.back();
}
