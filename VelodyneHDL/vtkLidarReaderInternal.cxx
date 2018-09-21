#include "vtkLidarReaderInternal.h"

#include "vtkLidarReader.h"
#include "vtkPacketFileWriter.h"

//-----------------------------------------------------------------------------
vtkLidarReaderInternal::vtkLidarReaderInternal(vtkLidarReader* obj)
  : vtkLidarProviderInternal(obj)
{
  this->Reader = nullptr;
  this->FileName = "";
  this->SplitCounter = 0;
}

//-----------------------------------------------------------------------------
std::string vtkLidarReaderInternal::GetFileName()
{
  return this->FileName;
}

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::SetFileName(const std::string &filename)
{
  if (filename == this->FileName)
  {
    return;
  }

  this->FileName = filename;
  this->FilePositions.clear();
  this->UnloadPerFrameData();
  this->Lidar->Modified();
}

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::Open()
{
  this->Close();
  this->Reader = new vtkPacketFileReader;
  if (!this->Reader->Open(this->FileName))
  {
//    vtkErrorMacro("Failed to open packet file: " << this->FileName << endl
//                                                 << this->Reader->GetLastError());
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
vtkSmartPointer<vtkPolyData> vtkLidarReaderInternal::GetFrame(int frameNumber, int wantedNumberOfTrailingFrames)
{
  this->UnloadPerFrameData();
  if (!this->Reader)
  {
//    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
  }
  if (!this->IsCalibrated)
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

//-----------------------------------------------------------------------------
void vtkLidarReaderInternal::SaveFrame(int startFrame, int endFrame, const std::string &filename)
{
  if (!this->Reader)
  {
//    vtkErrorMacro("DumpFrames() called but packet file reader is not open.");
    return;
  }

  vtkPacketFileWriter writer;
  if (!writer.Open(filename))
  {
//    vtkErrorMacro("Failed to open packet file for writing: " << filename);
    return;
  }

  pcap_pkthdr* header = 0;
  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  unsigned int dataHeaderLength = 0;
  double timeSinceStart = 0;

  int currentFrame = startFrame;

  this->Reader->SetFilePosition(&this->FilePositions[startFrame]);
  while (this->Reader->NextPacket(
           data, dataLength, timeSinceStart, &header, &dataHeaderLength) &&
    currentFrame <= endFrame)
  {
    if (this->IsLidarPacket(data, dataLength, &header, &dataHeaderLength))
    {
      writer.WritePacket(header, const_cast<unsigned char*>(data) - dataHeaderLength);

      currentFrame += this->CountNewFrameInPacket(data, dataLength, &header, &dataHeaderLength);
    }
  }
  writer.Close();
}

//-----------------------------------------------------------------------------
int vtkLidarReaderInternal::GetNumberOfFrames()
{
  return this->FilePositions.size();
}
