#ifndef VTKLIDARREADERINTERNAL_H
#define VTKLIDARREADERINTERNAL_H

//#include "vtkLidarSource.h"
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkTransform.h>

#include <vector>
#include "vtkLidarProviderInternal.h"
#include "vtkPacketFileReader.h"

class vtkLidarReader;
//-----------------------------------------------------------------------------
class vtkLidarReaderInternal : public vtkLidarProviderInternal
{
public:
  vtkLidarReaderInternal(vtkLidarReader* obj);

  std::string GetFileName();
  virtual void SetFileName(const std::string& filename);

  void Open();
  void Close();

  int GetNumberOfFrames() override;

  bool shouldBeCroppedOut(double pos[3], double theta);

  virtual void UnloadPerFrameData() = 0;

  virtual void SplitFrame(bool force) = 0;

  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrames);

  virtual void ProcessPacket(unsigned char* data, std::size_t bytesReceived) = 0;

  virtual void SaveFrame(int startFrame, int endFrame, const std::string& filename);

  virtual bool IsLidarPacket(const unsigned char*& data, unsigned int& dataLength,
                        pcap_pkthdr** headerReference, unsigned int* dataHeaderLength) = 0;

  virtual int CountNewFrameInPacket(const unsigned char*& data, unsigned int& dataLength,
                                    pcap_pkthdr** headerReference, unsigned int* dataHeaderLength) = 0;
public:
  std::string FileName;
  std::vector<fpos_t> FilePositions;
  vtkPacketFileReader* Reader;

  // Number of allowed split, for frame-range retrieval.
  int SplitCounter;

  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;
//  vtkSmartPointer<vtkPolyData> CurrentDataset;

//  vtkSmartPointer<vtkPoints> Points;
//  vtkSmartPointer<vtkDoubleArray> PointsX;
//  vtkSmartPointer<vtkDoubleArray> PointsY;
//  vtkSmartPointer<vtkDoubleArray> PointsZ;
//  vtkSmartPointer<vtkUnsignedCharArray> Intensity;
};

#endif // VTKLIDARREADERINTERNAL_H
