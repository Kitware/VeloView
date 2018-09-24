#ifndef VTKLIDARREADERINTERNAL_H
#define VTKLIDARREADERINTERNAL_H

#include "vtkLidarProviderInternal.h"
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkTransform.h>

#include <vector>


class vtkLidarReader;
class pcap_pkthdr;
class vtkPacketFileReader;
struct vtkLidarReaderInternal : public vtkLidarProviderInternal
{
  vtkLidarReader* Lidar;

  vtkLidarReaderInternal(vtkLidarReader* obj);


  void Open();
  void Close();

  bool shouldBeCroppedOut(double pos[3], double theta);

  virtual void CheckSensorCalibrationConsistency() = 0;

  virtual void UnloadPerFrameData() = 0;

  virtual void SplitFrame(bool force) = 0;

  ///
  /// \brief ProcessPacket process a data packet and gradually construct the current frame
  /// \param data
  /// \param bytesReceived
  ///
  virtual void ProcessPacket(unsigned char* data, std::size_t bytesReceived) = 0;

  ///
  /// \brief IsLidarPacket check if the given packet is really a lidar packet
  /// \param data
  /// \param dataLength
  /// \param headerReference
  /// \param dataHeaderLength
  /// \return
  ///
  virtual bool IsLidarPacket(const unsigned char*& data, unsigned int& dataLength,
                        pcap_pkthdr** headerReference, unsigned int* dataHeaderLength) = 0;

  ///
  /// \brief CountNewFrameInPacket return how many new frame should be created with this new packet,
  /// the return should be positive (0 most of the time)
  /// \param data
  /// \param dataLength
  /// \param headerReference
  /// \param dataHeaderLength
  /// \return
  ///
  virtual int CountNewFrameInPacket(const unsigned char*& data, unsigned int& dataLength,
                                    pcap_pkthdr** headerReference, unsigned int* dataHeaderLength) = 0;
  std::string FileName;
  std::vector<fpos_t> FilePositions;
  vtkPacketFileReader* Reader;

  // Number of allowed split, for frame-range retrieval.
  int SplitCounter;

  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;

};

#endif // VTKLIDARREADERINTERNAL_H
