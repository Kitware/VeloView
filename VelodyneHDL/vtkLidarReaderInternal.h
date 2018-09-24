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
struct  vtkLidarReaderInternal : public vtkLidarProviderInternal
{
  //! Backwward pointer to public class interface
  vtkLidarReader* Lidar;

  vtkLidarReaderInternal(vtkLidarReader* obj);

  /**
   * @brief Open open the pcap file
   */
  void Open();

  /**
   * @brief Close close the pcap file
   */
  void Close();

  /**
   * @brief shouldBeCroppedOut Check if a point is inside a region of interest determined
   * either by its cartesian coordinates system or spherical coordinates system.
   * @param pos cartesian coordinates of the point to be check
   * @param theta azimuth of the point to be check. Avoid to recompute this information using an atan2 function
   * @return
   */
  bool shouldBeCroppedOut(double pos[3], double theta);

  /**
   * @brief CheckSensorCalibrationConsistency Check the consistency between information provided
   * by the calibration file and the ones obtained by parsing the data packets
   */
  virtual void CheckSensorCalibrationConsistency() = 0;

  /**
   * @brief UnloadPerFrameData didn't really get it yet
   */
  virtual void UnloadPerFrameData() = 0;

  /**
   * @brief SplitFrame
   * @param force
   */
  virtual void SplitFrame(bool force) = 0;

  /**
   * @brief ProcessPacket Process the data packet to create incrementaly the frame.
   * Each time a packet is processed by the function, the points which are encoded
   * in the packet are decoded using the calibration information and add to the frame.
   * @param data raw data packet
   * @param bytesReceived size of the data packet
   */
  virtual void ProcessPacket(unsigned char* data, std::size_t bytesReceived) = 0;

  /**
   * @brief IsLidarPacket check if the given packet is really a lidar packet
   * @param data
   * @param dataLength
   * @param headerReference
   * @param dataHeaderLength
   * @return
   */
  virtual bool IsLidarPacket(const unsigned char*& data, unsigned int& dataLength,
                        pcap_pkthdr** headerReference, unsigned int* dataHeaderLength) = 0;

  /**
   * @brief CountNewFrameInPacket return how many new frame should be created with this new packet,
   * the return should be positive (0 most of the time)
   * @param data
   * @param dataLength
   * @param headerReference
   * @param dataHeaderLength
   * @return
   */
  virtual int CountNewFrameInPacket(const unsigned char*& data, unsigned int& dataLength,
                                    pcap_pkthdr** headerReference, unsigned int* dataHeaderLength) = 0;

  /**
   * @brief SetTimestepInformation
   * @param info
   */
  void SetTimestepInformation(vtkInformation* info);

  //! Name of the pcap file to read
  std::string FileName;

  //! pcap packet index for every frame which enable to jump quicky from one frame to another
  std::vector<fpos_t> FilePositions;

  //! libpcap wrapped reader which enable to get the raw pcap packet from the pcap file
  vtkPacketFileReader* Reader;

  //! Number of allowed split, for frame-range retrieval.
  int SplitCounter;

  //! Store the frame with
  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;

};

#endif // VTKLIDARREADERINTERNAL_H
