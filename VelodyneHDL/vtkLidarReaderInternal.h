#ifndef VTKLIDARREADERINTERNAL_H
#define VTKLIDARREADERINTERNAL_H

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkTransform.h>

#include <vector>


class vtkLidarReader;
class pcap_pkthdr;
class vtkPacketFileReader;
struct  vtkLidarReaderInternal
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
   * @brief ReadFrameInformation read the whole pcap and create a frame index. In case the
   * calibration is also contain in the pcap file, this will also read it
   */
  int ReadFrameInformation();

  /**
   * @brief SetTimestepInformation indicate to vtk which time step are available
   * @param info
   */
  void SetTimestepInformation(vtkInformation* info);

  //! Name of the pcap file to read
  std::string FileName;

  //! pcap packet index for every frame which enable to jump quicky from one frame to another
  std::vector<fpos_t> FilePositions;

  //! Frame do not new to start at the begin of a packet, this indicate
  std::vector<int> FilePositionsSkip;

  //! libpcap wrapped reader which enable to get the raw pcap packet from the pcap file
  vtkPacketFileReader* Reader;
};

#endif // VTKLIDARREADERINTERNAL_H
