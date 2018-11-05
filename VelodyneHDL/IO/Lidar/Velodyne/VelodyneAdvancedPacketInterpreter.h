#ifndef VELODYNE_ADVANCED_PACKET_INTERPRETOR_H
#define VELODYNE_ADVANCED_PACKET_INTERPRETOR_H

#include "LidarPacketInterpreter.h"
#include "vtkDataPacket.h"
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>
#include <memory>

#include <limits>

//------------------------------------------------------------------------------
// Forward declaration.
class FrameTracker;

//------------------------------------------------------------------------------
class VelodyneAdvancedPacketInterpreter : public LidarPacketInterpreter
{
private:
  FrameTracker CurrentFrameTracker;
  size_t FrameSize = 0;

public:
  VelodyneAdvancedPacketInterpreter();
  ~VelodyneAdvancedPacketInterpreter();

  void LoadCalibration(const std::string& filename) override;

  void ProcessPacket(unsigned char const * data, unsigned int dataLength, int startPosition = 0) override;

  bool SplitFrame(bool force = false) override;

  bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) override;

  vtkSmartPointer<vtkPolyData> CreateNewEmptyFrame(vtkIdType numberOfPoints) override;

  void ResetCurrentFrame() override;

  void PreProcessPacket(unsigned char const * data, unsigned int dataLength, bool &isNewFrame, int &framePositionInPacket) override;

  vtkSmartPointer<vtkPoints> Points;

  vtkSmartPointer<vtkDoubleArray>         INFO_Xs;
  vtkSmartPointer<vtkDoubleArray>         INFO_Ys;
  vtkSmartPointer<vtkDoubleArray>         INFO_Zs;
  vtkSmarkPointer<vtkDoubleArray>         INFO_Azimuths;
  vtkSmarkPointer<vtkDoubleArray>         INFO_VerticalAngles;

  vtkSmartPointer<vtkIntArray>            INFO_Confidences;
  vtkSmartPointer<vtkIntArray>            INFO_Intensities;
  vtkSmartPointer<vtkIntArray>            INFO_Reflectivities;
  
  vtkSmartPointer<vtkStringArray>         INFO_DistanceTypeStrings;
  vtkSmartPointer<vtkStringArray>         INFO_FiringModeStrings;
  vtkSmartPointer<vtkStringArray>         INFO_StatusStrings;

  vtkSmartPointer<vtkUnsignedCharArray>   INFO_ChannelNumbers;
  vtkSmartPointer<vtkUnsignedCharArray>   INFO_Noises;
  vtkSmartPointer<vtkUnsignedCharArray>   INFO_Powers;
  vtkSmartPointer<vtkUnsignedIntArray>    INFO_Pseqs;
};

#endif // VELODYNE_ADVANCED_PACKET_INTERPRETOR_H
