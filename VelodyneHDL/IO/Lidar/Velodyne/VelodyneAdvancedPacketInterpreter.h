#ifndef VELODYNE_ADVANCED_PACKET_INTERPRETOR_H
#define VELODYNE_ADVANCED_PACKET_INTERPRETOR_H

#include "LidarPacketInterpreter.h"
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkStringArray.h>

#include <memory>
#include <limits>

#include "vtkDataPacket.h"
using namespace DataPacketFixedLength;

//------------------------------------------------------------------------------
// Forward declaration.
class FrameTracker;

template <bool loadData>
class FiringReturn;

//------------------------------------------------------------------------------
class VelodyneAdvancedPacketInterpreter : public LidarPacketInterpreter
{
private:
  FrameTracker * CurrentFrameTracker;
  size_t FrameSize = 0;

public:
  VelodyneAdvancedPacketInterpreter();
  ~VelodyneAdvancedPacketInterpreter();

  void LoadCalibration(const std::string& filename) override;

  void ProcessPacket(unsigned char const * data, unsigned int dataLength, int startPosition = 0) override;

  bool SplitFrame(bool force = false) override;

  bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) override;

  vtkSmartPointer<vtkPolyData> CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints = 60000) override;

  void ResetCurrentFrame() override;

  void PreProcessPacket(unsigned char const * data, unsigned int dataLength, bool &isNewFrame, int &framePositionInPacket) override;

  vtkSmartPointer<vtkPoints> Points;

  vtkSmartPointer<vtkDoubleArray>         INFO_Xs;
  vtkSmartPointer<vtkDoubleArray>         INFO_Ys;
  vtkSmartPointer<vtkDoubleArray>         INFO_Zs;
  vtkSmartPointer<vtkDoubleArray>         INFO_Azimuths;
  vtkSmartPointer<vtkDoubleArray>         INFO_VerticalAngles;

  // Currently using signed instead of unsigned ints so that -1 can be used to
  // indicate that the value was not included in the return.
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
  vtkSmartPointer<vtkUnsignedIntArray>    INFO_TimeFractionOffsets;

//------------------------------------------------------------------------------
// Code from legacy packet format interpreter.
private:
	void Init();
	void InitTrigonometricTables();
  void PrecomputeCorrectionCosSin();
  template <typename T>
  void ComputeCorrectedValues(
    T const azimuth,
    FiringReturn<true> const firingReturn,
    size_t const correctionIndex,
    double pos[3]
  );

public:
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
  double XMLColorTable[HDL_MAX_NUM_LASERS][3];
  // bool IsCorrectionFromLiveStream;
  
  uint8_t ReportedFactoryField1 = 0;
  uint8_t ReportedFactoryField2 = 0;
  bool OutputPacketProcessingDebugInfo = false;
  bool UseIntraFiringAdjustment = false;
  unsigned int DualReturnFilter = 0;
  int FiringsSkip = 0;
  bool IsCorrectionFromLiveStream = false;
  bool IsHDL64Data = false;
  bool HasDualReturn = false;
  bool ShouldAddDualReturnArray = false;
  bool WantIntensityCorrection = false;
};

#endif // VELODYNE_ADVANCED_PACKET_INTERPRETOR_H
