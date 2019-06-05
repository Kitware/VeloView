#ifndef VTK_VELODYNE_ADVANCED_PACKET_INTERPRETOR_H
#define VTK_VELODYNE_ADVANCED_PACKET_INTERPRETOR_H

#include "vtkLidarPacketInterpreter.h"
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkDoubleArray.h>

#include <memory>
#include <limits>

#include "vtkDataPacket.h"
using namespace DataPacketFixedLength;

// #define DEBUG_MSG(msg) std::cout << msg << " [line " << __LINE__ << "]" << std::endl;

//------------------------------------------------------------------------------
// Forward declaration.
class FrameTracker;

//------------------------------------------------------------------------------
class VTK_EXPORT vtkVelodyneAdvancedPacketInterpreter : public vtkLidarPacketInterpreter
{
//------------------------------------------------------------------------------
private:
  FrameTracker * CurrentFrameTracker;

  //! @brief The maximum frame size seen so far.
  size_t MaxFrameSize;

  /*!
   * @brief     Update the maximum frame size.
   * @param[in] frameSize The currently requested frame size.
   */
  void UpdateMaxFrameSize(size_t frameSize);

  //! @brief The current allocated array size.
  size_t CurrentArraySize;

  //! @brief The number of points in the current frame.
  size_t NumberOfPointsInCurrentFrame;

  //! @brief Add the point and field arrays to a new polydata.
  // vtkSmartPointer<vtkPolyData> PreparePolyData();

  //! @brief Resize the point and metadata arrays to the current max frame size.
  void ResizeArrays();

  /*!
   * @brief     Set the number of items in the arrays.
   * @param[in] newSize The number of items to set.
   * 
   * If the number of points is greater than the current number of points,
   * ResizeArrays will be called first to ensure that points are preserved.
   */
  void SetNumberOfItems(size_t numberOfItems);

//------------------------------------------------------------------------------
public:
  static vtkVelodyneAdvancedPacketInterpreter* New();
  vtkTypeMacro(vtkVelodyneAdvancedPacketInterpreter, vtkLidarPacketInterpreter)

  std::string GetSensorInformation() override { return "advanced data format"; }

  void LoadCalibration(const std::string& filename) override;

  void ProcessPacket(unsigned char const * data, unsigned int dataLength) override;

  bool SplitFrame(bool force = false) override;

  bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) override;

  vtkSmartPointer<vtkPolyData> CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints = 60000) override;

  void ResetCurrentFrame() override;

  bool PreProcessPacket(const unsigned char *data, unsigned int dataLength, fpos_t filePosition, double packetNetworkTime, std::vector<FrameInformation> *frameCatalog) override;
//  void PreProcessPacket(unsigned char const * data, unsigned int dataLength, bool &isNewFrame, int &framePositionInPacket) override;

  void ResetParserMetaData() override;
  void SetParserMetaData(const FrameInformation& metaData) override;
  FrameInformation GetParserMetaData() override;

  vtkSmartPointer<vtkPoints> Points;

  // Update VAPI_FIELD_ARRAYS and CreateNewEmptyFrame whenever an array is
  // added or removed.

  vtkSmartPointer<vtkDoubleArray>         INFO_Xs;
  vtkSmartPointer<vtkDoubleArray>         INFO_Ys;
  vtkSmartPointer<vtkDoubleArray>         INFO_Zs;
  vtkSmartPointer<vtkUnsignedIntArray>    INFO_Distances;
  vtkSmartPointer<vtkDoubleArray>         INFO_Azimuths;
  vtkSmartPointer<vtkDoubleArray>         INFO_VerticalAngles;

  // Currently using signed instead of unsigned ints so that -1 can be used to
  // indicate that the value was not included in the return.
  vtkSmartPointer<vtkUnsignedIntArray>    INFO_Confidences;
  vtkSmartPointer<vtkUnsignedIntArray>    INFO_Intensities;
  vtkSmartPointer<vtkUnsignedIntArray>    INFO_Reflectivities;
  /*
  vtkSmartPointer<vtkStringArray>         INFO_DistanceTypeStrings;
  vtkSmartPointer<vtkStringArray>         INFO_FiringModeStrings;
  vtkSmartPointer<vtkStringArray>         INFO_StatusStrings;
*/
  vtkSmartPointer<vtkUnsignedCharArray>   INFO_DistanceTypes;
  vtkSmartPointer<vtkUnsignedCharArray>   INFO_ChannelNumbers;
  vtkSmartPointer<vtkUnsignedCharArray>   INFO_Noises;
  vtkSmartPointer<vtkUnsignedCharArray>   INFO_Powers;

  vtkSmartPointer<vtkUnsignedIntArray>    INFO_Pseqs;
  vtkSmartPointer<vtkUnsignedIntArray>    INFO_TimeFractionOffsets;

protected:
  vtkVelodyneAdvancedPacketInterpreter();
  ~vtkVelodyneAdvancedPacketInterpreter();

//------------------------------------------------------------------------------
// Code from legacy packet format interpreter.
private:
	void Init();
	void InitTrigonometricTables();
  void PrecomputeCorrectionCosSin();
  template <typename TAzm, typename TDist>
  void ComputeCorrectedValues(
    TAzm const azimuth,
    double const verticalAngleInDegrees,
    size_t const correctionIndex,
    double pos[3],
    TDist & distance
  );

public:
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
  double XMLColorTable[HDL_MAX_NUM_LASERS][3];
  // bool IsCorrectionFromLiveStream;

  uint8_t ReportedFactoryField1 ;
  uint8_t ReportedFactoryField2 ;
  bool OutputPacketProcessingDebugInfo ;
  bool UseIntraFiringAdjustment;
  unsigned int DualReturnFilter ;
  int FiringsSkip;
  bool IsCorrectionFromLiveStream;
  bool IsHDL64Data ;
  bool HasDualReturn ;
  bool ShouldAddDualReturnArray;
  bool WantIntensityCorrection;
};


/**
 * @brief VelodyneAdvancedSpecificFrameInformation frame information
 *        that are specific to velodyne's sensor
 */
struct VelodyneAdvancedSpecificFrameInformation : public SpecificFrameInformation
{
  //! Offset specific to the lidar data format
  //! Used because some frames start at the middle of a packet
  int FiringToSkip = 0;

  //! Indicates the number of time rolled that has occured
  //! since the beginning of the .pcap file. hence, to have
  //! a non rolling timestamp one should add to the rolling
  //! timestamp NbrOfRollingTime * MaxTimeBeforeRolling
  int NbrOfRollingTime = 0;

  //! Deep copy the specific frame information
  std::shared_ptr<SpecificFrameInformation> CopyTo()
  {
    auto copiedInfo = std::make_shared<VelodyneAdvancedSpecificFrameInformation>();
    auto* copiedPtr = reinterpret_cast<VelodyneAdvancedSpecificFrameInformation*>(copiedInfo.get());
    auto* toCopyPtr = reinterpret_cast<VelodyneAdvancedSpecificFrameInformation*>(this);
    copiedPtr->FiringToSkip = toCopyPtr->FiringToSkip;
    copiedPtr->NbrOfRollingTime = toCopyPtr->NbrOfRollingTime;
    return copiedInfo;
  }
};

#endif // VTK_VELODYNE_ADVANCED_PACKET_INTERPRETOR_H
