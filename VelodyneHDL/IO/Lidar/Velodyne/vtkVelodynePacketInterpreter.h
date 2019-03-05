#ifndef VELODYNEPACKETINTERPRETOR_H
#define VELODYNEPACKETINTERPRETOR_H

#include "vtkLidarPacketInterpreter.h"
#include "vtkDataPacket.h"
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>
#include <memory>

using namespace DataPacketFixedLength;

class RPMCalculator;
class FramingState;
class vtkRollingDataAccumulator;


class VTK_EXPORT vtkVelodynePacketInterpreter : public vtkLidarPacketInterpreter
{
public:
  static vtkVelodynePacketInterpreter* New();
  vtkTypeMacro(vtkVelodynePacketInterpreter, vtkLidarPacketInterpreter);
  void PrintSelf(ostream& os, vtkIndent indent){};
  enum DualFlag
  {
    DUAL_DISTANCE_NEAR = 0x1,  // point with lesser distance
    DUAL_DISTANCE_FAR = 0x2,   // point with greater distance
    DUAL_INTENSITY_HIGH = 0x4, // point with lesser intensity
    DUAL_INTENSITY_LOW = 0x8,  // point with greater intensity
    DUAL_DOUBLED = 0xf,        // point is single return
    DUAL_DISTANCE_MASK = 0x3,
    DUAL_INTENSITY_MASK = 0xc,
  };

  void LoadCalibration(const std::string& filename) override;

  void ProcessPacket(unsigned char const * data, unsigned int dataLength, int startPosition = 0) override;

  bool SplitFrame(bool force = false) override;

  bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) override;

  vtkSmartPointer<vtkPolyData> CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints = 60000) override;

  void ResetCurrentFrame() override;

  void PreProcessPacket(unsigned char const * data, unsigned int dataLength, bool &isNewFrame, int &framePositionInPacket) override;

  std::string GetSensorInformation() override;

  void SetSelectedPointsWithDualReturn(double* data, int Npoints);

  void GetXMLColorTable(double XMLColorTable[]);

  void GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
    double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
    double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
    double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
    double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
    double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
    double maxIntensity[HDL_MAX_NUM_LASERS]);

  vtkSetMacro(ShouldAddDualReturnArray, bool)

  vtkGetMacro(HasDualReturn, bool)

  vtkSetMacro(WantIntensityCorrection, bool)

  vtkGetMacro(FiringsSkip, int)
  vtkSetMacro(FiringsSkip, int)

  vtkGetMacro(UseIntraFiringAdjustment, bool)
  vtkSetMacro(UseIntraFiringAdjustment, bool)

  vtkSetMacro(DualReturnFilter, unsigned int)

protected:
  // Process the laser return from the firing data
  // firingData - one of HDL_FIRING_PER_PKT from the packet
  // hdl64offset - either 0 or 32 to support 64-laser systems
  // firingBlock - block of packet for firing [0-11]
  // azimuthDiff - average azimuth change between firings
  // timestamp - the timestamp of the packet
  // geotransform - georeferencing transform
  void ProcessFiring(const HDLFiringData* firingData,
    int firingBlockLaserOffset, int firingBlock, int azimuthDiff, double timestamp,
    unsigned int rawtime, bool isThisFiringDualReturnData, bool isDualReturnPacket);

  void PushFiringData(unsigned char laserId, unsigned char rawLaserId,
                      unsigned short azimuth, double timestamp,
                      unsigned int rawtime, const HDLLaserReturn* laserReturn,
                      const HDLLaserCorrection* correction, bool isFiringDualReturnData);

  void InitTrigonometricTables();

  void PrecomputeCorrectionCosSin();

  void Init();

  double ComputeTimestamp(unsigned int tohTime);

  void ComputeCorrectedValues(const unsigned short azimuth,
                              const HDLLaserReturn* laserReturn, const HDLLaserCorrection* correction, double pos[3],
                              double& distanceM, short& intensity, bool correctIntensity);

  bool HDL64LoadCorrectionsFromStreamData();

  bool CheckReportedSensorAndCalibrationFileConsistent(const HDLDataPacket* dataPacket);

  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkDoubleArray> PointsX;
  vtkSmartPointer<vtkDoubleArray> PointsY;
  vtkSmartPointer<vtkDoubleArray> PointsZ;
  vtkSmartPointer<vtkUnsignedCharArray> Intensity;
  vtkSmartPointer<vtkUnsignedCharArray> LaserId;
  vtkSmartPointer<vtkUnsignedShortArray> Azimuth;
  vtkSmartPointer<vtkDoubleArray> Distance;
  vtkSmartPointer<vtkUnsignedShortArray> DistanceRaw;
  vtkSmartPointer<vtkDoubleArray> Timestamp;
  vtkSmartPointer<vtkDoubleArray> VerticalAngle;
  vtkSmartPointer<vtkUnsignedIntArray> RawTime;
  vtkSmartPointer<vtkIntArray> IntensityFlag;
  vtkSmartPointer<vtkIntArray> DistanceFlag;
  vtkSmartPointer<vtkUnsignedIntArray> Flags;
  vtkSmartPointer<vtkIdTypeArray> DualReturnMatching;
  vtkSmartPointer<vtkDoubleArray> SelectedDualReturn;

  bool ShouldAddDualReturnArray;

  // sensor information
  bool HasDualReturn;
  SensorType ReportedSensor;
  DualReturnSensorMode ReportedSensorReturnMode;
  bool IsHDL64Data;
  bool IsVLS128;
  uint8_t ReportedFactoryField1;
  uint8_t ReportedFactoryField2;

  bool alreadyWarnedForIgnoredHDL64FiringPacket;

  bool OutputPacketProcessingDebugInfo;

  // Bolean to manage the correction of intensity which indicates if the user want to correct the
  // intensities
  bool WantIntensityCorrection;

  // WIP : We now have two method to compute the RPM :
  // - One method which computes the rpm using the point cloud
  // this method is not packets dependant but need a none empty
  // point cloud which can be tricky (hard cropping, none spinning sensor)
  // - One method which computes the rpm directly using the packets. the problem
  // is, if the packets format change, we will have to adapt the rpm computation
  // - For now, we will use the packet dependant method
//  std::unique_ptr<RPMCalculator> RpmCalculator_;

  RPMCalculator* RpmCalculator_;

  FramingState* CurrentFrameState;
  unsigned int LastTimestamp;
  std::vector<double> RpmByFrames;
  double TimeAdjust;
  vtkIdType LastPointId[HDL_MAX_NUM_LASERS];
  vtkIdType FirstPointIdOfDualReturnPair;

  unsigned char SensorPowerMode;

  // Parameters ready by calibration
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
  double XMLColorTable[HDL_MAX_NUM_LASERS][3];
  bool IsCorrectionFromLiveStream = true;

  // Sensor parameters presented as rolling data, extracted from enough packets
  vtkRollingDataAccumulator* rollingCalibrationData;

  // User configurable parameters
  int FiringsSkip;

  bool UseIntraFiringAdjustment;

  bool ShouldCheckSensor;

  unsigned int DualReturnFilter;

  vtkVelodynePacketInterpreter();
  ~vtkVelodynePacketInterpreter();

private:
  vtkVelodynePacketInterpreter(const vtkVelodynePacketInterpreter&) = delete;
  void operator=(const vtkVelodynePacketInterpreter&) = delete;
};

#endif // VELODYNEPACKETINTERPRETOR_H
