#ifndef VTKVELODYNEVELARRAYPACKETINTERPRETER_H
#define VTKVELODYNEVELARRAYPACKETINTERPRETER_H

#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>
#include <vtkUnsignedLongArray.h>
#include <vtkStringArray.h>
#include "vtkVelodyneBasePacketInterpreter.h"
#include "vtkVelodyneAdvancedPacketFraming.h"
#include "SpecialVelarrayPacket.h"

#include <memory>

using namespace DataPacketFixedLength;

class VTK_EXPORT vtkVelodyneSpecialVelarrayPacketInterpreter : public vtkVelodyneBasePacketInterpreter
{
public:
  static vtkVelodyneSpecialVelarrayPacketInterpreter* New();
  vtkTypeMacro(vtkVelodyneSpecialVelarrayPacketInterpreter, vtkVelodyneBasePacketInterpreter);
  void PrintSelf(ostream& vtkNotUsed(os), vtkIndent vtkNotUsed(indent)){};
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

  void ProcessPacket(unsigned char const * data, unsigned int dataLength) override;

  bool SplitFrame(bool force = false) override;

  bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) override;


  void ResetCurrentFrame() override;

  bool PreProcessPacket(unsigned char const * data, unsigned int dataLength,
                        fpos_t filePosition = fpos_t(), double packetNetworkTime = 0,
                        std::vector<FrameInformation>* frameCatalog = nullptr) override;

  std::string GetSensorInformation() override;


protected:
  template<typename T>
  vtkSmartPointer<T> CreateDataArray(bool isAdvanced, const char* name, vtkIdType np, vtkIdType prereserved_np, vtkPolyData* pd);

  vtkSmartPointer<vtkPolyData> CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints = 60000) override;

  void ProcessFiring(const SpecialVelarrayPacket::PayloadHeader* header,
                     const SpecialVelarrayPacket::FiringReturn* firingData,
                     const SpecialVelarrayPacket::PayloadFooter* footer);

  void PushFiringData(unsigned char laserId, unsigned char rawLaserId,
                      unsigned short azimuth, const unsigned short elevation, const double timestamp,
                      const unsigned int rawtime, const HDLLaserReturn* laserReturn,
                      const unsigned int channelNumber, const bool isFiringDualReturnData,
                      const int extDataPacketType, const HDLLaserReturn* extData);

  void Init();

  double ComputeTimestamp(unsigned int tohTime, const FrameInformation& frameInfo);

  bool CheckReportedSensorAndCalibrationFileConsistent(const SpecialVelarrayPacket::Packet* dataPacket);

  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkDoubleArray> PointsX;
  vtkSmartPointer<vtkDoubleArray> PointsY;
  vtkSmartPointer<vtkDoubleArray> PointsZ;
  vtkSmartPointer<vtkUnsignedCharArray> Intensity;
  vtkSmartPointer<vtkUnsignedCharArray> Drop;
  vtkSmartPointer<vtkStringArray> Confidence;
  vtkSmartPointer<vtkUnsignedCharArray> Interference;
  vtkSmartPointer<vtkUnsignedCharArray> SNR;
  vtkSmartPointer<vtkUnsignedCharArray> SunLevel;
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
  vtkSmartPointer<vtkUnsignedCharArray> HDir;
  vtkSmartPointer<vtkUnsignedCharArray> VDir;
  unsigned int LaserCounter = 0;
  vtkSmartPointer<vtkUnsignedIntArray> LaserCounters;
  vtkSmartPointer<vtkUnsignedLongArray> TREF; // reference timestamp in PTP truncated (64-bit) format
  vtkSmartPointer<vtkUnsignedIntArray> PSEQ; // payload sequence number
  vtkSmartPointer<vtkUnsignedCharArray> PSEQF; // payload sequence number within a frame
  vtkSmartPointer<vtkUnsignedCharArray> AC; // alive counter


  // sensor information
  unsigned char SensorPowerMode;
  SensorType ReportedSensor;
  DualReturnSensorMode ReportedSensorReturnMode;
  uint8_t ReportedFactoryField1;
  uint8_t ReportedFactoryField2;

  bool OutputPacketProcessingDebugInfo;

  // Bolean to manage the correction of intensity which indicates if the user want to correct the
  // intensities
  bool WantIntensityCorrection;

  FrameTracker* CurrentFrameState;
  unsigned int LastTimestamp;
  int LastAzimuth, LastAzimuthDiff;
  double TimeAdjust;
  vtkIdType LastPointId[HDL_MAX_NUM_LASERS];
  vtkIdType FirstPointIdOfDualReturnPair;

  unsigned char epororPowerMode;

  bool IsCorrectionFromLiveStream = true;

  bool ShouldCheckSensor;

  uint8_t ReportedMICField = 0;

  vtkVelodyneSpecialVelarrayPacketInterpreter();
  ~vtkVelodyneSpecialVelarrayPacketInterpreter();

private:
  vtkVelodyneSpecialVelarrayPacketInterpreter(const vtkVelodyneSpecialVelarrayPacketInterpreter&) = delete;
  void operator=(const vtkVelodyneSpecialVelarrayPacketInterpreter&) = delete;

};

struct SpecialVelarraySpecificFrameInformation : public SpecificFrameInformation
{
  //! Offset specific to the lidar data format
  //! Used because some frames start at the middle of a packet
  int FiringToSkip = 0;

  void reset() { *this = SpecialVelarraySpecificFrameInformation(); }
  std::unique_ptr<SpecificFrameInformation> clone() { return std::make_unique<SpecialVelarraySpecificFrameInformation>(*this); }
};

#endif // VTKVELODYNEVELARRAYPACKETINTERPRETER_H
