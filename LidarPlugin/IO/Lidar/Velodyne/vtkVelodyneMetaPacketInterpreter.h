#ifndef VELODYNEMETAPACKETINTERPRETOR_H
#define VELODYNEMETAPACKETINTERPRETOR_H

#include "vtkLidarPacketInterpreter.h"

#include <vtkNew.h>
#include <vtkCollection.h>

#include "vtkVelodyneLegacyPacketInterpreter.h"
#include "vtkVelodyneAdvancedPacketInterpreter.h"

// right now nothing enforce the different Velodyne Interpreter to have the same API.
// There is no "VelodynePacketInterpreter" based class, however if the API differ,
// this class won't compile.
#define CallWithDynamiCast(function_with_arg, element) \
    auto* pt1 = vtkVelodyneLegacyPacketInterpreter::SafeDownCast(this->element); \
    if(pt1) return pt1->function_with_arg; \
    auto* pt2 = vtkVelodyneAdvancedPacketInterpreter::SafeDownCast(this->element); \
    if(pt2) return pt2->function_with_arg;

#define GenericSetMacro(name,type) \
void Set##name (type _arg) override \
{ \
  if (this->SelectedInterp) \
  { \
    this->SelectedInterp->Set##name(_arg);\
  } \
  else \
  { \
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i) \
    { \
      vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(i)); \
      pt->Set##name(_arg); \
    } \
  } \
}

#define GenericGetMacro(name, type) \
type Get##name () override \
{ \
  if (this->SelectedInterp) \
  { \
    return this->SelectedInterp->Get##name();\
  } \
  vtkLidarPacketInterpreter* pt = vtkLidarPacketInterpreter::SafeDownCast(this->PotentialInterps->GetItemAsObject(0)); \
  return pt->Get##name(); \
}

#define SpecificSetMacro(name,type) \
void Set##name (type _arg) \
{ \
  if (this->SelectedInterp) \
  { \
    CallWithDynamiCast(Set##name(_arg), SelectedInterp) \
  } \
  else \
  { \
    for (int i = 0; i < this->PotentialInterps->GetNumberOfItems(); ++i) \
    { \
      CallWithDynamiCast(Set##name(_arg), PotentialInterps->GetItemAsObject(i)) \
    } \
  } \
}

#define SpecificGetMacro(name, type) \
type Get##name () \
{ \
  if (this->SelectedInterp) \
  { \
    CallWithDynamiCast(Get##name(), SelectedInterp) \
  } \
  CallWithDynamiCast(Get##name(), PotentialInterps->GetItemAsObject(0)) \
  return type(); \
}

#define redirect_directly_function(function_call) \
  assert(this->SelectedInterp); \
  return this->SelectedInterp->function_call;

//!
//! \brief The vtkVelodyneMetaPacketInterpreter class is a meta interpreter that hold different Velodyne Interpeter and redict all
//! call to the right one
//!
class VTK_EXPORT vtkVelodyneMetaPacketInterpreter : public vtkLidarPacketInterpreter
{
public:
  static vtkVelodyneMetaPacketInterpreter* New();
  vtkTypeMacro(vtkVelodyneMetaPacketInterpreter, vtkLidarPacketInterpreter)

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

  GenericGetMacro( CalibrationTable, vtkSmartPointer<vtkTable>)

  void ProcessPacket(unsigned char const * data, unsigned int dataLength) override { redirect_directly_function(ProcessPacket(data, dataLength)) }

  bool SplitFrame(bool force = false) override { redirect_directly_function(SplitFrame(force)) }

  bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) override;

  void ResetCurrentFrame() override;

  bool PreProcessPacket(unsigned char const * data, unsigned int dataLength,
                        fpos_t filePosition = fpos_t(), double packetNetworkTime = 0,
                        std::vector<FrameInformation>* frameCatalog = nullptr) override { redirect_directly_function(PreProcessPacket(data, dataLength, filePosition, packetNetworkTime, frameCatalog)) }

  bool IsNewFrameReady() override { redirect_directly_function(IsNewFrameReady()) }

  vtkSmartPointer<vtkPolyData> GetLastFrameAvailable() override { redirect_directly_function(GetLastFrameAvailable()) }

  void ClearAllFramesAvailable() override { redirect_directly_function(ClearAllFramesAvailable()) }

  std::string GetSensorInformation() override;

  GenericGetMacro(ParserMetaData, FrameInformation)

  void ResetParserMetaData() override;

  void SetParserMetaData(const FrameInformation &metaData) override { redirect_directly_function(SetParserMetaData(metaData)) }

  GenericGetMacro(NumberOfChannels, int)

  GenericGetMacro(CalibrationFileName, std::string)
  GenericSetMacro(CalibrationFileName, std::string)

  GenericGetMacro(IsCalibrated, bool)

  GenericGetMacro(TimeOffset, double)
  GenericSetMacro(TimeOffset, double)

  // Laser selection

  GenericGetMacro(DistanceResolutionM, double)

  GenericGetMacro(Frequency, double)

  GenericGetMacro(IgnoreZeroDistances, bool)
  GenericSetMacro(IgnoreZeroDistances, bool)

  GenericGetMacro(IgnoreEmptyFrames, bool)
  GenericSetMacro(IgnoreEmptyFrames, bool)

  GenericGetMacro(ApplyTransform, bool)
  GenericSetMacro(ApplyTransform, bool)

  GenericGetMacro(SensorTransform, vtkTransform*)
  GenericSetMacro(SensorTransform, vtkTransform*)

  GenericGetMacro(CropMode, int)
  GenericSetMacro(CropMode, int)

  GenericGetMacro(CropOutside, bool)
  GenericSetMacro(CropOutside, bool)

  void SetCropRegion(double _arg[6]) override { this->SetCropRegion(_arg[0], _arg[1], _arg[2], _arg[3], _arg[4], _arg[5]); }
  void SetCropRegion(double _arg1, double _arg2, double _arg3, double _arg4, double _arg5, double _arg6) override;


  //! @todo
  void GetXMLColorTable(double XMLColorTable[]) {}

  //! @todo
//  void GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
//    double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
//    double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
//    double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
//    double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
//    double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
//                           double maxIntensity[HDL_MAX_NUM_LASERS]) {}


  SpecificGetMacro(HasDualReturn, bool)

  SpecificGetMacro(WantIntensityCorrection, bool)
  SpecificSetMacro(WantIntensityCorrection, bool)

  SpecificGetMacro(FiringsSkip, bool)
  SpecificSetMacro(FiringsSkip, bool)

  SpecificGetMacro(UseIntraFiringAdjustment, bool)
  SpecificSetMacro(UseIntraFiringAdjustment, bool)

  SpecificGetMacro(DualReturnFilter, bool)
  SpecificSetMacro(DualReturnFilter, bool)

  vtkMTimeType GetMTime() override;

protected:
    vtkVelodyneMetaPacketInterpreter();

private:
  vtkVelodyneMetaPacketInterpreter(const vtkVelodyneMetaPacketInterpreter&) = delete;
  void operator=(const vtkVelodyneMetaPacketInterpreter&) = delete;

  //! Collection of interpreter that can be used by this meta interpreter
  vtkNew<vtkCollection> PotentialInterps;
  //! The interpreter actually used
  vtkLidarPacketInterpreter* SelectedInterp = nullptr;
};

#endif // VELODYNEMETAPACKETINTERPRETOR_H
