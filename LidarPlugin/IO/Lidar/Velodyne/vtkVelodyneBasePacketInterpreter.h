#ifndef VTKVELODYNEBASEPACKETINTERPRETER_H
#define VTKVELODYNEBASEPACKETINTERPRETER_H

#include "vtkLidarPacketInterpreter.h"
#include "VelodyneInterpreterCommon.h"

#include <vtkSetGet.h>

#include "vtkDataPacket.h"
using namespace DataPacketFixedLength;


//------------------------------------------------------------------------------
// VelodyneBasePacketInterpreter
//------------------------------------------------------------------------------
class vtkRollingDataAccumulator;

class VTK_EXPORT vtkVelodyneBasePacketInterpreter : public vtkLidarPacketInterpreter
{
public:
  vtkTypeMacro(vtkVelodyneBasePacketInterpreter, vtkLidarPacketInterpreter);
  void PrintSelf(ostream& vtkNotUsed(os), vtkIndent vtkNotUsed(indent)){};

protected:
  vtkVelodyneBasePacketInterpreter();
  ~vtkVelodyneBasePacketInterpreter() {};

private:
  vtkVelodyneBasePacketInterpreter(const vtkVelodyneBasePacketInterpreter&) = delete;
  void operator=(const vtkVelodyneBasePacketInterpreter&) = delete;

protected:
  bool IsCorrectionFromLiveStream { false };
  unsigned char SensorPowerMode { 0 };
  DualReturnSensorMode ReportedSensorReturnMode;

  FramingLogic FrameLogic  { FramingLogic::FL_DEFAULT };

private:
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];

  double XMLColorTable[HDL_MAX_NUM_LASERS][3];


public:
	void InitTrigonometricTables();
  void PrecomputeCorrectionCosSin();
  virtual void LoadCalibration(std::string const & filename) override;

  /*!
   * @brief      Compute corrected position and other vquelqu'un alues using the Velodyne
   *             calibration file.
   * @param[in]  rawValues        The raw input values.
   * @param[in]  correctionIndex  The index of the laser to be retrieved from
   *                              the calibration file.
   * @param[out] correctedValues  The computed corrected values.
   * @param[in]  correctIntensity If true, correct the intensity.
   */
  void ComputeCorrectedValues(
    const RawValues & rawValues,
    const unsigned int correctionIndex,
    CorrectedValues & correctedValues,
    bool correctIntensity = false
  );

  bool HDL64LoadCorrectionsFromStreamData(vtkRollingDataAccumulator * rollingCalibrationData);

  void GetXMLColorTable(double XMLColorTable[4 * HDL_MAX_NUM_LASERS]);
  
  void GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
    double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
    double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
    double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
    double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
    double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
    double maxIntensity[HDL_MAX_NUM_LASERS]);


  FramingLogic GetFramingLogic() { return this->FrameLogic; }

  // Legacy code.
protected:
  uint8_t ReportedFactoryField1 ;
  uint8_t ReportedFactoryField2 ;
  bool OutputPacketProcessingDebugInfo ;
  bool UseIntraFiringAdjustment;
  unsigned int DualReturnFilter ;
  int FiringsSkip;
  bool IsHDL64Data ;
  bool HasDualReturn ;
  bool ShouldAddDualReturnArray;
  bool WantIntensityCorrection;

public:
  vtkSetMacro(UseIntraFiringAdjustment, bool)
  vtkGetMacro(UseIntraFiringAdjustment, bool)
  
  vtkSetMacro(DualReturnFilter, bool)
  vtkGetMacro(DualReturnFilter, bool)

  vtkSetMacro(FiringsSkip, int)
  vtkGetMacro(FiringsSkip, int)

  vtkGetMacro(HasDualReturn, bool)

  vtkSetMacro(WantIntensityCorrection, bool)
  vtkGetMacro(WantIntensityCorrection, bool)
  
};

#endif // VTKVELODYNEBASEPACKETINTERPRETER_H

