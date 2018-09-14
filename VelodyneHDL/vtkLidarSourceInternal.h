#ifndef VTKLIDARSOURCEINTERNAL_H
#define VTKLIDARSOURCEINTERNAL_H

#include "vtkLidarSource.h"

#include "vtkPacketFileReader.h"
//-----------------------------------------------------------------------------
class vtkLidarSourceInternal
{
public:
  enum CropModeEnum
  {
    None = 0,
    Cartesian = 1,
    Spherical = 2,
    Cylindric = 3,
  };

  vtkLidarSourceInternal();

  ~vtkLidarSourceInternal()
  {
//    delete this->CurrentFrameState;
  }

  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;
//  vtkSmartPointer<vtkPolyData> CurrentDataset;

//  vtkNew<vtkTransform> SensorTransform;
//  vtkSmartPointer<vtkVelodyneTransformInterpolator> Interp;

//  vtkSmartPointer<vtkPoints> Points;
//  vtkSmartPointer<vtkDoubleArray> PointsX;
//  vtkSmartPointer<vtkDoubleArray> PointsY;
//  vtkSmartPointer<vtkDoubleArray> PointsZ;
//  vtkSmartPointer<vtkUnsignedCharArray> Intensity;
//  vtkSmartPointer<vtkUnsignedCharArray> LaserId;
//  vtkSmartPointer<vtkUnsignedShortArray> Azimuth;
//  vtkSmartPointer<vtkDoubleArray> Distance;
//  vtkSmartPointer<vtkUnsignedShortArray> DistanceRaw;
//  vtkSmartPointer<vtkDoubleArray> Timestamp;
//  vtkSmartPointer<vtkDoubleArray> VerticalAngle;
//  vtkSmartPointer<vtkUnsignedIntArray> RawTime;
//  vtkSmartPointer<vtkIntArray> IntensityFlag;
//  vtkSmartPointer<vtkIntArray> DistanceFlag;
//  vtkSmartPointer<vtkUnsignedIntArray> Flags;
//  vtkSmartPointer<vtkIdTypeArray> DualReturnMatching;
//  vtkSmartPointer<vtkDoubleArray> SelectedDualReturn;
//  bool ShouldAddDualReturnArray;

  // sensor information
//  bool HasDualReturn;
//  SensorType ReportedSensor;
//  DualReturnSensorMode ReportedSensorReturnMode;

  bool IgnoreEmptyFrames;

//  bool OutputPacketProcessingDebugInfo;

//  // Bolean to manage the correction of intensity which indicates if the user want to correct the
//  // intensities
//  bool WantIntensityCorrection;

//  FramingState* CurrentFrameState;
//  unsigned int LastTimestamp;
  double currentRpm;
//  std::vector<double> RpmByFrames;
//  double TimeAdjust;
//  vtkIdType LastPointId[HDL_MAX_NUM_LASERS];
//  vtkIdType FirstPointIdOfDualReturnPair;

  std::vector<fpos_t> FilePositions;
  std::vector<int> Skips;
  int Skip;
  vtkPacketFileReader* Reader;

//  unsigned char SensorPowerMode;

//  // Number of allowed split, for frame-range retrieval.
  int SplitCounter;

//  // Parameters ready by calibration
//  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
//  double XMLColorTable[HDL_MAX_NUM_LASERS][3];
  int CalibrationReportedNumLasers;
  bool CorrectionsInitialized;
//  bool IsCorrectionFromLiveStream;

//  // Sensor parameters presented as rolling data, extracted from enough packets
//  vtkRollingDataAccumulator* rollingCalibrationData;

//  // User configurable parameters
  int NumberOfTrailingFrames;
//  int ApplyTransform;
//  int FiringsSkip;
  bool IgnoreZeroDistances;
//  bool UseIntraFiringAdjustment;

  CropModeEnum CropMode;
  bool CropReturns;
  bool CropOutside;
  double CropRegion[6];

  //  bool AlreadyWarnAboutCalibration;
//  double distanceResolutionM;

  std::vector<bool> LaserSelection;
//  unsigned int DualReturnFilter;

//  void SplitFrame(bool force = false);
//  vtkSmartPointer<vtkPolyData> CreateData(vtkIdType numberOfPoints);
//  vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);

//  void Init();
//  void InitTrigonometricTables();
//  void PrecomputeCorrectionCosSin();
//  void LoadCorrectionsFile(const std::string& filename);
//  bool HDL64LoadCorrectionsFromStreamData();
  bool shouldBeCroppedOut(double pos[3], double theta);

//  void ProcessHDLPacket(unsigned char* data, std::size_t bytesReceived);
//  static bool shouldSplitFrame(uint16_t, int, int&);

//  double ComputeTimestamp(unsigned int tohTime);
//  void ComputeOrientation(double timestamp, vtkTransform* geotransform);
  virtual void UnloadPerFrameData() = 0;

  virtual void SplitFrame(bool force) = 0;

  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrames);

  virtual void ProcessPacket(unsigned char* data, std::size_t bytesReceived) = 0;
};

#endif // VTKLIDARSOURCEINTERNAL_H
