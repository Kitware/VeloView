#ifndef VTKLIDARSOURCEINTERNAL_H
#define VTKLIDARSOURCEINTERNAL_H

//-----------------------------------------------------------------------------
class vtkLidarSourceInternal
{
public:
  vtkLidarSourceInternal()
  {
//    this->RpmCalculator.Reset();
//    this->AlreadyWarnAboutCalibration = false;
//    this->IgnoreZeroDistances = true;
//    this->UseIntraFiringAdjustment = true;
//    this->CropMode = Cartesian;
//    this->ShouldAddDualReturnArray = false;
//    this->alreadyWarnedForIgnoredHDL64FiringPacket = false;
//    this->OutputPacketProcessingDebugInfo = false;
//    this->SensorPowerMode = 0;
//    this->Skip = 0;
//    this->CurrentFrameState = new FramingState;
//    this->LastTimestamp = std::numeric_limits<unsigned int>::max();
//    this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
//    this->Reader = 0;
//    this->SplitCounter = 0;
    this->NumberOfTrailingFrames = 0;
//    this->ApplyTransform = 0;
//    this->FiringsSkip = 0;
//    this->CropReturns = false;
//    this->CropOutside = false;
//    this->CropRegion[0] = this->CropRegion[1] = 0.0;
//    this->CropRegion[2] = this->CropRegion[3] = 0.0;
//    this->CropRegion[4] = this->CropRegion[5] = 0.0;
//    this->CorrectionsInitialized = false;
//    this->currentRpm = 0;

//    std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);

//    this->LaserSelection.resize(HDL_MAX_NUM_LASERS, true);
//    this->DualReturnFilter = 0;
//    this->IsHDL64Data = false;
//    this->ReportedFactoryField1 = 0;
//    this->ReportedFactoryField2 = 0;
//    this->CalibrationReportedNumLasers = -1;
//    this->IgnoreEmptyFrames = true;
//    this->distanceResolutionM = 0.002;
//    this->WantIntensityCorrection = false;

//    this->rollingCalibrationData = new vtkRollingDataAccumulator();
//    this->Init();
  }

  ~vtkLidarSourceInternal()
  {
//    if (this->rollingCalibrationData)
//    {
//      delete this->rollingCalibrationData;
//    }
//    delete this->CurrentFrameState;
  }

//  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;
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
//  bool IsHDL64Data;
//  uint8_t ReportedFactoryField1;
//  uint8_t ReportedFactoryField2;

//  bool IgnoreEmptyFrames;
//  bool alreadyWarnedForIgnoredHDL64FiringPacket;

//  bool OutputPacketProcessingDebugInfo;

//  // Bolean to manage the correction of intensity which indicates if the user want to correct the
//  // intensities
//  bool WantIntensityCorrection;

//  // WIP : We now have two method to compute the RPM :
//  // - One method which computes the rpm using the point cloud
//  // this method is not packets dependant but need a none empty
//  // point cloud which can be tricky (hard cropping, none spinning sensor)
//  // - One method which computes the rpm directly using the packets. the problem
//  // is, if the packets format change, we will have to adapt the rpm computation
//  // - For now, we will use the packet dependant method
//  RPMCalculator RpmCalculator;

//  FramingState* CurrentFrameState;
//  unsigned int LastTimestamp;
//  double currentRpm;
//  std::vector<double> RpmByFrames;
//  double TimeAdjust;
//  vtkIdType LastPointId[HDL_MAX_NUM_LASERS];
//  vtkIdType FirstPointIdOfDualReturnPair;

//  std::vector<fpos_t> FilePositions;
//  std::vector<int> Skips;
//  int Skip;
//  vtkPacketFileReader* Reader;

//  unsigned char SensorPowerMode;
//  CropModeEnum CropMode;

//  // Number of allowed split, for frame-range retrieval.
//  int SplitCounter;

//  // Parameters ready by calibration
//  std::vector<double> cos_lookup_table_;
//  std::vector<double> sin_lookup_table_;
//  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
//  double XMLColorTable[HDL_MAX_NUM_LASERS][3];
//  int CalibrationReportedNumLasers;
//  bool CorrectionsInitialized;
//  bool IsCorrectionFromLiveStream;

//  // Sensor parameters presented as rolling data, extracted from enough packets
//  vtkRollingDataAccumulator* rollingCalibrationData;

//  // User configurable parameters
  int NumberOfTrailingFrames;
//  int ApplyTransform;
//  int FiringsSkip;
//  bool IgnoreZeroDistances;
//  bool UseIntraFiringAdjustment;

//  bool CropReturns;
//  bool CropOutside;
//  bool AlreadyWarnAboutCalibration;
//  double CropRegion[6];
//  double distanceResolutionM;

//  std::vector<bool> LaserSelection;
//  unsigned int DualReturnFilter;

//  void SplitFrame(bool force = false);
//  vtkSmartPointer<vtkPolyData> CreateData(vtkIdType numberOfPoints);
//  vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);

//  void Init();
//  void InitTrigonometricTables();
//  void PrecomputeCorrectionCosSin();
//  void LoadCorrectionsFile(const std::string& filename);
//  bool HDL64LoadCorrectionsFromStreamData();
//  bool shouldBeCroppedOut(double pos[3], double theta);

//  void ProcessHDLPacket(unsigned char* data, std::size_t bytesReceived);
//  static bool shouldSplitFrame(uint16_t, int, int&);

//  double ComputeTimestamp(unsigned int tohTime);
//  void ComputeOrientation(double timestamp, vtkTransform* geotransform);
};

#endif // VTKLIDARSOURCEINTERNAL_H
