#ifndef VTKLIDARPROVIDERINTERNAL_H
#define VTKLIDARPROVIDERINTERNAL_H

#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkPolyData.h>

class vtkVelodyneTransformInterpolator;
class vtkLidarProvider;

class vtkLidarProviderInternal
{
public:
  enum CropModeEnum
  {
    None = 0,
    Cartesian = 1,
    Spherical = 2,
    Cylindric = 3,
  };

  vtkLidarProviderInternal(vtkLidarProvider* obj);

  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrame = 0) = 0;

  // Description:
  // Number of frames behind current frame to read.  Zero indicates only
  // show the current frame.  Negative numbers are invalid.
  void SetNumberOfTrailingFrames(const int numberTrailing);

  virtual int GetNumberOfFrames() = 0;
  virtual std::string GetSensorInformation() = 0;

  std::string GetCalibrationFileName();
  virtual void SetCalibrationFileName(const std::string& filename);
  virtual void LoadCalibration(const std::string& filename) = 0;
  bool GetIsCalibrated();

  int GetNumberOfChannels();
  double GetDistanceResolutionM();
  double GetCurrentRpm();

  int GetIgnoreZeroDistances() const;
  void SetIgnoreZeroDistances(const bool value);

  int GetIgnoreEmptyFrames() const;
  void SetIgnoreEmptyFrames(const bool  value);

  void SetLaserSelection(bool laserSelection[]);
  void GetLaserSelection(bool laserSelection[]);

  // Cropping related
  void SetCropMode(const int mode);
  void SetCropReturns(const bool value);
  void SetCropOutside(const bool value);
  void SetCropRegion(double region[6]);
  void SetCropRegion(const double v0, const double v1,
                     const double v2, const double v3,
                     const double v4, const double v5);

  // Transform related
  bool GetApplyTransform();
  void SetApplyTransform(const bool apply);
  void SetSensorTransform(vtkTransform* t);
  vtkVelodyneTransformInterpolator* GetInterpolator() const;
  void SetInterpolator(vtkVelodyneTransformInterpolator* interpolator);


public:

  vtkLidarProvider* Lidar;
  std::string CalibrationFileName;

//  vtkSmartPointer<vtkPolyData> CurrentDataset;


  bool ApplyTransform;
  vtkNew<vtkTransform> SensorTransform;
  vtkSmartPointer<vtkVelodyneTransformInterpolator> Interp;



  std::vector<bool> LaserSelection;
  double distanceResolutionM;
  double currentRpm;

  int CalibrationReportedNumLasers;
  bool IsCalibrated;
  int NumberOfTrailingFrames;
  bool IgnoreZeroDistances;
  bool IgnoreEmptyFrames;

  CropModeEnum CropMode;
  bool CropReturns;
  bool CropOutside;
  double CropRegion[6];


//  bool shouldBeCroppedOut(double pos[3], double theta);
};

#endif // VTKLIDARPROVIDERINTERNAL_H
