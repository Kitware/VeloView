#ifndef VTKLIDARPROVIDER_H
#define VTKLIDARPROVIDER_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkTransform.h>

class vtkVelodyneTransformInterpolator;

class vtkLidarProviderInternal;

class vtkLidarProvider : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkLidarProvider, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) {};

  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrame = 0);

  // Description:
  // Number of frames behind current frame to read.  Zero indicates only
  // show the current frame.  Negative numbers are invalid.
  void SetNumberOfTrailingFrames(const int numberTrailing);

  virtual int GetNumberOfFrames();

  std::string GetCalibrationFileName();
  void SetCalibrationFileName(const std::string& filename);

  // Sensor related
  std::string GetSensorInformation();
  bool GetIsCalibrated();
  double GetCurrentRpm();
  double GetDistanceResolutionM();


  int GetIgnoreZeroDistances() const;
  void SetIgnoreZeroDistances(const bool value);

  int GetIgnoreEmptyFrames() const;
  void SetIgnoreEmptyFrames(const bool  value);

  int GetNumberOfChannels();

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
  void SetSensorTransform(vtkTransform* t);
  bool GetApplyTransform();
  void SetApplyTransform(const bool apply);

  vtkVelodyneTransformInterpolator* GetInterpolator() const;
  void SetInterpolator(vtkVelodyneTransformInterpolator* interpolator);

  // A trick to workaround failure to wrap LaserSelection
  void SetDummyProperty(int);


protected:
  vtkLidarProvider();
  ~vtkLidarProvider();

  void SetPimpInternal(vtkLidarProviderInternal* internal) {this->Internal = internal;};

private:
  vtkLidarProvider(const vtkLidarProvider&) = delete;
  void operator=(const vtkLidarProvider&) = delete;

  vtkLidarProviderInternal* Internal;
};

#endif // VTKLIDARPROVIDER_H
