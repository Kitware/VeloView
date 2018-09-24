#ifndef VTKLIDARPROVIDER_H
#define VTKLIDARPROVIDER_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkTransform.h>

class vtkVelodyneTransformInterpolator;
class vtkLidarProviderInternal;

class VTK_EXPORT vtkLidarProvider : public vtkPolyDataAlgorithm
{
public:

  enum CropModeEnum
  {
    None = 0,
    Cartesian = 1,
    Spherical = 2,
    Cylindric = 3,
  };

  vtkTypeMacro(vtkLidarProvider, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) {};

  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrame = 0) = 0;

  // Description:
  // Number of frames behind current frame to read.  Zero indicates only
  // show the current frame.  Negative numbers are invalid.
  void SetNumberOfTrailingFrames(const int numberTrailing);

  virtual int GetNumberOfFrames() = 0;

  std::string GetCalibrationFileName();
  virtual void SetCalibrationFileName(const std::string& filename);
  bool GetIsCalibrated();

  // Sensor related
  virtual std::string GetSensorInformation() = 0;
  double GetFrequency();
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
  vtkLidarProvider(const vtkLidarProvider&); // not implemented
  void operator=(const vtkLidarProvider&); // not implemented

  vtkLidarProviderInternal* Internal;
};

#endif // VTKLIDARPROVIDER_H
