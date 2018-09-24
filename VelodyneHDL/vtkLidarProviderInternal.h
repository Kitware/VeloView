#ifndef VTKLIDARPROVIDERINTERNAL_H
#define VTKLIDARPROVIDERINTERNAL_H

#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkPolyData.h>

#include "vtkLidarProvider.h"

class vtkVelodyneTransformInterpolator;
struct vtkLidarProviderInternal
{
  vtkLidarProviderInternal(vtkLidarProvider* obj);

  virtual void LoadCalibration(const std::string& filename) = 0;

  vtkLidarProvider* Lidar;

  int NumberOfTrailingFrames;

  std::string CalibrationFileName;
  int CalibrationReportedNumLasers;
  bool IsCalibrated;

  std::vector<bool> LaserSelection;
  double distanceResolutionM;
  double Frequency;

  bool IgnoreZeroDistances;
  bool IgnoreEmptyFrames;

  bool ApplyTransform;
  vtkNew<vtkTransform> SensorTransform;
  vtkSmartPointer<vtkVelodyneTransformInterpolator> Interp;

  vtkLidarProvider::CropModeEnum CropMode;
  bool CropReturns;
  bool CropOutside;
  double CropRegion[6];

};

#endif // VTKLIDARPROVIDERINTERNAL_H
