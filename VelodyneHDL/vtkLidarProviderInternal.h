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

  /**
   * @brief LoadCalibration Read a provided calibration file to initialize the sensor's
   * calibration parameters (angles corrections, distances corrections, ...) which will be
   * used later on while processing the packet.
   * @param filename should be garanty to exist, as no check will be perform.
   */

  virtual void LoadCalibration(const std::string& filename) = 0;

  //! Backwward pointer to public class interface
  vtkLidarProvider* Lidar;

  //! Number of previous frames to display with the current frame (concatenation of frames)
  int NumberOfTrailingFrames;

  //! File containing all calibration information
  std::string CalibrationFileName;

  //! Number of laser which can be shoot at the same time or at least in dt < epsilon
  int CalibrationReportedNumLasers;

  //! Indicate if the sensor is calibrated or has been succesfully calibrated
  bool IsCalibrated;

  //! Indicate for each laser if the points obtained by this specific laser
  //! should process/display (true) or ignore (false)
  std::vector<bool> LaserSelection;

  //! Laser distance resolution (quantum) which also correspond to the points precision
  double DistanceResolutionM;

  //! Frequency at which a new frame is obtain. It can be constant or variable
  //! depending on the lidar. Note that it is note necessarily expressed in Hz, in case of
  //! rotational lidar, this correspond to the RPM (Rotation Per Minute).
  double Frequency;

  //! Process/skip points with a zero value distance.
  //! These points correspond to a missing return or a too close return.
  bool IgnoreZeroDistances;

  //! Proccess/skip frame with 0 points
  bool IgnoreEmptyFrames;

  //! Indicate if the vtkLidarProvider::SensorTransform is apply
  bool ApplyTransform;

  //! Fixed transform to apply to the Lidar points.
  //! This transform is always applied before the transform calculated by the interpolator
  //! as it is used to move the frame from the Lidar's coordinate system to another
  //! coordinate system such as that of the GPS/IMU sensor.
  vtkNew<vtkTransform> SensorTransform;

  //! Interpolator used to get a transform for each frame timestamp, this transform
  //! is always applied after the sensor transform.
  vtkSmartPointer<vtkVelodyneTransformInterpolator> Interp;

  //! Indicate which cropping mode should be used.
  vtkLidarProvider::CropModeEnum CropMode;

  //! enable/disable cropping
  bool CropReturns;

  //! If true, the region outside of the area defined by CropRegion is cropped/removed.
  //! If false, the region within the area is cropped/removed.
  bool CropOutside;

  //! Depending on the :CropingMode select this can have different meaning:
  //! - vtkLidarProvider::CropModeEnum::Cartesian it correspond to [X_min, X_max, Y_min, Y_max, Z_min, Z_max]
  //! - vtkLidarProvider::CropModeEnum::Spherical it correspond to [R_min, R_max, THETA_min, THETA_max, PHI_min, PHI_max]
  //! - vtkLidarProvider::CropModeEnum::Spherical -> Note implemented yet
  //! all distance are in cm and all angle are in degree
  double CropRegion[6];

};

#endif // VTKLIDARPROVIDERINTERNAL_H
