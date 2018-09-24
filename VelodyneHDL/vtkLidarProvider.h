#ifndef VTKLIDARPROVIDER_H
#define VTKLIDARPROVIDER_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkTransform.h>

/**

@defgroup  Lidar Lidar

All class which implement the vtkLidarProvider interface should implement the PIMPL with
inheritance idiom also know as opaque pointer or q pointer (in the Qt framework).

The raisons are:
- provide a stable ABI
- reduce compilation time

To implement the pimpl idiom in a class which inherit from vtkLidarProvider you must:
- inherit from your superclass
- have a private attribute which will be the opaque pointer (generaly named Internal)
- the opaque pointer should inherit from the superclass opaque pointer
- in the constructor, allocate the opaque pointer and then call superclass::SetPimpInternal()
  which will make your super class point to your opaque pointer
- no need to delete the allocate pointer in the destructor as the superclass will handle it
- if you do not implement a final classes, you must provide the same setPimplInternal method to your subclass

This implementation is a little bit different from the Qt onces, but has the advantage to
avoid to reinterpret cast the opaque pointer each time. The only drawback is that your object
will actually have multiple opaque pointer (one per inheritance level), but as we make them
point to the same object by calling SetPimpInternal this doesn't cause any problem.
  */

class vtkVelodyneTransformInterpolator;
class vtkLidarProviderInternal;


/**
 * @brief The vtkLidarProvider class is a pure abstract class which provides a
 * common interface that all Lidar subclasses should implement
 */
class VTK_EXPORT vtkLidarProvider : public vtkPolyDataAlgorithm
{
public:

  /**
   * @brief The CropModeEnum enum to select the cropping mode
   */
  enum CropModeEnum
  {
    None = 0,       /*!< 0 */
    Cartesian = 1,  /*!< 1 */
    Spherical = 2,  /*!< 2 */
    Cylindric = 3,  /*!< 3 */
  };

  vtkTypeMacro(vtkLidarProvider, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) {};

  /**
   * @brief GetFrame returns the requested frame concatenated with the n previous frames
   * when they exist
   * @param frameNumber start at 0
   * @param wantedNumberOfTrailingFrame number of frame preceding the asked one
   * to return. Negative numbers are invalid.
   */
  virtual vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrame = 0) = 0;

  /**
   * @brief GetNumberOfFrames return the number of available frames
   */
  virtual int GetNumberOfFrames() = 0;

  /**
   * @brief GetSensorInformation return some sensor information used for display purposes
   */
  virtual std::string GetSensorInformation() = 0;

  /**
   * @copydoc vtkLidarProviderInternal::NumberOfTrailingFrames
   */
  void SetNumberOfTrailingFrames(const int numberTrailing);

  /**
   * @copydoc vtkLidarProviderInternal::CalibrationFileName
   */
  std::string GetCalibrationFileName();

  /**
   * @copydoc vtkLidarProviderInternal::CalibrationFileName
   */
  virtual void SetCalibrationFileName(const std::string& filename);

  /**
   * @copydoc vtkLidarProviderInternal::IsCalibrated
   */
  bool GetIsCalibrated();

  /**
   * @copydoc vtkLidarProviderInternal::CalibrationReportedNumLasers
   */
  int GetNumberOfChannels();

  /**
   * \copydoc vtkLidarProviderInternal::Frequency
   */
  double GetFrequency();

  /**
   * @copydoc vtkLidarProviderInternal::DistanceResolutionM
   */
  double GetDistanceResolutionM();

  /**
   * @copydoc vtkLidarProviderInternal::IgnoreZeroDistances
   */
  int GetIgnoreZeroDistances() const;
  void SetIgnoreZeroDistances(const bool value);

  /**
   * @copydoc vtkLidarProviderInternal::IgnoreEmptyFrames
   */
  int GetIgnoreEmptyFrames() const;
  void SetIgnoreEmptyFrames(const bool  value);

  /**
   * @copydoc vtkLidarProviderInternal::LaserSelection
   */
  void SetLaserSelection(bool laserSelection[]);
  void GetLaserSelection(bool laserSelection[]);

  /**
   * @copydoc vtkLidarProviderInternal::CropMode
   * Due to the restrictions of the Python wrappings, an is is required instead of an enum.
   * vtkLidarProvider::CropModeEnum
   */
  void SetCropMode(const int mode);

  /**
   * @copydoc vtkLidarProviderInternal::CropReturns
   */
  void SetCropReturns(const bool value);

  /**
   * @copydoc vtkLidarProviderInternal::CropOutside
   */
  void SetCropOutside(const bool value);

  /**
   * @copydoc vtkLidarProviderInternal::CropRegion
   */
  void SetCropRegion(double region[6]);
  void SetCropRegion(const double v0, const double v1,
                     const double v2, const double v3,
                     const double v4, const double v5);

  /**
   * @copydoc vtkLidarProviderInternal::SensorTransform
   */
  void SetSensorTransform(vtkTransform* t);

  /**
   * @copydoc vtkLidarProviderInternal::ApplyTransform
   */
  void SetApplyTransform(const bool apply);
  bool GetApplyTransform();

  /**
   * @copydoc vtkLidarProviderInternal::Interp
   */
  vtkVelodyneTransformInterpolator* GetInterpolator() const;
  void SetInterpolator(vtkVelodyneTransformInterpolator* interpolator);

  /**
   * @brief SetDummyProperty a trick to workaround failure to wrap LaserSelection, this actually only calls Modified,
   * however for some obscure reason, doing the same from python does not have the same effect
   * @todo set how to remove this methode as it is a workaround
   */
  void SetDummyProperty(int);


protected:
  vtkLidarProvider();
  ~vtkLidarProvider();

  /**
   * @brief SetPimpInternal method used to switch the opaque pointer
   */
  void SetPimpInternal(vtkLidarProviderInternal* internal) {this->Internal = internal;};

private:
  vtkLidarProvider(const vtkLidarProvider&); // not implemented
  void operator=(const vtkLidarProvider&); // not implemented

  vtkLidarProviderInternal* Internal;
};

#endif // VTKLIDARPROVIDER_H
