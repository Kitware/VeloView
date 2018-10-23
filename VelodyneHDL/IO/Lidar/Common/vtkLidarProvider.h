// Copyright 2013 Velodyne Acoustics, Inc.
// Copyright 2018 Kitware SAS.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VTKLIDARPROVIDER_H
#define VTKLIDARPROVIDER_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkTransform.h>

#include "LidarPacketInterpreter.h"

//
// Set built-in type.  Creates member Set"name"() (e.g., SetVisibility());
//
#define vtkCustomSetMacro(name,type) \
virtual void Set##name (type _arg) \
  { \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg); \
  if (this->Interpreter->Get##name() != _arg) \
    { \
    this->Interpreter->Set##name(_arg); \
    this->Modified(); \
    } \
  }

//
// Get built-in type.  Creates member Get"name"() (e.g., GetVisibility());
//
#define vtkCustomGetMacro(name,type) \
virtual type Get##name () \
  { \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): returning " << #name " of " << this->Interpreter->Get##name() ); \
  return this->Interpreter->Get##name(); \
  }
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
  virtual vtkPolyData* GetFramePointer(int frameNumber, int wantedNumberOfTrailingFrame = 0) = 0;

  /**
   * @brief GetNumberOfFrames return the number of available frames
   */
  virtual int GetNumberOfFrames() = 0;

  /**
   * @brief GetSensorInformation return some sensor information used for display purposes
   */
  virtual std::string GetSensorInformation() = 0;

  /**
   * @copydoc LidarPacketInterpreter::NumberOfTrailingFrames
   */
  vtkCustomGetMacro(NumberOfTrailingFrames, int)
  vtkCustomSetMacro(NumberOfTrailingFrames, int)

  /**
   * @copydoc LidarPacketInterpreter::CalibrationFileName
   */
  vtkCustomGetMacro(CalibrationFileName, std::string)
  virtual void SetCalibrationFileName(const std::string& filename);

  /**
   * @copydoc LidarPacketInterpreter::IsCalibrated
   */
  vtkCustomGetMacro(IsCalibrated, bool)

  /**
   * @copydoc LidarPacketInterpreter::CalibrationReportedNumLasers
   */
  int GetNumberOfChannels();

  /**
   * \copydoc LidarPacketInterpreter::Frequency
   */
  vtkCustomGetMacro(Frequency, double)

  /**
   * @copydoc LidarPacketInterpreter::DistanceResolutionM
   */
  vtkCustomGetMacro(DistanceResolutionM, double)

  /**
   * @copydoc LidarPacketInterpreter::IgnoreZeroDistances
   */
  vtkCustomGetMacro(IgnoreZeroDistances, bool)
  vtkCustomSetMacro(IgnoreZeroDistances, bool)

  /**
   * @copydoc LidarPacketInterpreter::IgnoreEmptyFrames
   */
  vtkCustomGetMacro(IgnoreEmptyFrames, bool)
  vtkCustomSetMacro(IgnoreEmptyFrames, bool)

  /**
   * @copydoc LidarPacketInterpreter::LaserSelection
   */
  void SetLaserSelection(bool laserSelection[]);
  void GetLaserSelection(bool laserSelection[]);

  /**
   * @copydoc LidarPacketInterpreter::CropMode
   * Due to the restrictions of the Python wrappings, an int is required instead of an enum.
   * vtkLidarProvider::CropModeEnum
   */
  void SetCropMode(const int mode);

  /**
   * @copydoc LidarPacketInterpreter::CropReturns
   */
  vtkCustomGetMacro(CropReturns, bool)
  vtkCustomSetMacro(CropReturns, bool)

  /**
   * @copydoc LidarPacketInterpreter::CropOutside
   */
  vtkCustomGetMacro(CropOutside, bool)
  vtkCustomSetMacro(CropOutside, bool)

  /**
   * @copydoc LidarPacketInterpreter::CropRegion
   */
  void SetCropRegion(double region[6]);
  void SetCropRegion(const double v0, const double v1,
                     const double v2, const double v3,
                     const double v4, const double v5);

  /**
   * @copydoc LidarPacketInterpreter::SensorTransform
   */
  void SetSensorTransform(vtkTransform* t);

  /**
   * @copydoc LidarPacketInterpreter::ApplyTransform
   */
  vtkCustomGetMacro(ApplyTransform, bool)
  vtkCustomSetMacro(ApplyTransform, bool)

  /**
   * @copydoc LidarPacketInterpreter::Interp
   */
  vtkVelodyneTransformInterpolator* GetInterpolator() const;
  void SetInterpolator(vtkVelodyneTransformInterpolator* interpolator);

  /**
   * @brief SetDummyProperty a trick to workaround failure to wrap LaserSelection, this actually only calls Modified,
   * however for some obscure reason, doing the same from python does not have the same effect
   * @todo set how to remove this methode as it is a workaround
   */
  void SetDummyProperty(int);

  void SetGpsTransform(vtkTransform* t);
  void CreateLinearInterpolator();
  void CreateNearestInterpolator();

  // SLAM transforms related
  void AddTransform(double rx, double ry, double rz, double tx, double ty, double tz, double time);
  void LoadTransforms(const std::string& filename);
  void ExportTransforms(const std::string& filename);

  friend class vtkLidarReaderInternal;
  friend class vtkLidarStreamInternal;
protected:
  vtkLidarProvider();
  ~vtkLidarProvider();

  /**
   * @brief SetInterpreter method used to switch the opaque pointer
   */
  void SetInterpreter(LidarPacketInterpreter* interpreter) {this->Interpreter = interpreter;}
  LidarPacketInterpreter* Interpreter;
private:
  vtkLidarProvider(const vtkLidarProvider&); // not implemented
  void operator=(const vtkLidarProvider&); // not implemented

};

#endif // VTKLIDARPROVIDER_H
