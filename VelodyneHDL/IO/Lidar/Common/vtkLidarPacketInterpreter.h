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

#ifndef VTKLIDARPROVIDERINTERNAL_H
#define VTKLIDARPROVIDERINTERNAL_H

#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>
#include <vtkPolyData.h>
#include <vtkAlgorithm.h>

class vtkTransform;

class VTK_EXPORT  vtkLidarPacketInterpreter : public vtkAlgorithm
{
public:
  vtkTypeMacro(vtkLidarPacketInterpreter, vtkAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent) {}

  /**
   * @brief The CropModeEnum enum to select the cropping mode
   */
  enum CROP_MODE
  {
    None = 0,       /*!< 0 */
    Cartesian = 1,  /*!< 1 */
    Spherical = 2,  /*!< 2 */
    Cylindric = 3,  /*!< 3 */
  };

  /**
   * @brief LoadCalibration read a provided calibration file to initialize the sensor's
   * calibration parameters (angles corrections, distances corrections, ...) which will be
   * used later on while processing the packet.
   * @param filename should be garanty to exist, as no check will be perform.
   */
  virtual void LoadCalibration(const std::string& filename) = 0;

  /**
   * @brief GetCalibrationTable return a table conttaining all information related to the sensor
   * calibration.
   */
  virtual vtkSmartPointer<vtkTable> GetCalibrationTable() { return this->CalibrationData.Get(); }

  /**
   * @brief ProcessPacket process the data packet to create incrementaly the frame.
   * Each time a packet is processed by the function, the points which are encoded
   * in the packet are decoded using the calibration information and add to the frame. A warning
   * should be raise in case the calibration information does not match the data
   * (ex: factory field, number of laser, ...)
   * @param data raw data packet
   * @param bytesReceived size of the data packet
   * @param startPosition offset in the data packet used when a frame start in the middle of a packet
   */
  virtual void ProcessPacket(unsigned char const * data, unsigned int dataLength, int startPosition = 0) = 0;

  /**
   * @brief SplitFrame take the current frame under construction and place it in another buffer
   * @param force force the split even if the frame is empty
   */
  virtual bool SplitFrame(bool force = false);

  /**
   * @brief CreateNewEmptyFrame construct a empty polyData with the right DataArray and allocate some
   * space. No CellArray should be created as it can be create once the frame is ready.
   * @param numberOfPoints indicate the space to allocate @todo change the meaning
   */
  virtual vtkSmartPointer<vtkPolyData> CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints = 0) = 0;

  /**
   * @brief PreProcessPacket is use to construct the frame index and get some corretion
   * or live calibration comming from the data. A warning should be raise in case the calibration
   * information does not match the data (ex: factory field, number of laser, ...)
   * @param data raw data packet
   * @param dataLength size of the data packet
   * @param isNewFrame[out] indicate if a new frame should be created
   * @param framePositionInPacket[out] indicate the offset of the new, I
   */
  virtual void PreProcessPacket(unsigned char const * data, unsigned int dataLength,
                         bool& isNewFrame, int& framePositionInPacket) = 0;

  /**
   * @brief IsLidarPacket check if the given packet is really a lidar packet
   * @param data raw data packet
   * @param dataLength size of the data packet
   */
  virtual bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) = 0;

  /**
   * @brief ResetCurrentFrame reset all information to handle some new frame. This reset the
   * frame container, some information about the current frame, guesses about the sensor type, etc
   */
  virtual void ResetCurrentFrame() { this->CurrentFrame = this->CreateNewEmptyFrame(0); }

  /**
   * @brief isNewFrameReady check if a new frame is ready
   */
  bool IsNewFrameReady() { return this->Frames.size(); }

  /**
   * @brief GetLastFrameAvailable return the last frame that have been process completely
   * @return
   */
  vtkSmartPointer<vtkPolyData> GetLastFrameAvailable() { return this->Frames.back(); }

  /**
   * @brief ClearAllFramesAvailable delete all frames that have been process
   */
  void ClearAllFramesAvailable() { this->Frames.clear(); }

  /**
   * @brief GetSensorInformation return information to display to the user
   * @return
   */
  virtual std::string GetSensorInformation() = 0;

  virtual int GetNumberOfChannels() { return this->CalibrationReportedNumLasers; }

  vtkGetMacro(CalibrationFileName, std::string)
  vtkSetMacro(CalibrationFileName, std::string)

  vtkGetMacro(CalibrationReportedNumLasers, int)
  vtkSetMacro(CalibrationReportedNumLasers, int)

  vtkGetMacro(IsCalibrated, bool)
  vtkSetMacro(IsCalibrated, bool)

  vtkGetMacro(TimeOffset, double)
  vtkSetMacro(TimeOffset, double)

  /**
   * @copydoc LidarPacketInterpreter::LaserSelection
   */
  virtual void SetLaserSelection(const bool* v) { this->LaserSelection = std::vector<bool>(v, v + this->CalibrationReportedNumLasers); }
  virtual void GetLaserSelection(bool* v) { std::copy(this->LaserSelection.begin(), this->LaserSelection.end(), v);}
  virtual void SetLaserSelection(const std::vector<bool>& v) { this->LaserSelection = v; }
  virtual std::vector<bool> GetLaserSelection() const { return this->LaserSelection; }

  vtkGetMacro(DistanceResolutionM, double)
  vtkSetMacro(DistanceResolutionM, double)

  vtkGetMacro(Frequency, double)
  vtkSetMacro(Frequency, double)

  vtkGetMacro(IgnoreZeroDistances, bool)
  vtkSetMacro(IgnoreZeroDistances, bool)

  vtkGetMacro(IgnoreEmptyFrames, bool)
  vtkSetMacro(IgnoreEmptyFrames, bool)

  vtkGetMacro(ApplyTransform, bool)
  vtkSetMacro(ApplyTransform, bool)

  vtkGetObjectMacro(SensorTransform, vtkTransform)
  virtual void SetSensorTransform(vtkTransform *);

  vtkGetMacro(CropMode, int)
  vtkSetMacro(CropMode, int)

  vtkGetMacro(CropOutside, bool)
  vtkSetMacro(CropOutside, bool)

  vtkSetVector6Macro(CropRegion, double)

  vtkMTimeType GetMTime() override;

protected:
  /**
   * @brief shouldBeCroppedOut Check if a point is inside a region of interest determined
   * either by its cartesian coordinates system or spherical coordinates system.
   * @param pos cartesian coordinates of the point to be check
   * @param theta azimuth of the point to be check. Avoid to recompute this information using an atan2 function
   */
  bool shouldBeCroppedOut(double pos[3], double theta);

  //! Buffer to store the frame once they are ready
  std::vector<vtkSmartPointer<vtkPolyData> > Frames;

  //! Frame under construction
  vtkSmartPointer<vtkPolyData> CurrentFrame;

  //! File containing all calibration information
  std::string CalibrationFileName = "";

  ///! Calibration data store in a table
  vtkNew<vtkTable> CalibrationData;

  //! Number of laser which can be shoot at the same time or at least in dt < epsilon
  int CalibrationReportedNumLasers = -1;

  //! Indicate if the sensor is calibrated or has been succesfully calibrated
  bool IsCalibrated = false;

  //! TimeOffset in seconds relative to the system clock
  double TimeOffset = 0.;

  //! Indicate for each laser if the points obtained by this specific laser
  //! should process/display (true) or ignore (false)
  std::vector<bool> LaserSelection;

  //! Laser distance resolution (quantum) which also correspond to the points precision
  double DistanceResolutionM = 0;

  //! Frequency at which a new frame is obtain. It can be constant or variable
  //! depending on the lidar. Note that it is note necessarily expressed in Hz, in case of
  //! rotational lidar, this correspond to the RPM (Rotation Per Minute).
  double Frequency = 0;

  //! Process/skip points with a zero value distance.
  //! These points correspond to a missing return or a too close return.
  bool IgnoreZeroDistances = true;

  //! Proccess/skip frame with 0 points
  bool IgnoreEmptyFrames = false;

  //! Indicate if the vtkLidarProvider::SensorTransform is apply
  bool ApplyTransform = false;

  //! Fixed transform to apply to the Lidar points.
  vtkTransform* SensorTransform = nullptr;

  //! Indicate which cropping mode should be used.
  int CropMode = CROP_MODE::None;

  //! If true, the region outside of the area defined by CropRegion is cropped/removed.
  //! If false, the region within the area is cropped/removed.
  bool CropOutside = false;

  //! Depending on the :CropingMode select this can have different meaning:
  //! - vtkLidarProvider::CropModeEnum::Cartesian it correspond to [X_min, X_max, Y_min, Y_max, Z_min, Z_max]
  //! - vtkLidarProvider::CropModeEnum::Spherical it correspond to [R_min, R_max, THETA_min, THETA_max, PHI_min, PHI_max]
  //! - vtkLidarProvider::CropModeEnum::Spherical -> Note implemented yet
  //! all distance are in cm and all angle are in degree
  double CropRegion[6] = {0,0,0,0,0,0};

  vtkLidarPacketInterpreter() = default;
  virtual ~vtkLidarPacketInterpreter() = default;

private:
  vtkLidarPacketInterpreter(const vtkLidarPacketInterpreter&) = delete;
  void operator=(const vtkLidarPacketInterpreter&) = delete;

};

#endif // VTKLIDARPROVIDERINTERNAL_H
