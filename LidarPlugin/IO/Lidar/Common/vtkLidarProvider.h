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

#include "vtkLidarPacketInterpreter.h"

/**
 * @brief The vtkLidarProvider class is a pure abstract class which provides a
 * common interface that all Lidar subclasses should implement
 */
class VTK_EXPORT vtkLidarProvider : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkLidarProvider, vtkPolyDataAlgorithm)

  /**
   * @brief GetNumberOfFrames return the number of available frames
   */
  virtual int GetNumberOfFrames() = 0;

  /**
   * @brief GetSensorInformation return some sensor information used for display purposes
   */
  virtual std::string GetSensorInformation();

  /**
   * @copydoc vtkLidarPacketInterpreter::CalibrationFileName
   */
  virtual void SetCalibrationFileName(const std::string& filename);

  /**
   * @brief SetDummyProperty a trick to workaround failure to wrap LaserSelection, this actually only calls Modified,
   * however for some obscure reason, doing the same from python does not have the same effect
   * @todo set how to remove this methode as it is a workaround
   */
  void SetDummyProperty(int);

  /**
   * @copydoc vtkLidarPacketInterpreter
   */
  vtkGetObjectMacro(Interpreter, vtkLidarPacketInterpreter)
  virtual void SetInterpreter(vtkLidarPacketInterpreter *);

  /**
   * @brief The port the Lidar is listening to
   */
  virtual int GetLidarPort() = 0;
  virtual void SetLidarPort(int _arg) = 0;

  vtkGetMacro(DetectFrameDropping, bool)
  vtkSetMacro(DetectFrameDropping, bool)

  vtkMTimeType GetMTime() override;

  friend class vtkLidarReaderInternal;
  friend class vtkLidarStreamInternal;
protected:
  vtkLidarProvider();
  ~vtkLidarProvider() = default;
  int RequestInformation(vtkInformation *request,
                         vtkInformationVector **inputVector,
                         vtkInformationVector *outputVector) override;

  int FillOutputPortInformation(int port, vtkInformation* info) override;

  //! Indicate if we should detect that some frames are dropped
  bool DetectFrameDropping = false;

  //! Last Frame processed, this is important if we want to detect frame dropping
  int LastFrameProcessed = 0;

  //! Interpret the packet to create a frame, all the magic happen here
  vtkLidarPacketInterpreter* Interpreter = nullptr;

  //! The calibrationFileName to used and set to the Interpreter once one has been set
  std::string CalibrationFileName = "";
private:
  vtkLidarProvider(const vtkLidarProvider&) = delete;
  void operator=(const vtkLidarProvider&) = delete;

};

#endif // VTKLIDARPROVIDER_H
