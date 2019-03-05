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

#ifndef VTKLIDARSTREAM_H
#define VTKLIDARSTREAM_H

#include "vtkLidarProvider.h"

class vtkLidarStreamInternal;

class VTK_EXPORT vtkLidarStream : public vtkLidarProvider
{
public:
  static vtkLidarStream* New();
  vtkTypeMacro(vtkLidarStream, vtkLidarProvider)

  int GetNumberOfFrames() override;

  void Poll();

  void Start();
  void Stop();

  void UnloadFrames();

  int GetCacheSize();
  void SetCacheSize(int cacheSize);

  /**
   * @copydoc vtkLidarStreamInternal::OutputFileName
   */
  std::string GetOutputFile();
  void SetOutputFile(const std::string& filename);

  /**
   * @copydoc NetworkSource::LIDARPort
   */
  int GetLIDARPort();
  void SetLIDARPort(const int);

  /**
   * @copydoc NetworkSource::GPSPort
   */
  int GetGPSPort();
  void SetGPSPort(const int);

  /**
   * @copydoc NetworkSource::ForwardedIpAddress
   */
  std::string GetForwardedIpAddress();
  void SetForwardedIpAddress(const std::string& ipAddress);

  /**
   * @copydoc NetworkSource::ForwardedLIDARPort
   */
  int GetForwardedLIDARPort();
  void SetForwardedLIDARPort(const int);

  /**
   * @copydoc NetworkSource::ForwardedLIDARPort
   */
  int GetForwardedGPSPort();
  void SetForwardedGPSPort(const int);

  void EnableGPSListening(const bool);

  /**
   * @copydoc NetworkSource::IsForwarding
   */
  bool GetIsForwarding();
  void SetIsForwarding(const bool);

  /**
   * @copydoc NetworkSource::IsCrashAnalysing
   */
  bool GetIsCrashAnalysing();
  void SetIsCrashAnalysing(bool value);

  /**
   * @brief GetNeedsUpdate
   * @return true if a new frame is ready
   */
  bool GetNeedsUpdate();

protected:
  vtkLidarStream();
  ~vtkLidarStream();

  virtual int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);

  virtual int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*);

private:
  vtkLidarStreamInternal* Internal;
  vtkLidarStream(const vtkLidarStream&); // not implemented
  void operator=(const vtkLidarStream&); // not implemented

};

#endif // VTKLIDARSTREAM_H
