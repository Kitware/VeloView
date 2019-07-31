//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
// Data: 03-27-2019
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
//=========================================================================

#ifndef VTK_CAMERA_PROJECTOR_H
#define VTK_CAMERA_PROJECTOR_H

// VTK
#include <vtkImageAlgorithm.h>

// EIGEN
#include <Eigen/Dense>

// LOCAL
#include "CameraModel.h"

class VTK_EXPORT vtkCameraProjector : public vtkImageAlgorithm
{
public:
  static vtkCameraProjector *New();
  vtkTypeMacro(vtkCameraProjector, vtkImageAlgorithm)

  void SetFileName(const std::string &argfilename);

protected:
  vtkCameraProjector();

  int FillInputPortInformation(int port, vtkInformation *info) override;
  int FillOutputPortInformation(int port, vtkInformation *info) override;
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

private:
  vtkCameraProjector(const vtkCameraProjector&) = delete;
  void operator=(const vtkCameraProjector&) = delete;

  //! Camera model
  CameraModel Model;

  //! File containing the camera model and parameters
  std::string Filename;

  //! Type of the camera model projection
  ProjectionType Type = ProjectionType::FishEye;

  //! Name of the color array
  std::string ColorArrayName = "RGB";
};

#endif // VTK_CAMERA_PROJECTOR_H
