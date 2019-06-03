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

#ifndef VTK_FISH_EYE_PROJECTOR_H
#define VTK_FISH_EYE_PROJECTOR_H

// VTK
#include <vtkImageAlgorithm.h>

// EIGEN
#include <Eigen/Dense>

class VTK_EXPORT vtkFishEyeProjector : public vtkImageAlgorithm
{
public:
  static vtkFishEyeProjector *New();
  vtkTypeMacro(vtkFishEyeProjector, vtkImageAlgorithm)

protected:
  vtkFishEyeProjector();

  int FillInputPortInformation(int port, vtkInformation *info) override;
  int FillOutputPortInformation(int port, vtkInformation *info) override;
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

private:
  vtkFishEyeProjector(const vtkFishEyeProjector&) = delete;
  void operator=(const vtkFishEyeProjector&) = delete;

  //! Parameters of the fisheye camera model
  Eigen::Matrix<double, 15, 1> W = Eigen::Matrix<double, 15, 1>::Zero();
};

#endif // VTK_FISH_EYE_PROJECTOR_H
