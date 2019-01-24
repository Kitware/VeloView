// Copyright 2019 Kitware SAS.
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

#ifndef TemporalTransformsAPPLIER_H
#define TemporalTransformsAPPLIER_H

#include <vtkNew.h>
#include <vtkPolyDataAlgorithm.h>

#include "vtkVelodyneTransformInterpolator.h"

/**
 * @brief The vtkTemporalTransformsApplier take 2 inputs : a vtkTemporalTransforms which
 * contains the poses and orientation of the sensor and a polydata.
 * The filter will apply the transform corresponding to the pipeline time or to the current
 * pointcloud time. This Option can be manage with the InterpolateEachPoint paramater
 */
class VTK_EXPORT vtkTemporalTransformsApplier : public vtkPolyDataAlgorithm
{
public:
  static vtkTemporalTransformsApplier* New();
  vtkTypeMacro(vtkTemporalTransformsApplier,vtkPolyDataAlgorithm)

  //@{
  /**
   * @copydoc vtkTemporalTransformsApplier::InterpolationType
   */
  int GetInterpolationType() { return this->Interpolator->GetInterpolationType(); }
  void SetInterpolationType(int value) { this->Interpolator->SetInterpolationType(value); };
  //@}

  //@{
  /**
   * @copydoc vtkTemporalTransformsApplier::InterpolateEachPoint
   */
  vtkGetMacro(InterpolateEachPoint, bool)
  vtkSetMacro(InterpolateEachPoint, bool)
  //@}

  /**
   * @brief Override GetMTime() because we depend on the TransformInterpolator
   * which may be modified outside of this class.
   */
  vtkMTimeType GetMTime();

protected:
  vtkTemporalTransformsApplier();

  int RequestInformation(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector);

  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector);

private:
    //! Indicate if a different transform should be apply to each point,
  //! or if the same transform should be apply to the whole point cloud.
  //! In the first case you must specify the array from the pointcloud containing the
  //! timestamp with 'SetInputArrayToProcess'
  bool InterpolateEachPoint;

  //! Interpolator used to get the right transform
  vtkSmartPointer<vtkVelodyneTransformInterpolator> Interpolator;

  vtkTemporalTransformsApplier(const vtkTemporalTransformsApplier&) /*= delete*/;
  void operator =(const vtkTemporalTransformsApplier&) /*= delete*/;
};

#endif // TemporalTransformsAPPLIER_H
