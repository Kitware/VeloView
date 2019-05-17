//=========================================================================
//
// Copyright 2018 Kitware, Inc.
// Authors: Guilbert Pierre (spguilbert@gmail.com)
//         Laurenson Nick (nlaurenson5@gmail.com)
// Date: 03-27-2018
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
#ifndef VTKSpinningSensorKeypointExtractor_H
#define VTKSpinningSensorKeypointExtractor_H

#include <vtkObject.h>

#include "SpinningSensorKeypointExtractor.h"

//
// Set built-in type.  Creates member Set"name"() (e.g., SetVisibility());
//
#undef vtkCustomSetMacro(name,type)
#define vtkCustomSetMacro(name,type) \
virtual void Set##name (type _arg) \
{ \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg); \
  if (this->Extractor->Get##name() != _arg) \
  { \
    this->Extractor->Set##name(_arg); \
    this->Modified(); \
  } \
}

/**
 * @brief The class is a paraview wrapper for SpinningSensorKeypointExtractor in order to enable the
 *  creation of a proxy. This way we get a free GUI.
 *  It should only implement setter for the proxy, and a getter to the underlying keypointExtractor.
 */
class VTK_EXPORT vtkSpinningSensorKeypointExtractor : public vtkObject
{
public:
  static vtkSpinningSensorKeypointExtractor *New();
  vtkTypeMacro(vtkSpinningSensorKeypointExtractor, vtkObject)
  void PrintSelf(ostream& os, vtkIndent indent);

  vtkCustomSetMacro(NeighborWidth, int)

  vtkCustomSetMacro(MinDistanceToSensor, double)

  vtkCustomSetMacro(EdgeSinAngleThreshold, double)

  vtkCustomSetMacro(PlaneSinAngleThreshold, double)

  vtkCustomSetMacro(EdgeDepthGapThreshold, double)

  vtkCustomSetMacro(AngleResolution, double)

  vtkCustomSetMacro(SaillancyThreshold, double)

  std::shared_ptr<SpinningSensorKeypointExtractor> GetExtractor() { return Extractor; }

protected:
  vtkSpinningSensorKeypointExtractor();

  std::shared_ptr<SpinningSensorKeypointExtractor> Extractor;

private:
  vtkSpinningSensorKeypointExtractor(const vtkSpinningSensorKeypointExtractor&) = delete;
  void operator = (const vtkSpinningSensorKeypointExtractor&) = delete;
};

#endif // VTKSpinningSensorKeypointExtractor_H
