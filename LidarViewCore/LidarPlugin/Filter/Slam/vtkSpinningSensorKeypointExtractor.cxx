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
#include "vtkSpinningSensorKeypointExtractor.h"

#include <vtkObjectFactory.h>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSpinningSensorKeypointExtractor)

//-----------------------------------------------------------------------------
vtkSpinningSensorKeypointExtractor::vtkSpinningSensorKeypointExtractor()
  : Extractor(std::make_shared<SpinningSensorKeypointExtractor>())
{

}



void vtkSpinningSensorKeypointExtractor::PrintSelf(std::ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Slam Parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->Extractor->Get##param()  << std::endl;

  PrintParameter(NeighborWidth)
  PrintParameter(MinDistanceToSensor)
  PrintParameter(EdgeSinAngleThreshold)

  PrintParameter(PlaneSinAngleThreshold)
  PrintParameter(EdgeDepthGapThreshold)
  PrintParameter(AngleResolution)
  PrintParameter(SaillancyThreshold)
  PrintParameter(FarestKeypointDist)
  PrintParameter(NLasers)

}

