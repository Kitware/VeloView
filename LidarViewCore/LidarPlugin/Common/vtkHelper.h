//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre (pierre.guilbert@kitware.com)
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

#ifndef VTK_HELPER_H
#define VTK_HELPER_H

// STD
#include <string>
// VTK
#include <vtkSmartPointer.h>

template <typename T>
vtkSmartPointer<T> createArray(const std::string& Name, int NumberOfComponents = 1, int NumberOfTuples = 0)
{
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->SetNumberOfComponents(NumberOfComponents);
  array->SetNumberOfTuples(NumberOfTuples);
  array->SetName(Name.c_str());
  return array;
}

#endif // VTK_HELPER_H
