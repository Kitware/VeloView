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

#ifndef VTKTEMPORALTRANSFORMSWRITER_H
#define VTKTEMPORALTRANSFORMSWRITER_H

// #include <vtkPolyDataAlgorithm.h>
#include <vtkPolyDataWriter.h>

// Inspired by vtkObjWriter
class VTK_EXPORT vtkTemporalTransformsWriter : public vtkPolyDataWriter
{
public:
  static vtkTemporalTransformsWriter* New();
  vtkTypeMacro(vtkTemporalTransformsWriter, vtkPolyDataWriter)

  vtkSetStringMacro(FileName)
  vtkGetStringMacro(FileName)

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

protected:
  vtkTemporalTransformsWriter() = default;
  ~vtkTemporalTransformsWriter();

private:
  vtkTemporalTransformsWriter(const vtkTemporalTransformsWriter&) = delete;
  void operator =(const vtkTemporalTransformsWriter&) = delete;

  char* FileName = nullptr;
};

#endif
