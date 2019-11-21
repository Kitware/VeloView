//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Guilbert Pierre
// Data: 04-15-2019
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

#ifndef VTK_OPENCV_VIDEO_READER_H
#define VTK_OPENCV_VIDEO_READER_H

// VTK
#include <vtkPolyData.h>
#include <vtkImageAlgorithm.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>

class vtkOpenCVVideoReaderInternal;

class VTK_EXPORT vtkOpenCVVideoReader : public vtkImageAlgorithm
{
public:
  static vtkOpenCVVideoReader *New();
  vtkTypeMacro(vtkOpenCVVideoReader, vtkImageAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent) override;

  int GetNumberOfFrames();

  void SetFileName(const char* filename);

  void SetTimeOffset(double argTs);
  double GetTimeOffset();
protected:
  vtkOpenCVVideoReader();
  ~vtkOpenCVVideoReader() override;

  // Request data / information VTK mechanism
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;
  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkOpenCVVideoReaderInternal* Internal;
  vtkOpenCVVideoReader(const vtkOpenCVVideoReader&);
  void operator=(const vtkOpenCVVideoReader&);
};

#endif // VTK_OPENCV_VIDEO_READER_H
