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

#ifndef VTKLASFILEWRITER_H
#define VTKLASFILEWRITER_H

#include <vtkDataObjectAlgorithm.h>
#include <vtkPolyData.h>
#include <chrono>

#include "LASFileWriter.h"

// This class should be turned into something more generic.
// If you need to write point clouds to another format than LAS,
// please consider doing that.
// The aim is to allow setting another file writer than
// LASFileWriter at run time (this implies reworking the xml).

// How does it works ?
// Here are the expected function calls:
// RequestInformation()
// First pass:
// for (i = firstFrame; i < lastFrame; i++) {
//   RequestUpdateExtent()
//   RequestData()
// }
// Second and last pass:
// for (i = firstFrame; i < lastFrame; i++) {
//   RequestUpdateExtent()
//   RequestData()
// }

// Currently we require the input to be using UTM coordinates
class VTK_EXPORT vtkLASFileWriter : public vtkDataObjectAlgorithm
{
public:
  static vtkLASFileWriter* New();
  vtkTypeMacro(vtkLASFileWriter, vtkDataObjectAlgorithm)

  vtkSetStringMacro(FileName)
  vtkGetStringMacro(FileName)

  vtkSetMacro(SkipMetaDataPass, bool)

  vtkSetMacro(FirstFrame, int)
  vtkGetMacro(FirstFrame, int)

  vtkSetMacro(WriteSRS, bool)
  vtkGetMacro(WriteSRS, bool)

  vtkSetMacro(WriteColor, bool)
  vtkGetMacro(WriteColor, bool)

  vtkSetMacro(LastFrame, int)
  vtkGetMacro(LastFrame, int)

  vtkSetMacro(FrameStride, int)
  vtkGetMacro(FrameStride, int)

  vtkSetMacro(ExportType, int)
  vtkGetMacro(ExportType, int)

  vtkSetMacro(InOutSignedUTMZone, int)
  vtkGetMacro(InOutSignedUTMZone, int)

  vtkSetVector3Macro(Offset, double);
  vtkGetVector3Macro(Offset, double);

  // This method is the "entry point" of the writing process.
  int Write();

  // For debug purpose only:
  void Modified() override; // from vtkObject
  // For debug purpose only:
  int ProcessRequest(
                  vtkInformation*,
                  vtkInformationVector**,
                  vtkInformationVector*) VTK_OVERRIDE; // from vtkAlgorithm
  // For debug purpose only:
  void Update() VTK_OVERRIDE; // from vtkAlgorithm

  enum
  {
    EXPORT_UTM = 0,
    EXPORT_LATLONG = 1,
  };

protected:
  vtkLASFileWriter();
  ~vtkLASFileWriter();

  int RequestInformation(vtkInformation* request,
                         vtkInformationVector** inputVector,
                         vtkInformationVector* outputVector) VTK_OVERRIDE;
  int RequestUpdateExtent(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector) VTK_OVERRIDE;
  int RequestData(vtkInformation *,
                  vtkInformationVector **,
                  vtkInformationVector *) VTK_OVERRIDE;

private:
  vtkLASFileWriter(const vtkLASFileWriter&) = delete;
  void operator =(const vtkLASFileWriter&) = delete;

  char* FileName = nullptr;
  bool WriteSRS = true;
  bool WriteColor = false;
  int FirstFrame = 0;
  int LastFrame = -1; // negative numbers can be used à la Python list indexes
  int FrameStride = 1;
  int NumberOfFrames = 0;
  int CurrentFrame = 0;
  int ExportType = EXPORT_UTM;
  int InOutSignedUTMZone = 0;
  bool SkipMetaDataPass = false;
  int CurrentPass = 0; // pass 0 is to compute the header, pass 1 is to write
  static const int PassCount = 2;
  // This Offset must be applied to the coordinates of the points inside the
  // vtkPolyData in order to find their correct position.
  double Offset[3]; // TODO: decide if should be named "Origin"

  std::chrono::steady_clock::time_point Start;
  std::chrono::steady_clock::time_point End;

  int PolyFirstPass(vtkPolyData *polyData);
  int PolySecondPass(vtkPolyData *polyData);

  LASFileWriter LASWriter;
};

#endif
