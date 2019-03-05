// Copyright 2013 Velodyne Acoustics, Inc.
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

#ifndef __testHelpers_h
#define __testHelpers_h

#include <sstream>
#include <vector>

class vtkErrorObserver;
class vtkLidarReader;
class vtkLidarStream;
class vtkPolyData;

// Represent every processing options as a bit. Options are in the following order:
// IntensityCorrected (bit 0), IgnoreZeroDistances (bit 1),
// IntraFiringAdjust (bit 2), IgnoreEmptyFrames (bit 3)
// This allows to easily test and add options if necessary in the future.
typedef int vvProcessingOptionsType;

// Helper functions
bool compare(const double* const a, const double* const b, const size_t N, double epsilon);

std::vector<int> parseOptions(vvProcessingOptionsType currentOptions, int numProcessingOptions);

template <size_t N>
bool compare(double const (&a)[N], double const (&b)[N], double epsilon)
{
  return compare(a, b, N, epsilon);
}

std::string toString(const double* const d, const size_t N);

template <size_t N>
std::string toString(double const (&d)[N])
{
  return toString(d, N);
}

vtkPolyData* GetCurrentFrame(vtkLidarReader* HDLreader, int index);

vtkPolyData* GetCurrentFrame(vtkLidarStream* HDLsource, int index);

int GetNumberOfTimesteps(vtkLidarStream* HDLSource);

std::vector<std::string> GenerateFileList(const std::string& metaFileName);

vtkPolyData* GetCurrentReference(const std::vector<std::string>& referenceFilesList, int index);

void SetProcessingOptions(vtkLidarReader* HDLReader, vvProcessingOptionsType currentOptions,
  int numProcessingOptions);

//void SetProcessingOptions(vtkLidarStream* HDLSource, vvProcessingOptionsType currentOptions,
//  int numProcessingOptions, std::string pcapFileName, std::string destinationIp, int dataPort);

// Test functions
/**
 * @brief TestFrameCount Checks the number of frame on the actual dataset
 * @param frameCount Number of frame in the current dataset
 * @param referenceCount Number of frame in the reference dataset
 * @return 0 on succes, 1 on failure
 */
int TestFrameCount(unsigned int frameCount, unsigned int referenceCount);

/**
 * @brief TestPointCount Checks the number of points on the actual dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointCount(vtkPolyData* currentFrame, vtkPolyData* currentReference);

/**
 * @brief TestPointDataStructure Checks the number of pointdata arrays & their
 * structure on the actual dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointDataStructure(vtkPolyData* currentFrame, vtkPolyData* currentReference);

/**
 * @brief TestPointDataValues Checks each value on each point data array on the
 *  actual dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointDataValues(vtkPolyData* currentFrame, vtkPolyData* currentReference);

/**
 * @brief TestPointPositions Checks the position of each point on the actual
 * dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestPointPositions(vtkPolyData* currentFrame, vtkPolyData* currentReference);

/**
 * @brief TestRPMValues Checks each value on each point data array on the actual
 * dataset
 * @param currentFrame Current frame
 * @param currentReference Reference for the current frame
 * @return 0 on success, 1 on failure
 */
int TestRPMValues(vtkPolyData* currentFrame, vtkPolyData* currentReference);

/**
 * @brief TestProcessingOptions
 * @param HDLReader Current reader
 * @return 0 on success, 1 on failure
 */
int TestProcessingOptions(vtkLidarReader* HDLReader);

/**
 * @brief TestProcessingOptions
 * @param HDLSource Current source
 * @return 0 on success, 1 on failure
 */
int TestProcessingOptions(vtkLidarStream* HDLSource, std::string pcapFileName,
  std::string destinationIp, int dataPort);

#endif
