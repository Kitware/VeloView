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

class vtkVelodyneHDLReader;
class vtkVelodyneHDLSource;
class vtkPolyData;

// Helper functions
bool compare(const double* const a, const double* const b, const size_t N, double epsilon);

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

vtkPolyData* GetCurrentFrame(vtkVelodyneHDLReader* HDLreader, int index);

vtkPolyData* GetCurrentFrame(vtkVelodyneHDLSource* HDLsource, int index);

int GetNumberOfTimesteps(vtkVelodyneHDLSource* HDLSource);

std::vector<std::string> GenerateFileList(const std::string& metaFileName);

vtkPolyData* GetCurrentReference(const std::vector<std::string>& referenceFilesList, int index);

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

#endif