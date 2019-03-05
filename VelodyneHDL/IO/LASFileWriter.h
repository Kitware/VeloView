// Copyright 2014 Velodyne Acoustics, Inc.
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
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLASFileWriter.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkLASFileWriter -
// .SECTION Description
//

#ifndef __vtkLASFileWriter_h
#define __vtkLASFileWriter_h

#include <vtkSystemIncludes.h>
#include <vtk_libproj4.h>

#include <liblas/liblas.hpp>

#include <Eigen/Dense>

class vtkPolyData;

class VTK_EXPORT LASFileWriter
{
public:
  // The default LAS export scale is set to (1e-3, 1e-3, 1e-3)
  // The LAS file is opened immediatly, will be closed when destructor is called
  LASFileWriter(const char* filename);
  ~LASFileWriter();

  // If used, will restric to a time range the points written with WriteFrame
  void SetTimeRange(double min, double max);

  // Must be called before any call to WriteFrame()
  void SetOrigin(int gcs, double easting, double northing, double height);
  void SetGeoConversion(int in, int out);
  void SetGeoConversion(int in, int out, int utmZone, bool isLatLon);

  // Must be called before any call to WriteFrame()
  void SetPrecision(double neTol, double hTol = 1e-3);

  // - Computes the axis-aligned bounding box (Origin offset applied first)
  // - Count the number of points
  void UpdateMetaData(vtkPolyData* data);

  // Sets the metadata into the LAS header
  void FlushMetaData();

  // UpdateMetaData() and FlushMetaData() must have been called before the
  // first call to WriteFrame()
  // Will use arrays:
  // - intensity
  // - laser_id (has user data field)
  // - timestamp
  // Some values are hardcoded:
  // - return number = 1
  // - number of returns = 1
  // Note: if SetTimeRange() was used, it is possible that not all points will
  // be written
  void WriteFrame(vtkPolyData* data);

private:
  std::ofstream Stream;
  liblas::Writer* Writer;

  double MinTime;
  double MaxTime;
  Eigen::Vector3d Origin;

  size_t npoints;
  double MinPt[3];
  double MaxPt[3];

  liblas::Header header;
  bool IsWriterInstanciated;

  projPJ InProj;
  projPJ OutProj;
  int OutGcs;
};

#endif
