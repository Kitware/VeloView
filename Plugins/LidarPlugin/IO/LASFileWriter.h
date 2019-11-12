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
  LASFileWriter();
  ~LASFileWriter();

  // Open a file for writting to it. Deleted if exising.
  // Will be closed with Close() or when the destructor is called.
  // The default LAS export scale is set to (1e-3, 1e-3, 1e-3)
  int Open(const char* filename);

  void Close();

  // If used, will restric to a time range the points written with WriteFrame
  void SetTimeRange(double min, double max);

  // SetGeoConversion{EPSG,UTM}  and SetOrigin() must be called before any call
  // to WriteFrame()
  void SetGeoConversionEPSG(int inEPSG, int outEPSG);
  // if useLatLonForOut then inOutSignedUTMZone is not used for the output
  void SetGeoConversionUTM(int inOutSignedUTMZone, bool useLatLonForOut);
  // SetOrigin() must be called after SetGeoConversion{EPSG,UTM}()
  void SetOrigin(double easting, double northing, double height);

  // Must be called before any call to WriteFrame()
  void SetPrecision(double neTol, double hTol = 1e-3);

  // - Computes the axis-aligned bounding box (Origin offset applied first)
  // - Count the number of points
  void UpdateMetaData(vtkPolyData* data);

  void SetMaxPt(double const* pt);
  void SetMinPt(double const* pt);

  void SetWriteSRS(bool shouldWrite);
  void SetWriteColor(bool shouldWrite);

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

  projPJ InProj; // used to intepret the polyDatas points
  projPJ OutProj; // used to project the points into coordinates used inside LAS
  int OutGcsEPSG; // used to tell in the LAS header which projection is used
  // Obviously, OutGcsEPSG should be coherent with OutProj

  // If WriteColor is set to False, the point format liblas::ePointFormat1 is used,
  // else liblas::ePointFormat3 is used.
  bool WriteColor = false;

  // Setting WriteSRS to false can be used to simulate the absence of GDAL
  // library (in which case setting SRS fails), or to use the default
  // interpretation of the software that will use the LAS file.
  bool WriteSRS = true;
};

#endif
