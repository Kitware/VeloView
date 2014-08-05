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

#include "vtkLASFileWriter.h"

#include <vtkPointData.h>
#include <vtkPolyData.h>

#include <liblas/liblas.hpp>

#include <Eigen/Dense>

//-----------------------------------------------------------------------------
class vtkLASFileWriter::vtkInternal
{
public:
  void Close();

  std::ofstream Stream;
  liblas::Writer* Writer;

  double MinTime;
  double MaxTime;
  Eigen::Vector3d Origin;
};

//-----------------------------------------------------------------------------
void vtkLASFileWriter::vtkInternal::Close()
{
  delete this->Writer;
  this->Writer = 0;
  this->Stream.close();
}

//-----------------------------------------------------------------------------
vtkLASFileWriter::vtkLASFileWriter(const char* filename)
  : Internal(new vtkInternal)
{
  this->Internal->MinTime = -std::numeric_limits<double>::infinity();
  this->Internal->MaxTime = +std::numeric_limits<double>::infinity();

  this->Internal->Stream.open(
    filename, std::ios::out | std::ios::trunc | std::ios::binary);

  liblas::Header header;
  header.SetSoftwareId("VeloView");
  header.SetDataFormatId(liblas::ePointFormat1);
  header.SetScale(1e-3, 1e-3, 1e-3);
  this->Internal->Writer = new liblas::Writer(this->Internal->Stream, header);
}

//-----------------------------------------------------------------------------
vtkLASFileWriter::~vtkLASFileWriter()
{
  this->Internal->Close();
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetTimeRange(double min, double max)
{
  this->Internal->MinTime = min;
  this->Internal->MaxTime = max;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetUTMOrigin(
  int zone, double easting, double northing, double height)
{
  liblas::Header header = this->Internal->Writer->GetHeader();

  header.SetOffset(easting, northing, height);

  try
    {
    liblas::SpatialReference srs;
    std::ostringstream ss;
    ss << "EPSG:" << 32600 + zone;
    srs.SetFromUserInput(ss.str());

    header.SetSRS(srs);
    }
  catch (std::runtime_error)
    {
    std::cerr << "failed to set SRS" << std::endl;
    }

  this->Internal->Writer->SetHeader(header);
  this->Internal->Writer->WriteHeader();

  this->Internal->Origin[0] = easting;
  this->Internal->Origin[1] = northing;
  this->Internal->Origin[2] = height;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::WriteFrame(vtkPolyData* data)
{
  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const intensityData =
    data->GetPointData()->GetArray("intensity");
  vtkDataArray* const laserIdData =
    data->GetPointData()->GetArray("laser_id");
  vtkDataArray* const timestampData =
    data->GetPointData()->GetArray("timestamp");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
    {
    const double time = timestampData->GetComponent(n, 0) * 1e-6;
    if (time >= this->Internal->MinTime && time <= this->Internal->MaxTime)
      {
      Eigen::Vector3d pos;
      points->GetPoint(n, pos.data());
      pos += this->Internal->Origin;

      liblas::Point p(&this->Internal->Writer->GetHeader());
      p.SetCoordinates(pos[0], pos[1], pos[2]);
      p.SetIntensity(static_cast<uint16_t>(intensityData->GetComponent(n, 0)));
      p.SetReturnNumber(0);
      p.SetNumberOfReturns(1);
      p.SetUserData(static_cast<uint8_t>(laserIdData->GetComponent(n, 0)));
      p.SetTime(time);

      this->Internal->Writer->WritePoint(p);
      }
    }
}
