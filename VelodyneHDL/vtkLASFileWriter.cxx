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

//-----------------------------------------------------------------------------
class vtkLASFileWriter::vtkInternal
{
public:
  void Close();

  std::ofstream Stream;
  liblas::Writer* Writer;
};

//-----------------------------------------------------------------------------
void vtkLASFileWriter::vtkInternal::Close()
{
  // TODO
}

//-----------------------------------------------------------------------------
vtkLASFileWriter::vtkLASFileWriter(const char* filename)
  : Internal(new vtkInternal)
{
  this->Internal->Stream.open(
    filename, std::ios::out | std::ios::trunc | std::ios::binary);

  liblas::Header header;
  header.SetSoftwareId("VeloView");
  header.SetDataFormatId(liblas::ePointFormat1);
  this->Internal->Writer = new liblas::Writer(this->Internal->Stream, header);
}

//-----------------------------------------------------------------------------
vtkLASFileWriter::~vtkLASFileWriter()
{
  this->Internal->Close();
  delete this->Internal;
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
    double pos[3];
    points->GetPoint(n, pos);
    liblas::Point p;
    p.SetCoordinates(pos[0], pos[1], pos[2]);
    p.SetIntensity(static_cast<uint16_t>(intensityData->GetComponent(n, 0)));
    p.SetReturnNumber(0);
    p.SetNumberOfReturns(1);
    p.SetUserData(static_cast<uint8_t>(laserIdData->GetComponent(n, 0)));
    p.SetTime(timestampData->GetComponent(n, 0));

    this->Internal->Writer->WritePoint(p);
    }
}
