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

#include "LASFileWriter.h"

#include <vtkPointData.h>
#include <vtkPolyData.h>

namespace
{

//-----------------------------------------------------------------------------
projPJ CreateProj(int epsg)
{
  std::ostringstream ss;
  ss << "+init=epsg:" << epsg << " ";
  return pj_init_plus(ss.str().c_str());
}

//-----------------------------------------------------------------------------
Eigen::Vector3d ConvertGcs(Eigen::Vector3d p, projPJ inProj, projPJ outProj)
{
  if (pj_is_latlong(inProj))
  {
    p[0] *= DEG_TO_RAD;
    p[1] *= DEG_TO_RAD;
  }

  double* const data = p.data();
  //std::cout << "position in : [" << p[0] << ";" << p[1] << ";" << p[2] << "]" << std::endl;
  int last_errno = pj_transform(inProj, outProj, 1, 1, data + 0, data + 1, data + 2);
  //std::cout << "position out : [" << p[0] << ";" << p[1] << ";" << p[2] << "]" << std::endl << std::endl;

  if (last_errno != 0)
  {
    vtkGenericWarningMacro("Error : CRS conversion failed with error: " << last_errno);
  }

  if (pj_is_latlong(outProj))
  {
    p[0] *= RAD_TO_DEG;
    p[1] *= RAD_TO_DEG;
  }

  return p;
}
}

//-----------------------------------------------------------------------------
LASFileWriter::LASFileWriter(const char* filename)
{
  this->MinTime = -std::numeric_limits<double>::infinity();
  this->MaxTime = +std::numeric_limits<double>::infinity();

  this->InProj = 0;
  this->OutProj = 0;
  this->OutGcs = -1;

  this->npoints = 0;

  for (int i = 0; i < 3; ++i)
  {
    this->MaxPt[i] = -std::numeric_limits<double>::max();
    this->MinPt[i] = std::numeric_limits<double>::max();
  }

  this->Stream.open(filename, std::ios::out | std::ios::trunc | std::ios::binary);

  this->header.SetSoftwareId(SOFTWARE_NAME);
  this->header.SetDataFormatId(liblas::ePointFormat1);
  this->header.SetScale(1e-3, 1e-3, 1e-3);
}

//-----------------------------------------------------------------------------
LASFileWriter::~LASFileWriter()
{
  delete this->Writer;
  this->Writer = 0;
  this->Stream.close();

  pj_free(this->InProj);
  pj_free(this->OutProj);
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetTimeRange(double min, double max)
{
  this->MinTime = min;
  this->MaxTime = max;
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetOrigin(int gcs, double easting, double northing, double height)
{
  if (this->IsWriterInstanciated)
  {
    vtkGenericWarningMacro("Header can't be changed once the writer is instanciated");
    return;
  }

  Eigen::Vector3d origin(northing, easting, height);
  this->Origin = origin;

  // Convert offset to output GCS, if a geoconversion is set up
  if (this->OutProj)
  {
    origin = ConvertGcs(origin, this->InProj, this->OutProj);
    gcs = this->OutGcs;
  }

  // Update header
  this->header.SetOffset(origin[0], origin[1], origin[2]);
  try
  {
    liblas::SpatialReference srs;
    std::ostringstream ss;
    ss << "EPSG:" << gcs;
    srs.SetFromUserInput(ss.str());
    std::cout << srs << std::endl;
    this->header.SetSRS(srs);
  }
  catch (std::logic_error)
  {
    std::cerr << "failed to set SRS (logic)" << std::endl;
  }
  catch (std::runtime_error)
  {
    std::cerr << "failed to set SRS" << std::endl;
  }
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetGeoConversion(int in, int out)
{
  pj_free(this->InProj);
  pj_free(this->OutProj);

  this->InProj = CreateProj(in);
  this->OutProj = CreateProj(out);

  this->OutGcs = out;
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetGeoConversion(int in, int out, int utmZone, bool isLatLon)
{
  in = in;  // this was just added to avoid the warning: "parameter 'in' is not used"

  std::stringstream utmparamsIn;
  utmparamsIn << "+proj=utm ";
  std::stringstream zone;
  zone << "+zone=" << utmZone;
  std::string UTMString = zone.str();
  utmparamsIn << UTMString << " ";
  utmparamsIn << "+datum=WGS84 ";
  utmparamsIn << "+units=m ";
  utmparamsIn << "+no_defs ";
  this->InProj = pj_init_plus(utmparamsIn.str().c_str());
  std::cout << "init In : " << utmparamsIn.str() << std::endl;

  if (isLatLon)
  {
    std::stringstream utmparamsOut;
    utmparamsOut << "+proj=longlat ";
    utmparamsOut << "+ellps=WGS84 ";
    utmparamsOut << "+datum=WGS84 ";
    utmparamsOut << "+no_defs ";
    this->OutProj = pj_init_plus(utmparamsOut.str().c_str());
    std::cout << "init Out : " << utmparamsOut.str() << std::endl;
  }
  else
  {
    std::stringstream utmparamsOut;
    utmparamsOut << "+proj=utm ";
    std::stringstream zone;
    zone << "+zone=" << utmZone;
    std::string UTMString = zone.str();
    utmparamsOut << UTMString << " ";
    utmparamsOut << "+ellps=WGS84 ";
    utmparamsOut << "+datum=WGS84 ";
    utmparamsOut << "+no_defs ";
    this->OutProj = pj_init_plus(utmparamsOut.str().c_str());
  }

  std::cout << "InProj :  created : " << this->InProj << std::endl;
  std::cout << "OutProj created : " << this->OutProj << std::endl;
  if (this->InProj)
    std::cout << "inProj datum_type : [" << this->InProj->datum_type << "]" << std::endl;
  if (this->OutProj)
    std::cout << "outProj datum_type : [" << this->OutProj->datum_type << "]" << std::endl;

  this->OutGcs = out;
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetPrecision(double neTol, double hTol)
{
  if (this->IsWriterInstanciated)
  {
    vtkGenericWarningMacro("Header can't be changed once writer is instanciated");
    return;
  }

  this->header.SetScale(neTol, neTol, hTol);
}

//-----------------------------------------------------------------------------
void LASFileWriter::WriteFrame(vtkPolyData* data)
{
  if (!this->IsWriterInstanciated)
  {
    this->Writer = new liblas::Writer(this->Stream, this->header);
    this->IsWriterInstanciated = true;
  }

  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const intensityData = data->GetPointData()->GetArray("intensity");
  vtkDataArray* const laserIdData = data->GetPointData()->GetArray("laser_id");
  vtkDataArray* const timestampData = data->GetPointData()->GetArray("timestamp");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
  {
    const double time = timestampData->GetComponent(n, 0) * 1e-6;
    // This test implements the time-clamping feature
    if (time >= this->MinTime && time <= this->MaxTime)
    {
      Eigen::Vector3d pos;
      points->GetPoint(n, pos.data());
      pos += this->Origin;

      if (this->OutProj)
      {
        pos = ConvertGcs(pos, this->InProj, this->OutProj);
      }

      liblas::Point p(&this->Writer->GetHeader());
      p.SetCoordinates(pos[0], pos[1], pos[2]);
      p.SetIntensity(static_cast<uint16_t>(intensityData->GetComponent(n, 0)));
      p.SetReturnNumber(1);
      p.SetNumberOfReturns(1);
      p.SetUserData(static_cast<uint8_t>(laserIdData->GetComponent(n, 0)));
      p.SetTime(time);

      this->Writer->WritePoint(p);
    }
  }
}

//-----------------------------------------------------------------------------
void LASFileWriter::FlushMetaData()
{
  this->header.SetPointRecordsByReturnCount(0, this->npoints);
  this->header.SetMin(this->MinPt[0], this->MinPt[1], this->MinPt[2]);
  this->header.SetMax(this->MaxPt[0], this->MaxPt[1], this->MaxPt[2]);
}

//-----------------------------------------------------------------------------
void LASFileWriter::UpdateMetaData(vtkPolyData* data)
{
  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const timestampData = data->GetPointData()->GetArray("timestamp");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
  {
    const double time = timestampData->GetComponent(n, 0) * 1e-6;
    if (time >= this->MinTime && time <= this->MaxTime)
    {
      Eigen::Vector3d pos;
      points->GetPoint(n, pos.data());
      pos += this->Origin;

      if (this->OutProj)
      {
        pos = ConvertGcs(pos, this->InProj, this->OutProj);
      }

      this->npoints++;

      for (int i = 0; i < 3; ++i)
      {
        if (pos[i] > this->MaxPt[i])
        {
          this->MaxPt[i] = pos[i];
        }
        if (pos[i] < this->MinPt[i])
        {
          this->MinPt[i] = pos[i];
        }
      }
    }
  }
}
