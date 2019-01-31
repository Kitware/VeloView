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

#include <vtk_libproj4.h>

#include <liblas/liblas.hpp>

#include <Eigen/Dense>

#ifndef PJ_VERSION // 4.8 or later
#include <cassert>
#endif

namespace
{

#ifdef PJ_VERSION // 4.8 or later

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

#else

//-----------------------------------------------------------------------------
PROJ* CreateProj(int utmZone, bool south)
{
  std::ostringstream ss;
  ss << "+zone=" << utmZone;

  char buffer[65] = { 0 };
  strncpy(buffer, ss.str().c_str(), 64);

  std::vector<const char*> utmparams;
  utmparams.push_back("+proj=utm");
  utmparams.push_back("+ellps=WGS84");
  utmparams.push_back("+units=m");
  utmparams.push_back("+no_defs");
  utmparams.push_back(buffer);
  if (south)
  {
    utmparams.push_back("+south");
  }

  return proj_init(utmparams.size(), const_cast<char**>(&(utmparams[0])));
}

//-----------------------------------------------------------------------------
Eigen::Vector3d InvertProj(Eigen::Vector3d in, PROJ* proj)
{
  // This "lovely little gem" makes an awful lot of assumptions about the input
  // (some flavor of XY) and output (internal LP, which we assume / hope is
  // WGS'84) GCS's and what operations are "interesting" (i.e. the assumption
  // that the Z component does not need to be considered and can be passed
  // through unaltered). Unfortunately, it's the best we can do with PROJ 4.7
  // until VTK can be updated to use 4.8. Fortunately, given how we're being
  // used, our input really ought to always be UTM.
  PROJ_XY xy;
  xy.x = in[0];
  xy.y = in[1];

  const PROJ_LP lp = proj_inv(xy, proj);
  return Eigen::Vector3d(lp.lam * RAD_TO_DEG, lp.phi * RAD_TO_DEG, in[2]);
}

#endif
}

//-----------------------------------------------------------------------------
class vtkLASFileWriter::vtkInternal
{
public:
  vtkInternal()
  {
    this->IsWriterInstanciated = false;
  }

  void Close();

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

#ifdef PJ_VERSION // 4.8 or later
  projPJ InProj;
  projPJ OutProj;
#else
  PROJ* Proj;
#endif
  int OutGcs;
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

#ifdef PJ_VERSION // 4.8 or later
  this->Internal->InProj = 0;
  this->Internal->OutProj = 0;
#else
  this->Internal->Proj = 0;
#endif
  this->Internal->OutGcs = -1;

  this->Internal->npoints = 0;

  for (int i = 0; i < 3; ++i)
  {
    this->Internal->MaxPt[i] = -std::numeric_limits<double>::max();
    this->Internal->MinPt[i] = std::numeric_limits<double>::max();
  }

  this->Internal->Stream.open(filename, std::ios::out | std::ios::trunc | std::ios::binary);

  this->Internal->header.SetSoftwareId(SOFTWARE_NAME);
  this->Internal->header.SetDataFormatId(liblas::ePointFormat1);
  this->Internal->header.SetScale(1e-3, 1e-3, 1e-3);
}

//-----------------------------------------------------------------------------
vtkLASFileWriter::~vtkLASFileWriter()
{
  this->Internal->Close();

#ifdef PJ_VERSION // 4.8 or later
  pj_free(this->Internal->InProj);
  pj_free(this->Internal->OutProj);
#else
  proj_free(this->Internal->Proj);
#endif

  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetTimeRange(double min, double max)
{
  this->Internal->MinTime = min;
  this->Internal->MaxTime = max;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetOrigin(int gcs, double easting, double northing, double height)
{
  if (this->Internal->IsWriterInstanciated)
  {
    vtkGenericWarningMacro("Header can't be changed once the writer is instanciated");
    return;
  }

  Eigen::Vector3d origin(northing, easting, height);
  this->Internal->Origin = origin;

  // Convert offset to output GCS, if a geoconversion is set up
#ifdef PJ_VERSION // 4.8 or later
  if (this->Internal->OutProj)
  {
    origin = ConvertGcs(origin, this->Internal->InProj, this->Internal->OutProj);
    gcs = this->Internal->OutGcs;
  }
#else
  if (this->Internal->Proj)
  {
    origin = InvertProj(origin, this->Internal->Proj);
    gcs = this->Internal->OutGcs;
  }
#endif

  // Update header
  this->Internal->header.SetOffset(origin[0], origin[1], origin[2]);
  try
  {
    liblas::SpatialReference srs;
    std::ostringstream ss;
    ss << "EPSG:" << gcs;
    srs.SetFromUserInput(ss.str());
    std::cout << srs << std::endl;
    this->Internal->header.SetSRS(srs);
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
void vtkLASFileWriter::SetGeoConversion(int in, int out)
{
#ifdef PJ_VERSION // 4.8 or later
  pj_free(this->Internal->InProj);
  pj_free(this->Internal->OutProj);

  this->Internal->InProj = CreateProj(in);
  this->Internal->OutProj = CreateProj(out);
#else
  // The PROJ 4.7 API makes it near impossible to do generic transforms, hence
  // InvertProj (see also comments there) is full of assumptions. Assert some
  // of those assumptions here.
  assert((in > 32600 && in < 32661) || (in > 32700 && in < 32761));
  assert(out == 4326);

  proj_free(this->Internal->Proj);
  this->Internal->Proj = CreateProj(in % 100, in > 32700);
#endif

  this->Internal->OutGcs = out;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetGeoConversion(int in, int out, int utmZone, bool isLatLon)
{
#ifdef PJ_VERSION // 4.8 or later
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
  this->Internal->InProj = pj_init_plus(utmparamsIn.str().c_str());
  std::cout << "init In : " << utmparamsIn.str() << std::endl;

  if (isLatLon)
  {
    std::stringstream utmparamsOut;
    utmparamsOut << "+proj=longlat ";
    utmparamsOut << "+ellps=WGS84 ";
    utmparamsOut << "+datum=WGS84 ";
    utmparamsOut << "+no_defs ";
    this->Internal->OutProj = pj_init_plus(utmparamsOut.str().c_str());
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
    this->Internal->OutProj = pj_init_plus(utmparamsOut.str().c_str());
  }

  std::cout << "InProj :  created : " << this->Internal->InProj << std::endl;
  std::cout << "OutProj created : " << this->Internal->OutProj << std::endl;
  if (this->Internal->InProj)
    std::cout << "inProj datum_type : [" << this->Internal->InProj->datum_type << "]" << std::endl;
  if (this->Internal->OutProj)
    std::cout << "outProj datum_type : [" << this->Internal->OutProj->datum_type << "]" << std::endl;
#else
  // The PROJ 4.7 API makes it near impossible to do generic transforms, hence
  // InvertProj (see also comments there) is full of assumptions. Assert some
  // of those assumptions here.
  assert((in > 32600 && in < 32661) || (in > 32700 && in < 32761));
  assert(out == 4326);

  proj_free(this->Internal->Proj);
  this->Internal->Proj = CreateProj(in % 100, in > 32700);
#endif

  this->Internal->OutGcs = out;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetPrecision(double neTol, double hTol)
{
  if (this->Internal->IsWriterInstanciated)
  {
    vtkGenericWarningMacro("Header can't be changed once writer is instanciated");
    return;
  }

  this->Internal->header.SetScale(neTol, neTol, hTol);
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::WriteFrame(vtkPolyData* data)
{
  if (!this->Internal->IsWriterInstanciated)
  {
    this->Internal->Writer = new liblas::Writer(this->Internal->Stream, this->Internal->header);
    this->Internal->IsWriterInstanciated = true;
  }

  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const intensityData = data->GetPointData()->GetArray("intensity");
  vtkDataArray* const laserIdData = data->GetPointData()->GetArray("laser_id");
  vtkDataArray* const timestampData = data->GetPointData()->GetArray("timestamp");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
  {
    const double time = timestampData->GetComponent(n, 0) * 1e-6;
    if (time >= this->Internal->MinTime && time <= this->Internal->MaxTime)
    {
      Eigen::Vector3d pos;
      points->GetPoint(n, pos.data());
      pos += this->Internal->Origin;

#ifdef PJ_VERSION // 4.8 or later
      if (this->Internal->OutProj)
      {
        pos = ConvertGcs(pos, this->Internal->InProj, this->Internal->OutProj);
      }
#else
      if (this->Internal->Proj)
      {
        pos = InvertProj(pos, this->Internal->Proj);
      }
#endif

      liblas::Point p(&this->Internal->Writer->GetHeader());
      p.SetCoordinates(pos[0], pos[1], pos[2]);
      p.SetIntensity(static_cast<uint16_t>(intensityData->GetComponent(n, 0)));
      p.SetReturnNumber(1);
      p.SetNumberOfReturns(1);
      p.SetUserData(static_cast<uint8_t>(laserIdData->GetComponent(n, 0)));
      p.SetTime(time);

      this->Internal->Writer->WritePoint(p);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::FlushMetaData()
{
  this->Internal->header.SetPointRecordsByReturnCount(0, this->Internal->npoints);
  this->Internal->header.SetMin(this->Internal->MinPt[0], this->Internal->MinPt[1], this->Internal->MinPt[2]);
  this->Internal->header.SetMax(this->Internal->MaxPt[0], this->Internal->MaxPt[1], this->Internal->MaxPt[2]);
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::UpdateMetaData(vtkPolyData* data)
{
  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const timestampData = data->GetPointData()->GetArray("timestamp");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
  {
    const double time = timestampData->GetComponent(n, 0) * 1e-6;
    if (time >= this->Internal->MinTime && time <= this->Internal->MaxTime)
    {
      Eigen::Vector3d pos;
      points->GetPoint(n, pos.data());
      pos += this->Internal->Origin;

#ifdef PJ_VERSION // 4.8 or later
      if (this->Internal->OutProj)
      {
        pos = ConvertGcs(pos, this->Internal->InProj, this->Internal->OutProj);
      }
#else
      if (this->Internal->Proj)
      {
        pos = InvertProj(pos, this->Internal->Proj);
      }
#endif

      this->Internal->npoints++;

      for (int i = 0; i < 3; ++i)
      {
        if (pos[i] > this->Internal->MaxPt[i])
        {
          this->Internal->MaxPt[i] = pos[i];
        }
        if (pos[i] < this->Internal->MinPt[i])
        {
          this->Internal->MinPt[i] = pos[i];
        }
      }
    }
  }
}
