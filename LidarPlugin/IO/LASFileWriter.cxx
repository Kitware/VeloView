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

// positive are north zones, negative are south zones
int SignedUTMToEPSG(int signedUTM)
{
  if (signedUTM == 0 || std::abs(signedUTM) > 60)
  {
    return 0; // not an EPSG code, to my knowledge
  }

  if (signedUTM > 0)
  {
    return 32600 + signedUTM;
  }
  else
  {
    return 32700 + (- signedUTM);
  }
}

int EPSGToSignedUTM(int EPSG)
{
  if (EPSG >= 32601 && EPSG <= 32660)
  {
    return EPSG - 32600;
  }
  else if (EPSG >= 32701 && EPSG <= 32760)
  {
    return - (EPSG - 32700);
  }
  else
  {
    return 0; // not an UTM zone
  }
}

//-----------------------------------------------------------------------------
projPJ ProjFromEPSG(int epsg)
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
LASFileWriter::LASFileWriter()
{
  this->InProj = nullptr;
  this->OutProj = nullptr;
  this->OutGcsEPSG = -1;

  this->MinTime = -std::numeric_limits<double>::infinity();
  this->MaxTime = +std::numeric_limits<double>::infinity();

  this->Writer = nullptr;
}

int LASFileWriter::Open(const char* filename)
{
  // npoints and Max/MinPt are reseted, in case we already use this writer to
  // writer another file
  this->npoints = 0;

  for (int i = 0; i < 3; ++i)
  {
    this->MaxPt[i] = -std::numeric_limits<double>::max();
    this->MinPt[i] = std::numeric_limits<double>::max();
  }

  this->Stream.open(filename, std::ios::out | std::ios::trunc | std::ios::binary);
  if (!this->Stream.is_open())
  {
    return 0;
  }

  this->header.SetSoftwareId(SOFTWARE_NAME);
  if (this->WriteColor)
  {
    this->header.SetDataFormatId(liblas::ePointFormat3); // the first format with color and time
  }
  else
  {
    this->header.SetDataFormatId(liblas::ePointFormat1);
  }
  this->header.SetScale(1e-3, 1e-3, 1e-3);

  return 1;
}

void LASFileWriter::Close()
{
  if (this->Writer != nullptr)
  {
    delete this->Writer;
    this->Writer = nullptr;
  }

  if (this->Stream.is_open())
  {
    this->Stream.close();
  }

  if (this->InProj != nullptr)
  {
    pj_free(this->InProj);
    this->InProj = nullptr;
  }

  if (this->OutProj != nullptr)
  {
    pj_free(this->OutProj);
    this->OutProj = nullptr;
  }
}

//-----------------------------------------------------------------------------
LASFileWriter::~LASFileWriter()
{
  if (this->Stream.is_open())
  {
    this->Close();
  }
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetTimeRange(double min, double max)
{
  this->MinTime = min;
  this->MaxTime = max;
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetOrigin(double easting, double northing, double height)
{
  if (this->Writer)
  {
    vtkGenericWarningMacro("Header can't be changed once the writer is instanciated");
    return;
  }

  if (!this->OutProj || this->OutGcsEPSG <= 0)
  {
    vtkGenericWarningMacro("Origin cannot be set if GeoConversion is not set");
    return;
  }

  Eigen::Vector3d origin(northing, easting, height);
  this->Origin = origin;

  // Convert offset to output GCS, if a geoconversion is set up
  origin = ConvertGcs(origin, this->InProj, this->OutProj);

  // Update header
  this->header.SetOffset(origin[0], origin[1], origin[2]);
  if (this->WriteSRS)
  {
    try
    {
      liblas::SpatialReference srs;
      std::ostringstream ss;
      ss << "EPSG:" << this->OutGcsEPSG;
      srs.SetFromUserInput(ss.str());
      this->header.SetSRS(srs);
    }
    catch (std::logic_error &e)
    {
      std::cerr << "failed to set SRS (logic_error): " << e.what() << std::endl;
    }
    catch (std::runtime_error &e)
    {
      std::cerr << "failed to set SRS (runtime_error): " << e.what() << std::endl;
    }
  }
  else
  {
    std::cout << "As asked, not setting SRS in LAS Header." << std::endl;
  }
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetGeoConversionEPSG(int inEPSG, int outEPSG)
{
  pj_free(this->InProj);
  pj_free(this->OutProj);

  this->InProj = ProjFromEPSG(inEPSG);
  this->OutProj = ProjFromEPSG(outEPSG);

  this->OutGcsEPSG = outEPSG;
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetGeoConversionUTM(int inOutSignedUTMZone, bool useLatLonForOut)
{
  std::stringstream utmparamsIn;
  utmparamsIn << "+proj=utm ";
  std::stringstream zone;
  zone << "+zone=" << inOutSignedUTMZone;
  std::string UTMString = zone.str();
  utmparamsIn << UTMString << " ";
  if (inOutSignedUTMZone < 0)
  {
    utmparamsIn << "+south ";
  }
  utmparamsIn << "+datum=WGS84 ";
  utmparamsIn << "+units=m ";
  utmparamsIn << "+no_defs ";
  this->InProj = pj_init_plus(utmparamsIn.str().c_str());
  std::cout << "init In : " << utmparamsIn.str() << std::endl;

  if (useLatLonForOut)
  {
    std::stringstream utmparamsOut;
    utmparamsOut << "+proj=longlat ";
    utmparamsOut << "+ellps=WGS84 ";
    utmparamsOut << "+datum=WGS84 ";
    utmparamsOut << "+no_defs ";
    this->OutProj = pj_init_plus(utmparamsOut.str().c_str());
    std::cout << "init Out : " << utmparamsOut.str() << std::endl;
    // 4326 is EPSG ID code for lat-long-alt coordinates
    this->OutGcsEPSG = 4326;
  }
  else
  {
    std::stringstream utmparamsOut;
    utmparamsOut << "+proj=utm ";
    std::stringstream zone;
    zone << "+zone=" << inOutSignedUTMZone;
    std::string UTMString = zone.str();
    utmparamsOut << UTMString << " ";
    if (inOutSignedUTMZone < 0)
    {
      utmparamsOut << "+south ";
    }
    utmparamsOut << "+ellps=WGS84 ";
    utmparamsOut << "+datum=WGS84 ";
    utmparamsOut << "+no_defs ";
    this->OutProj = pj_init_plus(utmparamsOut.str().c_str());
    this->OutGcsEPSG = SignedUTMToEPSG(inOutSignedUTMZone);
  }

  std::cout << "InProj created : " << this->InProj << std::endl;
  std::cout << "OutProj created : " << this->OutProj << std::endl;
  if (this->InProj)
  {
    std::cout << "inProj datum_type : [" << this->InProj->datum_type << "]" << std::endl;
  }
  if (this->OutProj)
  {
    std::cout << "outProj datum_type : [" << this->OutProj->datum_type << "]" << std::endl;
  }
}

//-----------------------------------------------------------------------------
void LASFileWriter::SetPrecision(double neTol, double hTol)
{
  if (this->Writer)
  {
    vtkGenericWarningMacro("Header can't be changed once writer is instanciated");
    return;
  }

  this->header.SetScale(neTol, neTol, hTol);
}

//-----------------------------------------------------------------------------
void LASFileWriter::WriteFrame(vtkPolyData* data)
{
  if (!this->Writer)
  {
    this->Writer = new liblas::Writer(this->Stream, this->header);
  }

  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const intensityData = data->GetPointData()->GetArray("intensity");
  vtkDataArray* const laserIdData = data->GetPointData()->GetArray("laser_id");
  vtkDataArray* const timestampData = data->GetPointData()->GetArray("adjustedtime");
  vtkDataArray* const colorData = data->GetPointData()->GetArray("camera_color");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
  {
    const double time = timestampData == nullptr ? 0.0 : timestampData->GetComponent(n, 0) * 1e-6;
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
      p.SetIntensity(static_cast<uint16_t>(intensityData == nullptr ? 0.0 : intensityData->GetComponent(n, 0)));
      p.SetReturnNumber(1);
      p.SetNumberOfReturns(1);
      p.SetUserData(static_cast<uint8_t>(laserIdData == nullptr ? 0.0 : laserIdData->GetComponent(n, 0)));
      if (this->WriteColor && colorData != nullptr)
      {
        liblas::Color color = liblas::Color(
              static_cast<uint32_t>(colorData->GetComponent(n, 0)),
              static_cast<uint32_t>(colorData->GetComponent(n, 1)),
              static_cast<uint32_t>(colorData->GetComponent(n, 2))
              );
        p.SetColor(color);
      }
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
    const double time = timestampData == nullptr ? 0.0 : timestampData->GetComponent(n, 0) * 1e-6;
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

void LASFileWriter::SetMaxPt(double const* pt)
{
  this->MaxPt[0] = pt[0];
  this->MaxPt[1] = pt[1];
  this->MaxPt[2] = pt[2];
}
void LASFileWriter::SetMinPt(double const* pt)
{
  this->MinPt[0] = pt[0];
  this->MinPt[1] = pt[1];
  this->MinPt[2] = pt[2];
}

void LASFileWriter::SetWriteSRS(bool shouldWrite)
{
  this->WriteSRS = shouldWrite;
}

void LASFileWriter::SetWriteColor(bool shouldWrite)
{
  this->WriteColor = shouldWrite;
}
