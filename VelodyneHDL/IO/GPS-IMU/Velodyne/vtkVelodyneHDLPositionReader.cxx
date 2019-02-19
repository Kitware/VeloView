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
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVelodyneHDLPositionReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkVelodyneHDLPositionReader.h"

#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"
#include "vtkVelodyneTransformInterpolator.h"

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTransform.h>
#include <vtkTupleInterpolator.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>

#include <vtk_libproj4.h>
#include "NMEAParser.h"
#include "statistics.h"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>

#include <iomanip>
#include <algorithm>
#include <map>
#include <sstream>

#include <cmath>

#ifdef _MSC_VER
#include <boost/cstdint.hpp>
typedef boost::uint8_t uint8_t;
#else
#include <stdint.h>
#endif

namespace
{
struct PositionPacket
{
  // tohTimestamp is a microseconds rolling counter provided by the lidar
  // internal clock which will be adjusted using the gps data (if one is plugged)
  // tohTimestamp gives the number of microseconds ellapsed since the top of
  // the hour. The top of the hour is defined as (modulo 1 hour):
  // - without a gps plugged: the instant when the lidar was powered on
  // - with a gps: any full UTC hour (such as 12:00:00 am, UTC)
  unsigned int tohTimestamp;
  short gyro[3];
  short temp[3];
  short accelx[3];
  short accely[3];
  // Usage of field PPS (Pulse Per Second signal) is explained in PDF document
  // "Webserver User Guide (VLP-16 & HDL-32E)" available at
  // https://velodynelidar.com/downloads.html#application_notes
  unsigned char PPSSync;
  // UDP position packet is 554 bytes long with 42 bytes of header and 512 bytes
  // of payload. In the payload, 512 - (198 + 4 + 1 + 3) = 306 are available for
  // the NMEA sequence.
  // The last bytes stored in the sentence char array should be 0 (NMEA
  // sentences are not that long).
  char sentance[306];
};
}

namespace
{
int LatLongToZone(double lat, double lon)
{
  double longTemp = (lon + 180) - static_cast<int>((lon + 180) / 360) * 360 - 180;

  int zone = static_cast<int>((longTemp + 180) / 6) + 1;
  if (lat >= 56.0 && lat < 64.0 && longTemp >= 3.0 && longTemp < 12.0)
  {
    zone = 32;
  }

  if (lat >= 72.0 && lat < 84)
  {
    if (longTemp >= 0.0 && longTemp < 9.0)
    {
      zone = 31;
    }
    else if (longTemp >= 9.0 && longTemp < 21.0)
    {
      zone = 33;
    }
    else if (longTemp >= 21.0 && longTemp < 33.0)
    {
      zone = 35;
    }
    else if (longTemp >= 33.0 && longTemp < 42.0)
    {
      zone = 37;
    }
  }

  return zone;
}


class UTMProjector
{
  public:
  UTMProjector(bool shouldWarnOnWeirdGPSData)
  {
    this->ShouldWarnOnWeirdGPSData = shouldWarnOnWeirdGPSData;
    this->pj_utm = nullptr;
    this->UTMZone = -1;
  }

  ~UTMProjector()
  {
    if (this->IsInitialized())
    {
      pj_free(this->pj_utm);
    }
  }

  void Project(double lat, double lon, double& x, double& y)
  {
    if (!this->IsInitialized())
    {
      this->Init(lat, lon);
    }

    projUV lp;
    lp.u = DEG_TO_RAD * lon;
    lp.v = DEG_TO_RAD * lat;

    projUV xy;
    xy = pj_fwd(lp, pj_utm);
    if (pj_utm->ctx->last_errno != 0 && this->ShouldWarnOnWeirdGPSData)
    {
      vtkGenericWarningMacro("Error : WGS84 projection failed, this will create a GPS error. "
                             "Please check the latitude and longitude inputs");
    }

    x = xy.u;
    y = xy.v;
  }

  private:
  bool IsInitialized()
  {
    return this->pj_utm != nullptr;
  }

  void Init(double initial_lat, double initial_lon)
  {
    assert(!pj_utm);
    this->UTMZone = LatLongToZone(initial_lat, initial_lon);
    std::stringstream utmparams;
    utmparams << "+proj=utm ";
    std::stringstream zone;
    zone << "+zone=" << this->UTMZone;
    this->UTMString = zone.str();
    // WARNING: Dont let the string stream pass out of scope until
    // we finish initialization
    utmparams << this->UTMString << " ";
    if (initial_lat < 0)
    {
      utmparams << "+south ";
    }

    utmparams << "+ellps=WGS84 ";
    utmparams << "+units=m ";
    utmparams << "+no_defs ";
    pj_utm = pj_init_plus(utmparams.str().c_str());
  }

  bool ShouldWarnOnWeirdGPSData;
  projPJ pj_utm;
  int UTMZone;
  std::string UTMString;
};
}

//-----------------------------------------------------------------------------
class vtkVelodyneHDLPositionReader::vtkInternal
{
public:
  vtkInternal()
  {
    this->Reader = 0;
    this->Offset[0] = 0.0;
    this->Offset[1] = 0.0;
    this->Offset[2] = 0.0;
    this->CalibrationTransform->Identity();
  }

  int ProcessHDLPacket(const unsigned char* data, unsigned int bytes, PositionPacket& position);

  void InterpolateGPS(
    vtkPoints* points, vtkDataArray* gpsTime, vtkDataArray* times, vtkDataArray* heading);

  vtkPacketFileReader* Reader;
  double Offset[3];

  vtkNew<vtkVelodyneTransformInterpolator> Interp;
  vtkNew<vtkTransform> CalibrationTransform;
};

namespace
{
const unsigned short BIT_12_MASK = 0x0fff;
const unsigned short REMAINDER_12_MASK = 0x07ff;
const unsigned short SIGN_12_MASK = 0x0800;

const double GYRO_SCALE = 0.09766;   // deg / s
const double TEMP_SCALE = 0.1453;    // C
const double TEMP_OFFSET = 25.0;     // C
const double ACCEL_SCALE = 0.001221; // G
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLPositionReader::vtkInternal::ProcessHDLPacket(
  const unsigned char* data, unsigned int bytes, PositionPacket& position)
{
  if (bytes != 512)
  {
    // Data-Packet Specifications says that position-packets are 512 byte long.
    return 0;
  }

  for (int i = 0; i < 14; ++i)
  {
    if (data[i] != 0)
    {
      std::cerr << "unexpected data in first zeros block\n";
      return 0;
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    memcpy(position.gyro + i, data + 14 + i * 8, 2);
    memcpy(position.temp + i, data + 14 + i * 8 + 2, 2);
    memcpy(position.accelx + i, data + 14 + i * 8 + 4, 2);
    memcpy(position.accely + i, data + 14 + i * 8 + 6, 2);
  }

  for (int i = 0; i < 3; ++i)
  {
    // Selector only least significant 12 bits
    position.gyro[i] &= BIT_12_MASK;
    position.temp[i] &= BIT_12_MASK;
    position.accelx[i] &= BIT_12_MASK;
    position.accely[i] &= BIT_12_MASK;

    // Perform 12 bit twos complement
    position.gyro[i] =
      -2048 * ((position.gyro[i] & SIGN_12_MASK) >> 11) + (position.gyro[i] & REMAINDER_12_MASK);
    position.temp[i] =
      -2048 * ((position.temp[i] & SIGN_12_MASK) >> 11) + (position.temp[i] & REMAINDER_12_MASK);
    position.accelx[i] = -2048 * ((position.accelx[i] & SIGN_12_MASK) >> 11) +
      (position.accelx[i] & REMAINDER_12_MASK);
    position.accely[i] = -2048 * ((position.accely[i] & SIGN_12_MASK) >> 11) +
      (position.accely[i] & REMAINDER_12_MASK);
  }

  memcpy(&position.tohTimestamp, data + 14 + 3 * 8 + 160, 4);

  // ethernet payload starts at byte 2A, PPS byte is at F4 inside full ethernet
  // frame, so PPS in payload is at F4 - 2A = 244 - 42 = 202
  memcpy(&position.PPSSync, data + 202, 1);

  const int sentence_start =  14 + 8 + 8 + 8 + 160 + 4 + 4;
  std::copy(data + sentence_start,
            data + sentence_start + 306,
            position.sentance);
  // protection to terminate the string in case the sentence does not fit in the
  // 306 bytes.
  position.sentance[305] = '\0';

  return 1;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::SetShouldWarnOnWeirdGPSData(bool ShouldWarnOnWeirdGPSData_)
{
  this->ShouldWarnOnWeirdGPSData = ShouldWarnOnWeirdGPSData_;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::SetCalibrationTransform(vtkTransform* transform)
{
  if (transform)
  {
    this->Internal->CalibrationTransform->SetMatrix(transform->GetMatrix());
  }
  else
  {
    this->Internal->CalibrationTransform->Identity();
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator* vtkVelodyneHDLPositionReader::GetInterpolator()
{
  return this->Internal->Interp.GetPointer();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::vtkInternal::InterpolateGPS(
  vtkPoints* points, vtkDataArray* gpsTime, vtkDataArray* times, vtkDataArray* headings)
{
  vtkNew<vtkTupleInterpolator> headingInterpolator;

  // assert(gpsTime is sorted)
  assert(points->GetNumberOfPoints() == times->GetNumberOfTuples());

  this->Interp->SetInterpolationTypeToLinear();
  this->Interp->Initialize();
  headingInterpolator->SetInterpolationTypeToLinear();
  headingInterpolator->SetNumberOfComponents(2);

  assert(times->GetNumberOfTuples() == gpsTime->GetNumberOfTuples());

  double lastGPS = 0.0;
  for (vtkIdType i = 0, k = times->GetNumberOfTuples(); i < k; ++i)
  {
    const double currGPS = gpsTime->GetTuple1(i);
    if (currGPS != lastGPS)
    {
      // Get position and heading
      double pos[3];
      points->GetPoint(i, pos);

      const double heading = headings->GetTuple1(i);

      // Check the input data
      bool isTranslationFinite =
        vtkMath::IsFinite(pos[0]) && vtkMath::IsFinite(pos[1]) && vtkMath::IsFinite(pos[2]);
      bool isRotationFinite = vtkMath::IsFinite(heading);

      // Compute transform
      // Here we want to compute the transform to go
      // from the solid referential frame to the world
      // georeferenced frame. Hence, given the position
      // and orientation of the GPS in the solid frame we
      // need to apply the transform from the solid frame
      // to the GPS (backward gps pose) and then the transform
      // from the GPS to the world georeferenced frame
      vtkNew<vtkTransform> transformGpsWorld, transformVehiculeWorld;
      transformGpsWorld->PostMultiply();
      if (isRotationFinite)
        transformGpsWorld->RotateZ(heading);
      else
        vtkGenericWarningMacro("Error in GPS rotation");
      if (isTranslationFinite)
        transformGpsWorld->Translate(pos);
      else
        vtkGenericWarningMacro("Error in GPS position");

      // Compute transform from vehicule to GPS
      // and then compose with the transform GPS to world
      vtkNew<vtkMatrix4x4> gpsToWorld, vehiculeToGps, vehiculeToWorld;
      this->CalibrationTransform->GetMatrix(vehiculeToGps.Get());
      transformGpsWorld->GetMatrix(gpsToWorld.Get());
      vehiculeToGps->Invert();
      vtkMatrix4x4::Multiply4x4(gpsToWorld.Get(), vehiculeToGps.Get(), vehiculeToWorld.Get());
      transformVehiculeWorld->SetMatrix(vehiculeToWorld.Get());
      transformVehiculeWorld->Modified();

      // Add the transform to the interpolator
      this->Interp->AddTransform(currGPS, transformVehiculeWorld.GetPointer());

      // Compute heading vector for interpolation
      double ha = heading * DEG_TO_RAD;
      double hv[2] = { cos(ha), sin(ha) };
      headingInterpolator->AddTuple(currGPS, hv);
    }
  }
}

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLPositionReader)

//-----------------------------------------------------------------------------
vtkVelodyneHDLPositionReader::vtkVelodyneHDLPositionReader()
{
  this->UseGPGGASentences = false;
  this->Internal = new vtkInternal;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
  this->PPSSynced = false;
  this->LastPPSState = this->PPSState::PPS_ABSENT;
  this->HasTimeshiftEstimation = false;
  this->TimeshiftMeasurements.clear();
  this->AssumedHardwareLag = 0.094; // always positive, in seconds
}

//-----------------------------------------------------------------------------
vtkVelodyneHDLPositionReader::~vtkVelodyneHDLPositionReader()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLPositionReader::GetFileName()
{
  return this->FileName;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::SetFileName(const std::string& filename)
{
  if (filename == this->FileName)
  {
    return;
  }

  this->PPSSynced = false;
  this->LastPPSState = this->PPSState::PPS_ABSENT;
  this->HasTimeshiftEstimation = false;
  this->TimeshiftMeasurements.clear();
  this->FileName = filename;

  this->Modified();
}


//-----------------------------------------------------------------------------
int vtkVelodyneHDLPositionReader::RequestData(vtkInformation* vtkNotUsed(request),
                                              vtkInformationVector** vtkNotUsed(inputVector),
                                              vtkInformationVector* outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector);

  if (!this->FileName.length())
  {
    vtkErrorMacro("FileName has not been set.");
    return 0;
  }


  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
  vtkSmartPointer<vtkIdList> polyIds = polyLine->GetPointIds();

  // Data arrays
  vtkSmartPointer<vtkDoubleArray> lats = vtkSmartPointer<vtkDoubleArray>::New();
  lats->SetName("lat");
  vtkSmartPointer<vtkDoubleArray> lons = vtkSmartPointer<vtkDoubleArray>::New();
  lons->SetName("lon");

  vtkSmartPointer<vtkDoubleArray> times = vtkSmartPointer<vtkDoubleArray>::New();
  times->SetName("time");

  vtkSmartPointer<vtkDoubleArray> gpsTime = vtkSmartPointer<vtkDoubleArray>::New();
  gpsTime->SetName("gpstime");

  typedef std::map<std::string, vtkSmartPointer<vtkDoubleArray> > VecMap;
  VecMap dataVectors;
  dataVectors.insert(std::make_pair("gyro1", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("gyro2", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("gyro3", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("temp1", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("temp2", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("temp3", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel1x", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel1y", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel2x", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel2y", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel3x", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel3y", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("heading", vtkSmartPointer<vtkDoubleArray>::New()));
  for (VecMap::iterator it = dataVectors.begin(); it != dataVectors.end(); ++it)
  {
    it->second->SetName(it->first.c_str());
  }

  points->Allocate(5000, 5000);
  cells->Allocate(5000, 5000);
  lats->Allocate(5000, 5000);
  lons->Allocate(5000, 5000);
  gpsTime->Allocate(5000, 5000);

  const unsigned char* data;
  unsigned int dataLength;
  double timeSinceStart;

  UTMProjector proj(this->ShouldWarnOnWeirdGPSData);

  this->Open();
  vtkIdType pointcount = 0;

  bool hasLastGPSUpdateTime = false;
  double lastGPSUpdateTime = 0.0;
  double GPSTimeOffset = 0.0;
  double convertedGPSUpdateTime = 0.0;

  bool hasLastLidarUpdateTime = false;
  double lastLidarUpdateTime = 0.0;
  double lidarTimeOffset = 0.0;

  double previousConvertedGPSUpdateTime = -1.0; // negative means "no previous"

  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
  {
    PositionPacket position;
    if (!this->Internal->ProcessHDLPacket(data, dataLength, position))
    {
      continue;
    }

    if (!hasLastLidarUpdateTime)
    {
      hasLastLidarUpdateTime = true;
      lastLidarUpdateTime = position.tohTimestamp;
    }
    else
    {
      if (position.tohTimestamp - lastLidarUpdateTime < - 1e6 * 0.5 * 3600.0)
      {
        // tod wrap detected
        lidarTimeOffset += 1e6 * 3600.0;
      }
    }
    double convertedLidarUpdateTime = position.tohTimestamp + lidarTimeOffset;


    double x, y, z, lat, lon, heading, gpsUpdateTime;
    if (std::string(position.sentance).size() == 0)
    {
      // If there is no sentence to parse (no gps connected),
      // we use the following:
      x = 0.0;
      y = 0.0;
      z = 0.0;
      lat = 0.0;
      lon = 0.0;
      heading = 0.0;
      // the follwing value is wrong (no gps update so no update time),
      // so it should also be 0.0 (invalid)
      // but this value is kept to not risk breaking anything:
      gpsUpdateTime = position.tohTimestamp;
    }
    else
    {
      NMEAParser parser;
      NMEALocation parsedNMEA;
      parsedNMEA.Init();
      std::string NMEASentence = std::string(position.sentance);
      boost::trim_right(NMEASentence);
      std::vector<std::string> NMEAwords = parser.SplitWords(NMEASentence);
      if (!parser.ChecksumValid(NMEASentence))
      {
        vtkGenericWarningMacro("NMEA sentence: "
                               << "<" << NMEASentence << ">"
                               << "has invalid checksum");
        // TODO: should we skip or should we expect lazy NMEA implementers ?
      }

      if ((this->UseGPGGASentences && !parser.IsGPGGA(NMEAwords))
          || (!this->UseGPGGASentences && !parser.IsGPRMC(NMEAwords)))
      {
        continue; // not the NMEA sentence we are interested in, skipping
      }

      if ( !( (parser.IsGPGGA(NMEAwords) && parser.ParseGPGGA(NMEAwords, parsedNMEA))
           || (parser.IsGPRMC(NMEAwords) && parser.ParseGPRMC(NMEAwords, parsedNMEA)) ))
      {
        vtkGenericWarningMacro("Failed to parse NMEA sentence: "
                               << "<" << NMEASentence << ">");
        continue; // skipping this PositionPacket
      }

      // Gathering information on time synchronization between Lidar & GPS,
      // see Velodyne doc mentioned at definition of PositionPacket above.
      // We assume the Lidar will remain synchronized on gps (UTC) time even
      // if some NMEA packets without fix are received later (tunnel, hill ...).
      if (!this->PPSSynced
          && position.PPSSync == PPSState::PPS_LOCKED
          && parsedNMEA.Valid)
      {
        this->PPSSynced = true;
      }
      this->LastPPSState = static_cast<PPSState>(position.PPSSync);

      lat = parsedNMEA.Lat;
      lon = parsedNMEA.Long;
      x = 0.0;
      y = 0.0;
      proj.Project(lat, lon, x, y);
      z = 0.0;
      // If sentence is GPGGA,  we have a chance to get an altitude
      if (parser.IsGPGGA(NMEAwords))
      {
        if (parsedNMEA.HasAltitude)
        {
          if (parsedNMEA.HasGeoidalSeparation)
          {
            // setting z to Height above ellipsoid
            // (coherent with setting 'datum=WGS84' in proj4)
            z = parsedNMEA.Altitude + parsedNMEA.GeoidalSeparation;
          }
          else
          {
            // Two possibilities: either Altitude is actually height above
            // ellipsoid, or it is effectively height above local MSL.
            // In this case we could need something better than this
            // (such as a look up table for geoid separation like GeographicLib)
            z = parsedNMEA.Altitude;
          }
        }
        else
        {
          z = 0.0;
        }
      }

      if (parsedNMEA.HasTrackAngle)
      {
        heading = parsedNMEA.TrackAngle;
      }
      else
      {
	heading = 0.0;
      }

      gpsUpdateTime = parsedNMEA.UTCSecondsOfDay;
      if (!hasLastGPSUpdateTime)
      {
        hasLastGPSUpdateTime = true;
        lastGPSUpdateTime = gpsUpdateTime;
      }
      else
      {
        if (gpsUpdateTime - lastGPSUpdateTime < - 12.0 * 3600.0)
        {
          // tod wrap detected
          GPSTimeOffset += 24.0 * 3600.0;
        }
      }

      convertedGPSUpdateTime = gpsUpdateTime + GPSTimeOffset;
      if (previousConvertedGPSUpdateTime < 0.0)
      {
        previousConvertedGPSUpdateTime = convertedGPSUpdateTime;
      }

      if (convertedGPSUpdateTime > previousConvertedGPSUpdateTime
          && parsedNMEA.Valid)
      {
        // We have detected that this position packet is the first one since
        // last gps fix (there are more position packets than there are fixes)
        // and that the new NMEA sentence refers to a valid fix,
        // so we can do an estimation of the timeshift.
        this->HasTimeshiftEstimation = true;
        // To understand this formula, think that we want to add this timeshift
        // to a lidar time to get a gps time, and that the "lidar instant" that
        // corresponds to the new fix is earlier than convertedLidarUpdateTime,
        // because of the time it took the information to go from GPS to Lidar.
        // Possible improvement: store the different estimations
        // (one per new fix) and return the median.
        this->TimeshiftMeasurements.push_back(
            convertedGPSUpdateTime -
            (1e-6 * convertedLidarUpdateTime - this->AssumedHardwareLag));
      }
      previousConvertedGPSUpdateTime = convertedGPSUpdateTime;
    }

    if (pointcount == 0)
    {
      this->Internal->Offset[0] = x;
      this->Internal->Offset[1] = y;
    }

    x -= this->Internal->Offset[0];
    y -= this->Internal->Offset[1];

    points->InsertNextPoint(x, y, z);
    lats->InsertNextValue(lat);
    lons->InsertNextValue(lon);
    gpsTime->InsertNextValue(convertedGPSUpdateTime);
    polyIds->InsertNextId(pointcount);

    times->InsertNextValue(convertedLidarUpdateTime);

    dataVectors["gyro1"]->InsertNextValue(position.gyro[0] * GYRO_SCALE);
    dataVectors["gyro2"]->InsertNextValue(position.gyro[1] * GYRO_SCALE);
    dataVectors["gyro3"]->InsertNextValue(position.gyro[2] * GYRO_SCALE);
    dataVectors["temp1"]->InsertNextValue(position.temp[0] * TEMP_SCALE + TEMP_OFFSET);
    dataVectors["temp2"]->InsertNextValue(position.temp[1] * TEMP_SCALE + TEMP_OFFSET);
    dataVectors["temp3"]->InsertNextValue(position.temp[2] * TEMP_SCALE + TEMP_OFFSET);
    dataVectors["accel1x"]->InsertNextValue(position.accelx[0] * ACCEL_SCALE);
    dataVectors["accel2x"]->InsertNextValue(position.accelx[1] * ACCEL_SCALE);
    dataVectors["accel3x"]->InsertNextValue(position.accelx[2] * ACCEL_SCALE);
    dataVectors["accel1y"]->InsertNextValue(position.accely[0] * ACCEL_SCALE);
    dataVectors["accel2y"]->InsertNextValue(position.accely[1] * ACCEL_SCALE);
    dataVectors["accel3y"]->InsertNextValue(position.accely[2] * ACCEL_SCALE);
    dataVectors["heading"]->InsertNextValue(heading);

    pointcount++;
  }
  this->Close();

  cells->InsertNextCell(polyLine);

  // Optionally interpolate the GPS values... note that we assume that the
  // first GPS point is not 0,0 if we have valid GPS data; otherwise we assume
  // that the GPS data is garbage and ignore it
  if (lats->GetNumberOfTuples() && lons->GetNumberOfTuples() &&
    (lats->GetValue(0) != 0.0 || lons->GetValue(0) != 0.0))
  {
    this->Internal->InterpolateGPS(points, gpsTime, times, dataVectors["heading"]);
  }

  output->SetPoints(points);
  output->SetLines(cells);
  output->GetPointData()->AddArray(lats);
  output->GetPointData()->AddArray(lons);
  output->GetPointData()->AddArray(gpsTime);
  output->GetPointData()->AddArray(times);
  for (VecMap::iterator it = dataVectors.begin(); it != dataVectors.end(); ++it)
  {
    output->GetPointData()->AddArray(it->second);
  }

  return 1;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLPositionReader::RequestInformation(
  vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  return this->Superclass::RequestInformation(request, inputVector, outputVector);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FileName: " << this->FileName << endl;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::SetUseGPGGASentences(bool useGPGGASentences)
{
  this->UseGPGGASentences = useGPGGASentences;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::Open()
{
  this->Close();
  this->Internal->Reader = new vtkPacketFileReader;
  if (!this->Internal->Reader->Open(this->FileName))
  {
    vtkErrorMacro("Failed to open packet file: " << this->FileName << endl
                                                 << this->Internal->Reader->GetLastError());
    this->Close();
  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::Close()
{
  delete this->Internal->Reader;
  this->Internal->Reader = 0;
}

//-----------------------------------------------------------------------------
double vtkVelodyneHDLPositionReader::GetTimeshiftEstimation() {
  if (this->TimeshiftMeasurements.size() == 0)
  {
    vtkGenericWarningMacro("Error : timeshift estimation asked"
                           " but no measurement available.")
    return 0.0;
  }
  return ComputeMedian(this->TimeshiftMeasurements);
}

//-----------------------------------------------------------------------------
std::string vtkVelodyneHDLPositionReader::GetTimeSyncInfo()
{
  std::string PPSDesc;
  if (this->LastPPSState == PPSState::PPS_ABSENT)
  {
    PPSDesc = "absent";
  }
  else if (this->LastPPSState == PPSState::PPS_ATTEMPTING_TO_SYNC)
  {
    PPSDesc = "attempting to sync";
  }
  else if (this->LastPPSState == PPSState::PPS_LOCKED)
  {
    PPSDesc = "locked";
  }
  else if (this->LastPPSState == PPSState::PPS_ERROR)
  {
    PPSDesc = "error";
  }
  else
  {
    PPSDesc = "unknown";
  }

  if  (!this->PPSSynced && this->HasTimeshiftEstimation)
  {
    std::ostringstream timeshiftEstimation;
    timeshiftEstimation << std::fixed << std::setprecision(6)
                        << this->GetTimeshiftEstimation();
    std::ostringstream assumedHWLag;
    assumedHWLag << std::fixed << std::setprecision(6)
                        << this->AssumedHardwareLag;
    vtkGenericWarningMacro("Recovered timeshift despite missing PPS sync: "
                        << timeshiftEstimation.str()
                        << " seconds (assuming hardware lag of: "
                        << assumedHWLag.str()
                        << " seconds)."
                        << " Add this value to the lidar timestamps (ToH)"
                        << " to get GPS UTC time (mod. 1 hour).");
  }

  return " Lidar PPS signal: "
      + PPSDesc
      + " - "
      + (this->PPSSynced ? "Lidar clock synced on GPS UTC ToH"
                         : "NO Lidar clock sync on GPS UTC ToH")
      + " ";
}
