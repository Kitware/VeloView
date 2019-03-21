#include "vtkArduPilotDataFlashLogReader.h"

#include <stdio.h>

#include <vtkObjectFactory.h>
#include <vtkInformationVector.h>

#include <boost/algorithm/string.hpp>

#include <Eigen/Geometry>

#include "GPSProjectionUtils.h"

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkArduPilotDataFlashLogReader)

//-----------------------------------------------------------------------------
vtkArduPilotDataFlashLogReader::vtkArduPilotDataFlashLogReader()
{
  this->GPSTrajectory = vtkSmartPointer<vtkTemporalTransforms>::New();
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
int vtkArduPilotDataFlashLogReader::RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *outputVector)
{
  std::ifstream f(this->FileName);
  if (!f.good())
  {
    vtkErrorMacro("Failed to open input file \"" << this->FileName << "\"");
    return VTK_ERROR;
  }

  bool offsetFound = false;
  Eigen::Vector3d offset;
  std::string line;
  while (std::getline(f, line))
  {
    boost::algorithm::trim(line);
    if (line.empty())
    {
      continue;
    }

    if (boost::starts_with(line, "GPS2"))
    {
      // GPS2 lines contain fields:
      // TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,U
      // Useful: https://groups.google.com/forum/#!topic/swiftnav-discuss/XOr7WQto9ZI
      // quoting from: https://discuss.ardupilot.org/t/correct-gps-time-stamp/14329
      // "GMS and GWK would be the best way to get the current gps time yes.
      // the tick with that though is there is latency vs the measurement data.
      // so correlating that data could be interesting."
      // GMS = GPS ms since beginning of week
      // GWk = None # GPS week (a GPS week is 7*24h afaik)
      // Status: 0 = no GPS, 1 = GPS but no fix, 2 = GPS with 2D fix, 3 = GPS with 3D fix


      std::vector<std::string> elements;
      boost::algorithm::split(
        elements, line, boost::is_any_of(", "), boost::token_compress_on);

      double GMS = std::stod(elements[3]); // ms since beginning of GPS week
      double lat = std::stod(elements[7]); // latitude in degrees
      double lng = std::stod(elements[8]); // longitude in degrees
      // I beleive "Alt" is altitude over ellipsoid (WGS84, standard for GPS)
      // but it could be height above geoid.
      double alt = std::stod(elements[9]); // in meters

      double Time = GMS / 1e3;

      double z = alt;
      double easting, northing;
      UTMProjector proj;
      proj.Project(lat, lng, easting, northing);
      Eigen::Vector3d  position;
      position << easting, northing, z; // ENU referential (right hand oriented)
      if (offsetFound)
      {
        position = position - offset;
      }
      else
      {
        this->SignedUTMZone = proj.SignedUTMZone;
        offset = position;
        position = Eigen::Vector3d::Zero();
        this->Offset[0] = offset[0];
        this->Offset[1] = offset[1];
        this->Offset[2] = offset[2];
        offsetFound = true;
      }

      GPSTrajectory->PushBack(Time + TimeOffset, Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()), position);
    }
  }

  auto *output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  output->ShallowCopy(this->GPSTrajectory);
  return VTK_OK;
}
