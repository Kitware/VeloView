#include "GPSProjectionUtils.h"

#include <sstream>
#include <cassert>

#include <vtkObject.h>
#include <vtkSetGet.h>


//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
UTMProjector::~UTMProjector()
{
  if (this->IsInitialized())
  {
    pj_free(this->pj_utm);
  }
}

//------------------------------------------------------------------------------
void UTMProjector::Project(double lat, double lon, double &easting, double &northing)
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

  // I checked the correspondence between xy.u/v and  easting, northing
  // against a reliable converter for point: lat=4.613473, longitude=41.080385
  // (easting increases when you go east, northing increases when you go north)
  easting = xy.u;
  northing = xy.v;
}

//------------------------------------------------------------------------------
bool UTMProjector::IsInitialized()
{
  return this->pj_utm != nullptr;
}

//------------------------------------------------------------------------------
void UTMProjector::Init(double initial_lat, double initial_lon)
{
  assert(!pj_utm);
  const int unsignedZone = LatLongToZone(initial_lat, initial_lon);
  if (initial_lat < 0)
  {
    this->SignedUTMZone = - unsignedZone;
  }
  else
  {
    this->SignedUTMZone = unsignedZone;
  }
  std::stringstream utmparams;
  utmparams << "+proj=utm ";
  std::stringstream zone;
  zone << "+zone=" << unsignedZone;
  this->UTMString = zone.str();
  // WARNING: Dont let the string stream pass out of scope until
  // we finish initialization
  utmparams << this->UTMString << " ";
  if (this->SignedUTMZone < 0)
  {
    utmparams << "+south ";
  }

  utmparams << "+ellps=WGS84 ";
  utmparams << "+units=m ";
  utmparams << "+no_defs ";
  pj_utm = pj_init_plus(utmparams.str().c_str());
}
