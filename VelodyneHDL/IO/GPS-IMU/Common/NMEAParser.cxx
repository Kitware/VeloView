#include "NMEAParser.h"

#include <cstring>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <limits>

#include <vtkMath.h>

// Silence one "unused variable" warning https://stackoverflow.com/q/1486904/
#define UNUSED(expr) do { (void)(expr); } while (0)

namespace {
  /* parse in format HHMMSS.SS (.SS optional) */
  bool ParseUTCSecondsOfDay(const std::vector<std::string>& w,
                    unsigned int pos,
                    NMEALocation& location)
  {
    try {
      double read = std::stod(w[pos]);
      double integral_part;
      std::modf(read, &integral_part);
      double fractional_part = read - integral_part;
      int HHMMSS = static_cast<int>(vtkMath::Round(integral_part));
      int SS = HHMMSS % 100;
      int MM = ((HHMMSS - SS) % 10000) / 100;
      int HH = (HHMMSS - SS - 100 * MM) / 10000;
      location.UTCSecondsOfDay =
          fractional_part
          + static_cast<double>(SS)
          + 60.0 * static_cast<double>(MM)
          + 3600.0 * static_cast<double>(HH);
    }
    catch (const std::logic_error&) {
      return false;
    }
    return true;
  }

  bool ParseFAA(const std::vector<std::string>& w,
                    unsigned int pos,
                    NMEALocation& location)
  {
    if (w[pos] == "")
    {
      location.HasFAA = false;
      location.FAA = NMEALocation::UNDEFINED_FAA;
    }
    else if (w[pos] == "A")
    {
      location.HasFAA = true;
      location.FAA = NMEALocation::AUTONOMOUS_FAA;
    }
    else if (w[pos] == "D")
    {
      location.HasFAA = true;
      location.FAA = NMEALocation::DIFFERENTIAL_FAA;
    }
    else if (w[pos] == "E")
    {
      location.HasFAA = true;
      location.FAA = NMEALocation::ESTIMATED_FAA;
    }
    else if (w[pos] == "M")
    {
      location.HasFAA = true;
      location.FAA = NMEALocation::MANUAL_FAA;
    }
    else if (w[pos] == "S")
    {
      location.HasFAA = true;
      location.FAA = NMEALocation::SIMULATED_FAA;
    }
    else if (w[pos] == "N")
    {
      location.HasFAA = true;
      location.FAA = NMEALocation::DATA_NOT_VALID_FAA;
    }
    else if (w[pos] == "P")
    {
      location.HasFAA = true;
      location.FAA = NMEALocation::PRECISE_FAA;
    }

    return true;
  }

  bool ParseLatLong(const std::vector<std::string>& w,
                    unsigned int uLat,
                    unsigned int latNS,
                    unsigned int uLong,
                    unsigned int lonEW,
                    NMEALocation& location)
  {
    /* Latitude and Longitude */
    // We make the fields ULAT, ULONG, LATNS and LONGEW mandatory
    double latDec = 0.0;
    double lonDec = 0.0;
    try {
      latDec = std::stod(w[uLat]);
      lonDec = std::stod(w[uLong]);
    }
    catch (const std::logic_error&) {
      return false;
    }
    double latDeg = std::floor(latDec / 100.0);
    double lonDeg = std::floor(lonDec / 100.0);
    // was written in existing code (vtkVelodyneHDLPositionReader):
    // double latMin = 100.0 * (latDec / 100.0 - latDeg);
    // TODO: check it would improve precision
    double latMin = latDec - 100.0 * latDeg;
    double lonMin = lonDec - 100.0 * lonDeg;
    double lat = latDeg + latMin / 60.0;
    double lon = lonDeg + lonMin / 60.0;
    if (w[latNS] == "S")
    {
      lat = -lat;
    }
    else if (w[latNS] != "N")
    {
      // we decided this field was mandatory
      return false;
    }
    if (w[lonEW] == "W")
    {
      lon = -lon;
    }
    else if (w[lonEW] != "E")
    {
      // we decided this field was mandatory
      return false;
    }
    location.Lat = lat;
    location.Long = lon;

    return true;
  }
}


//------------------------------------------------------------------------------
void NMEALocation::Init()
{
  this->Valid = false;
  this->Lat = 0.0;
  this->Long = 0.0;
  this->UTCSecondsOfDay = 0;
  this->HasAltitude = false;
  this->Altitude = 0.0;
  this->HasGeoidalSeparation = false;
  this->GeoidalSeparation = 0.0;
  this->HasTypeOfFix = false;
  this->TypeOfFix = NMEALocation::UNDEFINED_FIX;
  this->HasHorizontalDOP = false;
  this->HorizontalDOP = std::numeric_limits<double>::max();
  this->HasSpeed = false;
  this->Speed = 0.0;
  this->HasTrackAngle = false;
  this->TrackAngle = 0.0;
  this->HasDate = false;
  this->DateDay = 0;
  this->DateMonth = 0;
  this->DateYear = 0;
  this->HasFAA = false;
  this->FAA = NMEALocation::UNDEFINED_FAA;
}


//------------------------------------------------------------------------------
bool NMEAParser::ChecksumValid(const std::string& sentence)
{
  unsigned int computed = ComputeChecksum(sentence);
  unsigned int read = ReadChecksum(sentence);
  // (checks that we do not have a return corresponding to an error)
  return read == computed && read != std::numeric_limits<unsigned int>::max();
}


//------------------------------------------------------------------------------
unsigned int NMEAParser::ReadChecksum(const std::string& sentence)
{
  if (sentence.size() < 2)
  {
    // returns a value that does not fit in a byte,
    // can be used to detect that Checksum is not readable
    return std::numeric_limits<unsigned int>::max();
  }

  std::string checksumString = sentence.substr(sentence.size() - 2, 2);
  unsigned int checksumByte; // does not seem to work with unsigned char
  std::stringstream ss;
  ss << std::hex << checksumString;
  ss >> checksumByte;
  return checksumByte;
}


//------------------------------------------------------------------------------
unsigned int NMEAParser::ComputeChecksum(const std::string& sentence)
{
  const char* str = sentence.c_str();
  if (strlen(str) < 1 + 1 + 2) /* at least: $, *, checksum */
  {
    return std::numeric_limits<unsigned int>::max();
  }

  unsigned int computed = 0;
  for (int i = 1; i < static_cast<int>(strlen(str)) - 3; i++)
  {
    computed ^= static_cast<unsigned int>(str[i]);
  }

  return computed;
}


//------------------------------------------------------------------------------
bool NMEAParser::ParseGPRMC(const std::vector<std::string>& w,
                            NMEALocation& location)
{
  const unsigned int RMC_UTC_TIME = 1;
  const unsigned int RMC_STATUS = 2;
  const unsigned int RMC_ULAT = 3;
  const unsigned int RMC_LATNS = 4;
  const unsigned int RMC_ULONG = 5;
  const unsigned int RMC_LONGEW = 6;
  const unsigned int RMC_SPEED = 7;
  const unsigned int RMC_ANGLE = 8;
  const unsigned int RMC_DATE = 9;
  const unsigned int RMC_MAGVAR = 10;
  const unsigned int RMC_MAGVAREW = 11;
  const unsigned int RMC_FAA = 12; // ! warning: present only if version >= 2.3
  /* UTC seconds of day */
  // We make the field mandatory
  if (w[RMC_UTC_TIME] == "")
  {
    return false;
  }
  else if (!ParseUTCSecondsOfDay(w, RMC_UTC_TIME, location))
  {
    return false;
  }


  /* Validity */
  // We make this field mandatory
  if (w[RMC_STATUS] == "V")
  {
    location.Valid = false;
  }
  else if (w[RMC_STATUS] == "A")
  {
    location.Valid = true;
  }
  else
  {
    return false;
  }


  /* Latitude and Longitude */
  // We make the fields ULAT, ULONG, LATNS and LONGEW mandatory
  if (!ParseLatLong(w, RMC_ULAT, RMC_LATNS, RMC_ULONG, RMC_LONGEW, location))
  {
    return false;
  }


  /* Speed */
  if (w[RMC_SPEED] != "")
  {
    location.HasSpeed = true;
    try {
      location.Speed = std::stod(w[RMC_SPEED]);
    }
    catch (const std::logic_error&) {
      return false;
    }
  }
  else
  {
    location.HasSpeed = false;
    location.Speed = 0.0;
  }


  /* Angle */
  if (w[RMC_ANGLE] != "")
  {
    location.HasTrackAngle = true;
    try {
      location.TrackAngle = std::stod(w[RMC_ANGLE]);
    }
    catch (const std::logic_error&) {
      return false;
    }
  }
  else
  {
    location.HasTrackAngle = false;
    location.TrackAngle = 0.0;
  }


  /* Date */
  if (w[RMC_DATE] != "")
  {
    if (w[RMC_DATE].size() != 6)
    {
      return false;
    }
    location.HasDate = true;
    try {
      location.DateDay = static_cast<int>(
                              std::stoul(w[RMC_DATE].substr(0, 2)));
      location.DateMonth = static_cast<int>(
                              std::stoul(w[RMC_DATE].substr(2,2)));
      location.DateYear = static_cast<int>(
                              std::stoul(w[RMC_DATE].substr(4,2)));
    }
    catch (const std::logic_error&) {
      return false;
    }
  }
  else
  {
    location.HasDate = false;
  }


  /* Magnetic variation not parsed */
  UNUSED(RMC_MAGVAR);
  UNUSED(RMC_MAGVAREW);


  /* FAA mode */
  if (w.size() == 14 && !ParseFAA(w, RMC_FAA, location))
  {
    return false;
  }

  return true;
}


//------------------------------------------------------------------------------
bool NMEAParser::ParseGPGGA(const std::vector<std::string>& w,
                            NMEALocation& location)
{
  const unsigned int GGA_UTC_TIME = 1;
  const unsigned int GGA_ULAT = 2;
  const unsigned int GGA_LATNS = 3;
  const unsigned int GGA_ULONG = 4;
  const unsigned int GGA_LONGEW = 5;
  const unsigned int GGA_QUALITY = 6;
  const unsigned int GGA_SAT_COUNT = 7;
  const unsigned int GGA_HDOP = 8;
  const unsigned int GGA_ALT= 9;
  const unsigned int GGA_ALTUNIT = 10;
  const unsigned int GGA_GEOSEP = 11;
  const unsigned int GGA_GEOSEPUNIT = 12;
  const unsigned int GGA_AGE = 13;
  const unsigned int GGA_DIF_STATION = 14;
  /* UTC seconds of day */
  // We make the field mandatory
  if (w[GGA_UTC_TIME] == "")
  {
    return false;
  }
  else if (!ParseUTCSecondsOfDay(w, GGA_UTC_TIME, location))
  {
    return false;
  }

  /* Latitude and Longitude */
  // We make the fields ULAT, ULONG, LATNS and LONGEW mandatory
  if (!ParseLatLong(w, GGA_ULAT, GGA_LATNS, GGA_ULONG, GGA_LONGEW, location))
  {
    return false;
  }


  /* "Quality" (it is more about the type of fix) */
  if (w[GGA_QUALITY] == "")
  {
    location.HasTypeOfFix = false;
  }
  else
  {
    location.HasTypeOfFix = true;
    try {
      int quality = static_cast<int>(std::stoul(w[GGA_QUALITY]));
      switch (quality) {
        case 0:
          location.TypeOfFix = NMEALocation::NO_FIX;
          break;
        case 1:
          location.TypeOfFix = NMEALocation::GPS_FIX;
          break;
        case 2:
          location.TypeOfFix = NMEALocation::DIFFERENTIAL_GPS_FIX;
          break;
        case 3:
          location.TypeOfFix = NMEALocation::PPS_FIX;
          break;
        case 4:
          location.TypeOfFix = NMEALocation::RTK_FIX;
          break;
        case 5:
          location.TypeOfFix = NMEALocation::FLOAT_RTK_FIX;
          break;
        case 6:
          location.TypeOfFix = NMEALocation::ESTIMATED_FIX;
          break;
        case 7:
          location.TypeOfFix = NMEALocation::MANUAL_INPUT_FIX;
          break;
        case 8:
          location.TypeOfFix = NMEALocation::SIMULATION_FIX;
          break;
        default:
          location.TypeOfFix = NMEALocation::UNDEFINED_FIX;
          return false;
      }
    }
    catch (const std::logic_error&) {
      return false;
    }
  }


  /* Number of satellites not parsed */
  UNUSED(GGA_SAT_COUNT);


  /* Horizontal dilution of precision */
  if (w[GGA_HDOP] != "")
  {
    location.HasHorizontalDOP = true;
    try {
      location.HorizontalDOP = std::stod(w[GGA_HDOP]);
    }
    catch (const std::logic_error&) {
      return false;
    }
  }
  else
  {
    location.HasHorizontalDOP = false;
    location.HorizontalDOP = std::numeric_limits<double>::max();
  }


  /* Antenna Altitude above/below sea level */
  if (w[GGA_ALT] != "")
  {
    // makes the unit field mandatory
    if (w[GGA_ALTUNIT] == "M")
    {
      location.HasAltitude = true;
      try {
        location.Altitude = std::stod(w[GGA_ALT]);
      }
      catch (const std::logic_error&) {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  else
  {
    location.HasAltitude = false;
    location.Altitude = 0.0;
  }


  /* Geoidal separation */
  if (w[GGA_GEOSEP] != "")
  {
    // makes the unit field mandatory
    if (w[GGA_GEOSEPUNIT] == "M")
    {
      location.HasGeoidalSeparation = true;
      try {
        location.GeoidalSeparation = std::stod(w[GGA_GEOSEP]);
      }
      catch (const std::logic_error&) {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  else
  {
    location.HasGeoidalSeparation = false;
    location.GeoidalSeparation = 0.0;
  }


  /* Age of differential GPS data not parsed */
  UNUSED(GGA_AGE);
  /* Differential reference station id not parsed */
  UNUSED(GGA_DIF_STATION);


  /* Now set fields of location for which GPGGA does not provide data: */
  location.Valid = (location.HasTypeOfFix
                    && location.TypeOfFix != NMEALocation::NO_FIX
                    && location.TypeOfFix != NMEALocation::UNDEFINED_FIX);
  return true;
}


//------------------------------------------------------------------------------
bool NMEAParser::ParseGPGLL(const std::vector<std::string>& w,
                            NMEALocation& location)
{
  const unsigned int GLL_ULAT = 1;
  const unsigned int GLL_LATNS = 2;
  const unsigned int GLL_ULONG = 3;
  const unsigned int GLL_LONGEW = 4;
  const unsigned int GLL_UTC_TIME = 5;
  const unsigned int GLL_STATUS = 6;
  const unsigned int GLL_FAA = 7; //
  /* Latitude and Longitude */
  // We make the fields ULAT, ULONG, LATNS and LONGEW mandatory
  if (!ParseLatLong(w, GLL_ULAT, GLL_LATNS, GLL_ULONG, GLL_LONGEW, location))
  {
    return false;
  }


  /* UTC seconds of day */
  // We make the field mandatory
  if (w[GLL_UTC_TIME] == "")
  {
    return false;
  }
  else if (!ParseUTCSecondsOfDay(w, GLL_UTC_TIME, location))
  {
    return false;
  }


  /* Validity */
  // We make this field mandatory
  if (w[GLL_STATUS] == "V")
  {
    location.Valid = false;
  }
  else if (w[GLL_STATUS] == "A")
  {
    location.Valid = true;
  }
  else
  {
    return false;
  }


  /* FAA mode */
  if (w.size() == 9 && !ParseFAA(w, GLL_FAA, location))
  {
    return false;
  }

  return true;
}


//------------------------------------------------------------------------------
bool NMEAParser::IsGPRMC(const std::vector<std::string>& w)
{
  const unsigned int NAME = 0;
  return w.size() > 0
      && w[NAME] == "$GPRMC"
      && (w.size() == 13 || w.size() == 14);
}


//------------------------------------------------------------------------------
bool NMEAParser::IsGPGGA(const std::vector<std::string>& w)
{
  const unsigned int NAME = 0;
  return w.size() > 0
      && w[NAME] == "$GPGGA"
      && w.size() == 15; // On first read of catdb.org I understood 16
}


//------------------------------------------------------------------------------
bool NMEAParser::IsGPGLL(const std::vector<std::string>& w)
{
  const unsigned int NAME = 0;
  return w.size() > 0
      && w[NAME] == "$GPGLL"
      && (w.size() == 8 || w.size() == 9);
}


//------------------------------------------------------------------------------
bool NMEAParser::ParseLocation(const std::string& sentence, NMEALocation& location)
{
  // reset location. This is important to do because no sentence can fill
  // all NMEALocation fields.
  location.Init();
  std::vector<std::string> w = SplitWords(sentence);
  if (w.size() < 1)
  {
    // the sequence is empty, so it contains no location
    return false;
  }

  if (!ChecksumValid(sentence))
  {
    return false;
  }

  // Look for sentences providing location,
  // and check that their length is correct.
  // There can be multiple possible length because "FAA mode indicator" is
  // present in NMEA 2.3 and later.
  // This "FAA mode indicator" is not present in Velodyne relay packets of
  // file "HDL32-V2_R into Butterfield into Digital Drive.pcap".
  if (IsGPRMC(w))
  {
    return ParseGPRMC(w, location);
  }
  if (IsGPGGA(w))
  {
    return ParseGPGGA(w, location);
  }
  else if (IsGPGLL(w))
  {
    return ParseGPGLL(w, location);
  }
  else
  {
    // not a location sentence, or sentence with invalid length
    return false;
  }
}


//------------------------------------------------------------------------------
bool NMEAParser::ParseLocation(const char* sentence, NMEALocation& location)
{
  return this->ParseLocation(std::string(sentence), location);
}


//------------------------------------------------------------------------------
std::vector<std::string> NMEAParser::SplitWords(const std::string& sentence)
{
  std::stringstream sstr(sentence);
  std::string token;

  std::vector<std::string> result;

  while (std::getline(sstr, token, ','))
  {
    result.push_back(token);
  }

  return result;
}
