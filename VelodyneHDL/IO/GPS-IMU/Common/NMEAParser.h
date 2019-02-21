#ifndef NMEAPARSER_H
#define NMEAPARSER_H

#include <string>
#include <vector>
#include <vvConfigure.h>

struct NMEALocation;

/**
 * @brief NMEAParser parses a NMEA 0183 sentence that provides location data
 * (GPRMC, GPGGA or GPGLL sequence).
 *
 * If the sentence can be parsed, the result is stored inside a NMEALocation
 * structure.
 */
class VelodyneHDLPlugin_EXPORT NMEAParser
{
public:
  std::vector<std::string> SplitWords(const std::string& sentence);
  bool IsGPGLL(const std::vector<std::string>& w);
  bool IsGPGGA(const std::vector<std::string>& w);
  bool IsGPRMC(const std::vector<std::string>& w);
  /** @name ParseLocation functions
   * @brief Parse a NMEA 0183 sentence that provides a location
   * @param sentence must be a string starting with $GP{RMC,GGA,GLL}
   *
   * @return Returns true if the sentence could be processed
   * and could provide a location. <br>
   * (Note that it does not mean that the GPS has a fix and provides valid
   * lat/lon. For that you have to check the member "Valid".) <br>
   * If false is returned, do not use the struct location.
   */
  ///@{
  bool ParseGPRMC(const std::vector<std::string>& w, NMEALocation& location);
  bool ParseGPGGA(const std::vector<std::string>& w, NMEALocation& location);
  bool ParseGPGLL(const std::vector<std::string>& w, NMEALocation& location);
  bool ParseLocation(const char* sentence, NMEALocation& location);
  bool ParseLocation(const std::string& sentence, NMEALocation& location);
  ///@}

  /** @name Check that the checksum is valid
   * @brief Compute the checksum and compare it with the one at then end of the
   * sentence
   * @param sentence must be a string starting with $, ending with *XX
   * with XX the checksum encoded as two hexadecimal characters (0-9, A-F),
   * the most-significant-nibble being sent first
   * (source: http://catb.org/gpsd/NMEA.html)
   *
   * @return Returns true if the sentence has a valid checksum
   */
  bool ChecksumValid(const std::string& sentence);
  unsigned int ReadChecksum(const std::string& sentence);
  unsigned int ComputeChecksum(const std::string& sentence);
};

/**
 * @brief NMEALocation stores the data of a GPRMC, GPGGA or GPGLL sequence

 *
 *
 * - the record is "valid" if it can be used (location is known up to an extent)
 * To be valid:
 *     - a GPGGA record requires the quality of the fix to not be empty or "0"
 *     - a GPRMC record requires the field validity to be "A",
 * We also require the fields: UTC seconds of day, latitude and longitude.
 *
 * - UTC seconds of day (looping counter, between 0 and 86400, double)
 * - latitude (double)
 * - longitude (double)
 *
 * Optionaly we store if available:
 * - altitude above mean sea level (meters)
 * - height of geoid (mean sea level) above WGS84 (meters) <br>
 * http://www.gpsinformation.org/dale/nmea.htm tell us about height of geoid: <br>
 * "If the height of geoid is missing then the altitude should be suspect.
 * Some non-standard implementations report altitude with respect to the
 * ellipsoid rather than geoid altitude. Some units do not report negative
 * altitudes at all. This is the only sentence that reports altitude."
 * - quality (NO_FIX, GPS, DIFFERENTIAL_GPS) (requires GPGGA)
 * - horizontal DOP (no unit, the lower the better. 1.0 is good, 8.0 bad)
 * - date of fix (day, month, year: 3 positive integers)
 * - speed (in knots)
 * - track angle (in degrees)
 * - FAA mode indicator (RMC and GLL only)
 *
 * We could also store:
 * - Magnetic variation (RMC only)
 * - Number of satellites used (less useful that horizontal DOP) (GGA only)
 * - Age of differential GPS data (GGA only)
 * - Differential reference station id (GGA only)
 *
 * Warning: {GPGGA, GPRMC, GPGLL} are not equivalent:
 * - only GPGGA provides:
 * altitude, geoidal separation, type of fix, horizontal dilution of precision
 * - only GPRMC provides:
 * speed, track angle, date of day, magnetic variation
 * - only {GPRMC, GPGLL} provide:
 */

struct NMEALocation {
public:
  enum FixType { UNDEFINED_FIX = 0,
                 NO_FIX,
                 GPS_FIX,
                 DIFFERENTIAL_GPS_FIX,
                 PPS_FIX,
                 RTK_FIX,
                 FLOAT_RTK_FIX,
                 ESTIMATED_FIX,
                 MANUAL_INPUT_FIX,
                 SIMULATION_FIX };
  enum FAAMode { UNDEFINED_FAA = 0,
                 AUTONOMOUS_FAA,
                 DIFFERENTIAL_FAA,
                 ESTIMATED_FAA,
                 MANUAL_FAA,
                 SIMULATED_FAA,
                 DATA_NOT_VALID_FAA,
                 PRECISE_FAA };
  bool Valid;
  double Lat;
  double Long;
  double UTCSecondsOfDay;
  bool HasAltitude;
  double Altitude;
  bool HasGeoidalSeparation;
  double GeoidalSeparation;
  bool HasTypeOfFix;
  FixType TypeOfFix;
  bool HasHorizontalDOP;
  double HorizontalDOP;
  bool HasSpeed;
  double Speed;
  bool HasTrackAngle;
  double TrackAngle;
  bool HasDate;
  int DateDay;
  int DateMonth;
  int DateYear;
  bool HasFAA;
  FAAMode FAA;

  void Init();
};

#endif // NMEAPARSER_H
