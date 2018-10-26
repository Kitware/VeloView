#include "NMEAParser.h"

#include <iostream>
#include <iomanip>

#include "TestHelpers.h"

const double epsilon = 1e-9;

#define COMPARE_DISCRETE(varname, structure, flag)\
if (varname != structure.varname)\
{\
  std::cerr << #varname" does not match, got " << structure.varname\
  << " expected " << varname << std::endl;\
  flag = false;\
}

#define COMPARE_FLOAT(varname, structure, flag)\
if (!compare(&varname, &structure.varname, 1, epsilon))\
{\
  std::cerr << #varname" does not match, got " << std::setprecision(17)\
  << structure.varname\
  << " expected " << varname << std::endl;\
  flag = false;\
}


bool test_sentence(NMEAParser& parser,
                   std::string sentence,
                   bool Valid,
                   double Lat,
                   double Long,
                   double UTCSecondsOfDay,
                   bool HasAltitude,
                   double Altitude,
                   bool HasGeoidalSeparation,
                   double GeoidalSeparation,
                   bool HasTypeOfFix,
                   NMEALocation::FixType TypeOfFix,
                   bool HasHorizontalDOP,
                   double HorizontalDOP,
                   bool HasSpeed,
                   double Speed,
                   bool HasTrackAngle,
                   double TrackAngle,
                   bool HasDate,
                   int DateDay,
                   int DateMonth,
                   int DateYear,
                   bool HasFAA,
                   NMEALocation::FAAMode FAA)
{
  std::cout << "Testing sentence: <" << sentence << ">" << std::endl;
  NMEALocation location;
  bool corresponds = true;
  if (!parser.ParseLocation(sentence, location))
  {
      std::cerr << "Checksum computed to be: (decimal notation): "
                << parser.ComputeChecksum(sentence)
                << std::endl;
      std::cerr << "Could not parse the sentence" << std::endl;
      return false;
  }
  COMPARE_DISCRETE(Valid, location, corresponds)
  COMPARE_FLOAT(Lat, location, corresponds)
  COMPARE_FLOAT(Long, location, corresponds)
  COMPARE_FLOAT(UTCSecondsOfDay, location, corresponds)
  COMPARE_DISCRETE(HasAltitude, location, corresponds)
  if (HasAltitude)
  {
    COMPARE_FLOAT(Altitude, location, corresponds)
  }
  COMPARE_DISCRETE(HasGeoidalSeparation, location, corresponds)
  if (HasGeoidalSeparation)
  {
    COMPARE_FLOAT(GeoidalSeparation, location, corresponds)
  }
  COMPARE_DISCRETE(HasTypeOfFix, location, corresponds)
  if (HasTypeOfFix)
  {
    COMPARE_DISCRETE(TypeOfFix, location, corresponds)
  }
  COMPARE_DISCRETE(HasHorizontalDOP, location, corresponds)
  if (HasHorizontalDOP)
  {
    COMPARE_FLOAT(HorizontalDOP, location, corresponds)
  }
  COMPARE_DISCRETE(HasSpeed, location, corresponds)
  if (HasSpeed)
  {
    COMPARE_FLOAT(Speed, location, corresponds)
  }
  COMPARE_DISCRETE(HasTrackAngle, location, corresponds)
  if (HasTrackAngle)
  {
    COMPARE_FLOAT(TrackAngle, location, corresponds)
  }
  COMPARE_DISCRETE(HasDate, location, corresponds)
  if (HasDate)
  {
    COMPARE_DISCRETE(DateDay, location, corresponds)
    COMPARE_DISCRETE(DateMonth, location, corresponds)
    COMPARE_DISCRETE(DateYear, location, corresponds)
  }
  COMPARE_DISCRETE(HasFAA, location, corresponds)
  if (HasFAA)
  {
    COMPARE_DISCRETE(FAA, location, corresponds)
  }

  return corresponds;
}


int main(int argc, char* argv[])
{
  double unused_double = 42.0;
  int unused_int = 42;
  NMEALocation::FixType unused_type_of_fix = NMEALocation::GPS_FIX;
  NMEAParser parser;

  // test_sentence(parser, "test", true);
  bool allgood = true;
  // took this sequence on http://www.gpsinformation.org/dale/nmea.htm#GGA
  // changed N->S and E->W
  // checksum recomputed
  allgood &= test_sentence(parser,
                "$GPGGA,123519.5,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*53",
                true, // Valid
                - 48.1173, // Lat
                - 11.516666667, // Long
                45319.5, // UTC seconds
                true, // has altitude
                545.4, // altitude
                true, // has geoidal separation
                46.9, // geoidal separation
                true, // has type of fix
                NMEALocation::GPS_FIX, // GPS fix
                true, // has horizontal DOP
                0.9, // horizontal DOP
                false, // does not has speed
                unused_double,
                false, // does not has track angle
                unused_double,
                false, // does not has date
                unused_int,
                unused_int,
                unused_int,
                false, // no FAA mode
                NMEALocation::UNDEFINED_FAA);

  allgood &= test_sentence(parser,
                           "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,*46",
                           true, // Valid
                           48.1173, // Lat
                           11.516666667, // Long
                           45319.0, // UTC seconds
                           false, // has altitude
                           unused_double,
                           false, // has geoidal separation
                           unused_double,
                           false, // has type of fix
                           unused_type_of_fix,
                           false, // has horizontal DOP
                           unused_double, // horizontal DOP
                           true, // has speed
                           22.4,
                           true, // has track angle
                           84.4,
                           true, // has date
                           23,
                           03,
                           94,
                           false, // no FAA mode
                           NMEALocation::UNDEFINED_FAA);

  // same sentence, but with FAA mode
  allgood &= test_sentence(parser,
                           "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,A,*2B",
                           true, // Valid
                           48.1173, // Lat
                           11.516666667, // Long
                           45319.0, // UTC seconds
                           false, // has altitude
                           unused_double,
                           false, // has geoidal separation
                           unused_double,
                           false, // has type of fix
                           unused_type_of_fix,
                           false, // has horizontal DOP
                           unused_double, // horizontal DOP
                           true, // has speed
                           22.4,
                           true, // has track angle
                           84.4,
                           true, // has date
                           23,
                           03,
                           94,
                           true,
                           NMEALocation::AUTONOMOUS_FAA);

  allgood &= test_sentence(parser,
                           "$GPGLL,4916.45,N,12311.12,W,225444,A,*1D",
                           true, // Valid
                           49.274166667, // Lat
                           -123.185333333, // Long
                           82484.0, // UTC seconds
                           false, // has altitude
                           unused_double,
                           false, // has geoidal separation
                           unused_double,
                           false, // has type of fix
                           unused_type_of_fix,
                           false, // has horizontal DOP
                           unused_double, // horizontal DOP
                           false, // has speed
                           unused_double,
                           false, // has track angle
                           unused_double,
                           false, // has date
                           unused_int,
                           unused_int,
                           unused_int,
                           false, // no FAA
                           NMEALocation::UNDEFINED_FAA);

  // same sentence, with FAA (D: DIFFERENTIAL)
  allgood &= test_sentence(parser,
                           "$GPGLL,4916.45,N,12311.12,W,225444,A,D,*75",
                           true, // Valid
                           49.274166667, // Lat
                           -123.185333333, // Long
                           82484.0, // UTC seconds
                           false, // has altitude
                           unused_double,
                           false, // has geoidal separation
                           unused_double,
                           false, // has type of fix
                           unused_type_of_fix,
                           false, // has horizontal DOP
                           unused_double, // horizontal DOP
                           false, // has speed
                           unused_double,
                           false, // has track angle
                           unused_double,
                           false, // has date
                           unused_int,
                           unused_int,
                           unused_int,
                           true, // no FAA
                           NMEALocation::DIFFERENTIAL_FAA);


  return allgood ? 0 : 1;
}
