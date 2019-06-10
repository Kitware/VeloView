#ifndef VTKVELODYNEPACKETINTERPRETERCOMMON_H
#define VTKVELODYNEPACKETINTERPRETERCOMMON_H

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/preprocessor.hpp>

#include <vtkTable.h>

#include "vtkDataPacket.h"
using namespace DataPacketFixedLength;

//------------------------------------------------------------------------------
// Enum macros.
//------------------------------------------------------------------------------
/*
 * Define some macros to facilitate maintenance of different enum types. These
 * allow a single macro to define an enum type with values, an overloaded
 * ToString function to convert macro values to strings, and a templated
 * To<enum> function to convert integral values to enums.
 *
 * For details of the Boost preprocessing macros, see
 * https://www.boost.org/doc/libs/1_67_0/libs/preprocessor/doc/AppendixA-AnIntroductiontoPreprocessorMetaprogramming.html
 */

//! @brief Internal macro for defining enum values.
#define DEFINE_ENUM_VALUES_INTERNAL(r, prefix, pair) \
  BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair)) = BOOST_PP_TUPLE_ELEM(1, pair)

//! @brief Macro for defining enum values.
#define DEFINE_ENUM_VALUES(name, prefix, enumerators) \
  enum name {                                         \
    BOOST_PP_SEQ_ENUM(                                \
      BOOST_PP_SEQ_TRANSFORM(                         \
        DEFINE_ENUM_VALUES_INTERNAL,                  \
        prefix,                                       \
        enumerators                                   \
      )                                               \
    )                                                 \
  };


//! @brief Internal macro for defining enum string conversions.
#define DEFINE_ENUM_STRING_CASES_INTERNAL(r, prefix, pair) \
  case BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair)): return BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(0, pair));


//! @brief Macro for defining enum string conversions.
#define DEFINE_ENUM_TO_STRING(name, prefix, enumerators)                                \
  inline                                                                                \
  char const * toString(name const x)                                                   \
  {                                                                                     \
    switch(x)                                                                           \
    {                                                                                   \
      BOOST_PP_SEQ_FOR_EACH(                                                            \
        DEFINE_ENUM_STRING_CASES_INTERNAL,                                              \
        prefix,                                                                         \
        enumerators                                                                     \
      )                                                                                 \
      default: return "<unrecognized enum value of type " BOOST_PP_STRINGIZE(name) ">"; \
    }                                                                                   \
  }


//! @brief Internal macro for converting integral values to enums.
#define DEFINE_VALUE_TO_ENUM_CASES_INTERNAL(r, prefix, pair) \
  case static_cast<T>(BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair))): return BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair));


//! @brief Macro for converting integral values to enums.
#define DEFINE_VALUE_TO_ENUM(name, prefix, enumerators, default_value) \
  template <typename T>                                                \
  inline                                                               \
  name to ## name(T const x)                                           \
  {                                                                    \
    switch(x)                                                          \
    {                                                                  \
      BOOST_PP_SEQ_FOR_EACH(                                           \
        DEFINE_VALUE_TO_ENUM_CASES_INTERNAL,                           \
        prefix,                                                        \
        enumerators                                                    \
      )                                                                \
      default: return BOOST_PP_CAT(prefix, default_value);             \
    }                                                                  \
  }

/*!
 * @brief Define enum type with string and value conversion functions.
 * @param name        The typename of the enum. This will also be concatenated
 *                    with "To" to define a function that converts integral
 *                    values to this enum type. For example. the name "Foo" will
 *                    define "Foo toFoo(T x)".
 * @param prefix      The prefix to attach to all values of the enum. This may
 *                    be an empty string.
 * @param enumerators A sequence of name-value pairs in double-parentheses, e.g.
 *                    ((a, 1))((b, 2))((c , 4)). All valid enum values may be
 *                    used, e.g. ((a, (1<<0)))((b, (1<<1))), etc.
 */
#define DEFINE_ENUM(name, prefix, enumerators, default_value)    \
  DEFINE_ENUM_VALUES(name, prefix, enumerators)                  \
  DEFINE_ENUM_TO_STRING(name, prefix, enumerators)               \
  DEFINE_VALUE_TO_ENUM(name, prefix, enumerators, default_value)





//------------------------------------------------------------------------------
// Collect common enums here.
//------------------------------------------------------------------------------
// Use these when unifying the frame logic.

//------------------------------------------------------------------------------
//! @brief Horizontal direction of Lidar.
DEFINE_ENUM(
  HorizontalDirection,
  HDIR_,
  ((CLOCKWISE         , 0))
  ((COUNTER_CLOCKWISE , 1)),
  CLOCKWISE
)

//------------------------------------------------------------------------------
//! @brief Vertical direction of Lidar.
DEFINE_ENUM(
  VerticalDirection,
  VDIR_,
  ((UP   , 0))
  ((DOWN , 1)),
  UP
)


//------------------------------------------------------------------------------
/*!
 * @brief The framing logic.
 *
 * The legacy and advanced packet formats (lfp and afp, resp.) use the same
 * calibration files but afp introduced the notion of using vdir changes to
 * delineate frames even in lidars that do not support vertical scanning. The
 * default behavior should be that afp packets are split into frames when the
 * vdir changes but the lfp are split using the legacy logic. The configuration
 * files should have an option to toggle this behavior. An enum is used here
 * both to allow for other framing logic in the future and to provide a default
 * value so that each interpreter may correctly fall back to the expected
 * behavior when the configuration file does not specify the logic.
 */
DEFINE_ENUM(
  FramingLogic,
  FL_,
  ((DEFAULT          , 0))
  ((AZIMUTH_CROSSING , 1))
  ((VDIR_CHANGE      , 2)),
  DEFAULT
)


//-----------------------------------------------------------------------------
// Raw input values required by ComputeCorrectedValues.
struct RawValues
{
  //! @brief The azimuthal angle, in centidegrees.
  unsigned short azimuth;
  //! @brief The altitudinal angle, in centidegrees.
  unsigned short elevation;
  //! @brief The distance, in raw units.
  uint32_t distance;
  //! @brief The intensity, in raw units.
  uint32_t intensity;

  RawValues(
    decltype(azimuth) azm,
    decltype(elevation) elev,
    decltype(distance) dist,
    decltype(intensity) inten = 0
  )
    : azimuth { azm }
    , elevation { elev }
    , distance { dist }
    , intensity { inten }
  {
  }
};

//-----------------------------------------------------------------------------
// ComputeCorrectedValues needs to calculate multiple corrected values which
// may or may not be used by different interpreters. Collect these in a common
// struct so that the interpreter can easily ignore unused values without
// creating multiple dummy variables just to invoke the function.
struct CorrectedValues
{
  double position[3];
  double distance;
  short intensity;
  double elevation;
};

//------------------------------------------------------------------------------
//VelodyneCalibrationData
//------------------------------------------------------------------------------
class VTK_EXPORT VelodyneCalibrationData
{
private:
  vtkTable * CalibrationData { nullptr };

  bool IsCalibrated { false };
  bool IsCorrectionFromLiveStream { false };
  FramingLogic FrameLogic  { FramingLogic::FL_DEFAULT };
  int CalibrationReportedNumLasers { -1 };
  double DistanceResolutionM;

  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];

  double XMLColorTable[HDL_MAX_NUM_LASERS][3];

public:
	void InitTrigonometricTables();
  void PrecomputeCorrectionCosSin();
  void LoadCalibration(std::string const & filename);
  VelodyneCalibrationData(vtkTable * calibDat) : CalibrationData { calibDat } {};

  /*!
   * @brief      Compute corrected position and other values using the Velodyne
   *             calibration file.
   * @param[in]  rawValues        The raw input values.
   * @param[in]  correctionIndex  The index of the laser to be retrieved from
   *                              the calibration file.
   * @param[out] correctedValues  The computed corrected values.
   * @param[in]  correctIntensity If true, correct the intensity.
   */
  void ComputeCorrectedValues(
    const RawValues & rawValues,
    const unsigned int correctionIndex,
    CorrectedValues & correctedValues,
    bool correctIntensity = false
  );

  void GetXMLColorTable(double XMLColorTable[4 * HDL_MAX_NUM_LASERS]);

  FramingLogic GetFramingLogic() { return this->FrameLogic; }
  bool GetIsCalibrated() { return this->IsCalibrated; }
  bool GetIsCorrectionFromLiveStream() { return this->IsCorrectionFromLiveStream; }
  int GetCalibrationReportedNumLasers() { return this->CalibrationReportedNumLasers; }
  double GetDistanceResolutionM() { return this->DistanceResolutionM; }
  // TODO
  // This is a temporary workaround to avoid breaking code in the legacy
  // interpreter that updates the corrections. All of those updates should be
  // moved into this class and this function should then be removed.
  HDLLaserCorrection * GetLaserCorrections() { return this->laser_corrections_; }
};

#endif // VTKVELODYNEPACKETINTERPRETER_H

