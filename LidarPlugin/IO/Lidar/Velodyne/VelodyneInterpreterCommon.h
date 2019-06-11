#ifndef VELODYNEENUMMACROS_H
#define VELODYNEENUMMACROS_H

#include "EnumMacros.h"



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

#endif // VELODYNEENUMMACROS_H

