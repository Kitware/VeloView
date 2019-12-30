#ifndef VTK_VELODYNE_ADVANCED_PACKET_FRAMING_H
#define VTK_VELODYNE_ADVANCED_PACKET_FRAMING_H

#include "VelodyneInterpreterCommon.h"
#include <cstdint>

//------------------------------------------------------------------------------
// FrameTracker
//------------------------------------------------------------------------------
/*!
 * @brief Object to track information related to frame changes. All frame change
 *        logic should be kept here.
 *
 * The current logic was taken from the VelArray branch.
 */
class FrameTracker
{
private:
  FramingLogic FrameLogic { FramingLogic::FL_DEFAULT };
  bool HasLastState;
  HorizontalDirection LastHorDir;
  VerticalDirection LastVertDir;
  int LastSlope;
  uint16_t LastAzimuth;
  uint16_t LastVertDeflection;

public:
  FrameTracker();

  void SetFramingLogic(FramingLogic fl);

  //! @brief Reset frame detection.
  void Reset();

  /*!
   * @brief     Update the frame tracker.
   * @return    True if a new frame is detected, false otherwise.
   */
  bool
  Update(uint16_t azimuth,
         uint16_t vertDeflection,
         HorizontalDirection horDir,
         VerticalDirection vertDir);
};

#endif // VTK_VELODYNE_ADVANCED_PACKET_FRAMING_H
