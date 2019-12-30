#include "vtkVelodyneAdvancedPacketFraming.h"

#include <vtkObjectFactory.h>

FrameTracker::FrameTracker()
{
  this->Reset();
}

void FrameTracker::SetFramingLogic(FramingLogic fl)
{
  this->FrameLogic = fl;
}

void FrameTracker::Reset()
{
  this->HasLastState = false;
  this->LastSlope      = 0;
  this->LastAzimuth    = static_cast<decltype(this->LastAzimuth)>(-1);
}

bool
FrameTracker::Update(uint16_t azimuth,
                     uint16_t vertDeflection,
                     HorizontalDirection horDir,
                     VerticalDirection vertDir)
{
  // Get and update all member values here to avoid doing so in various blocks
  // below just before returning.

  decltype(this->LastAzimuth) lastAzimuth = this->LastAzimuth;

  // Follow the logic of the legacy interpreter.
  int slope = static_cast<int>(azimuth) - static_cast<int>(lastAzimuth),
      lastSlope = this->LastSlope;

  VerticalDirection lastVertDir = this->LastVertDir;

  bool hasLastState = this->HasLastState;

  this->LastAzimuth = azimuth; // static_cast<decltype(this->LastAzimuth)>(-1);
  this->LastVertDeflection = vertDeflection;
  // this->LastSlope   = slope;
  this->LastHorDir = horDir;
  this->LastVertDir = vertDir;
  this->HasLastState = true;

  if (! hasLastState)
  {
    return false;
  }

  if (this->FrameLogic != FramingLogic::FL_AZIMUTH_CROSSING)
  {
    return (vertDir != lastVertDir);
  }

  // // VelArray
  // ModelIdentificationCode mic = payloadHeader->GetMic();
  // if (mic == ModelIdentificationCode::MIC_VELARRAY || firingGroupHeader->GetVdfl() != 0)
  // {
  //   // The frame split if either the vertical direction changes (vertical
  //   // scanning) OR the horizontal direction changes without a change in the
  //   // vertical deflection (pure horizontal scanning).
  //   return (
  //       (vertDir != lastVertDir) ||
  //       (horDir != lastHorDir && vertDeflection == lastVertDeflection)
  //     );
  // }

  // Not VelArray
  else
  {
    // // New frame when the azimuth rolls over.
    // if (azimuth == 0 and lastAzimuth != 0)
    // {
    //   return true;
    // }
    // else
    // {
    //   return this->HasLastHorDir ? (horDir != lastHorDir) : false;
    // }

    // Old logic from VelArray FramingState. This doesn't seem to work for
    // other sensors.
    if (slope == 0)
    {
      return false;
    }

    int isSameSlopeDir = slope * lastSlope;
    if (isSameSlopeDir > 0)
    {
      return false;
    }

    else if (isSameSlopeDir < 0)
    {
      this->LastSlope = 0;
      return true;
    }

    if (this->LastSlope == 0 && slope != 0)
    {
      this->LastSlope = slope;
      return false;
    }
    vtkGenericWarningMacro("Unhandled sequence in framing state.");
    return false;
  }

  return false;
}
