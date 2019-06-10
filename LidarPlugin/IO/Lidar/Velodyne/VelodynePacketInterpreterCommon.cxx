#include "VelodynePacketInterpreterCommon.h"

#include <vtkMath.h>
#include <vtkDoubleArray.h>

//------------------------------------------------------------------------------
// Convenience functions.
//------------------------------------------------------------------------------
/*!
 * @brief     Convert degrees to radians.
 * @param[in] degrees The input value in degrees.
 * @return    The input value converted to radians.
 */
inline double
degreesToRadians(double degrees)
{
  return (degrees * vtkMath::Pi()) / 180.0;
}

//------------------------------------------------------------------------------
// Code from the legacy packet format interpreter.
//------------------------------------------------------------------------------
void
VelodyneCalibrationData::InitTrigonometricTables()
{
  if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
  {
    cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
    {
      double rad           = degreesToRadians(i / 100.0);
      cos_lookup_table_[i] = std::cos(rad);
      sin_lookup_table_[i] = std::sin(rad);
    }
  }
}

//------------------------------------------------------------------------------
void
VelodyneCalibrationData::PrecomputeCorrectionCosSin()
{
  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    HDLLaserCorrection & correction = laser_corrections_[i];
    correction.cosVertCorrection =
      std::cos(degreesToRadians(correction.verticalCorrection));
    correction.sinVertCorrection =
      std::sin(degreesToRadians(correction.verticalCorrection));
    correction.cosRotationalCorrection =
      std::cos(degreesToRadians(correction.rotationalCorrection));
    correction.sinRotationalCorrection =
      std::sin(degreesToRadians(correction.rotationalCorrection));
    correction.sinVertOffsetCorrection =
      correction.verticalOffsetCorrection * correction.sinVertCorrection;
    correction.cosVertOffsetCorrection =
      correction.verticalOffsetCorrection * correction.cosVertCorrection;
  }
}

//-----------------------------------------------------------------------------
void VelodyneCalibrationData::LoadCalibration(const std::string& filename)
{
  // the HDL64 allow autocalibration, so no calibration can be provided
  if (filename.empty())
  {
    this->IsCalibrated = false;
    this->IsCorrectionFromLiveStream = true;
    return;
  }
  else
  {
    this->IsCorrectionFromLiveStream = false;
  }

  boost::property_tree::ptree pt;
  try
  {
    read_xml(filename, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (boost::exception const&)
  {
    vtkGenericWarningMacro(
      "LoadCalibration: error reading calibration file: " << filename);
    return;
  }
  // Read distLSB if provided
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v, pt.get_child("boost_serialization.DB"))
  {
    if (v.first == "distLSB_")
    { // Stored in cm in xml
      DistanceResolutionM = atof(v.second.data().c_str()) / 100.0;
    }
  }

  int i, j;
  i = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type& p, pt.get_child("boost_serialization.DB.colors_"))
  {
    if (p.first == "item")
    {
      j = 0;
      BOOST_FOREACH (boost::property_tree::ptree::value_type& v, p.second.get_child("rgb"))
        if (v.first == "item")
        {
          std::stringstream ss;
          double val;
          ss << v.second.data();
          ss >> val;

          XMLColorTable[i][j] = val;
          j++;
        }
      i++;
    }
  }

  int enabledCount = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type& v, pt.get_child("boost_serialization.DB.enabled_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      int test = 0;
      ss >> test;
      if (!ss.fail() && test == 1)
      {
        enabledCount++;
      }
    }
  }
  this->CalibrationReportedNumLasers = enabledCount;

  // Getting min & max intensities from XML
  int laserId = 0;
  int minIntensity[HDL_MAX_NUM_LASERS], maxIntensity[HDL_MAX_NUM_LASERS];
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
    pt.get_child("boost_serialization.DB.minIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      ss >> minIntensity[laserId];
      laserId++;
    }
  }

  laserId = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
    pt.get_child("boost_serialization.DB.maxIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      ss >> maxIntensity[laserId];
      laserId++;
    }
  }

  BOOST_FOREACH (
    boost::property_tree::ptree::value_type& v, pt.get_child("boost_serialization.DB.points_"))
  {
    if (v.first == "item")
    {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH (boost::property_tree::ptree::value_type& px, points)
      {
        if (px.first == "px")
        {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          HDLLaserCorrection xmlData;

          BOOST_FOREACH (boost::property_tree::ptree::value_type& item, calibrationData)
          {
            if (item.first == "id_")
              index = atoi(item.second.data().c_str());
            if (item.first == "rotCorrection_")
              xmlData.rotationalCorrection = atof(item.second.data().c_str());
            if (item.first == "vertCorrection_")
              xmlData.verticalCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrection_")
              xmlData.distanceCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrectionX_")
              xmlData.distanceCorrectionX = atof(item.second.data().c_str());
            if (item.first == "distCorrectionY_")
              xmlData.distanceCorrectionY = atof(item.second.data().c_str());
            if (item.first == "vertOffsetCorrection_")
              xmlData.verticalOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "horizOffsetCorrection_")
              xmlData.horizontalOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "focalDistance_")
              xmlData.focalDistance = atof(item.second.data().c_str());
            if (item.first == "focalSlope_")
              xmlData.focalSlope = atof(item.second.data().c_str());
            if (item.first == "closeSlope_")
              xmlData.closeSlope = atof(item.second.data().c_str());
          }
          if (index != -1 && index < HDL_MAX_NUM_LASERS)
          {
            laser_corrections_[index] = xmlData;
            // Angles are already stored in degrees in xml
            // Distances are stored in centimeters in xml, and we store meters.
            laser_corrections_[index].distanceCorrection /= 100.0;
            laser_corrections_[index].distanceCorrectionX /= 100.0;
            laser_corrections_[index].distanceCorrectionY /= 100.0;
            laser_corrections_[index].verticalOffsetCorrection /= 100.0;
            laser_corrections_[index].horizontalOffsetCorrection /= 100.0;
            laser_corrections_[index].focalDistance /= 100.0;
            laser_corrections_[index].focalSlope /= 100.0;
            laser_corrections_[index].closeSlope /= 100.0;
            if (laser_corrections_[index].closeSlope == 0.0)
              laser_corrections_[index].closeSlope = laser_corrections_[index].focalSlope;
            laser_corrections_[index].minIntensity = minIntensity[index];
            laser_corrections_[index].maxIntensity = maxIntensity[index];
          }
        }
      }
    }
  }

  int idx = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
    pt.get_child("boost_serialization.DB.minIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      int intensity = 0;
      ss >> intensity;
      if (!ss.fail() && idx < HDL_MAX_NUM_LASERS)
      {
        laser_corrections_[idx].minIntensity = intensity;
      }
      idx++;
    }
  }

  idx = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
    pt.get_child("boost_serialization.DB.maxIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      int intensity = 0;
      ss >> intensity;
      if (!ss.fail() && idx < HDL_MAX_NUM_LASERS)
      {
        laser_corrections_[idx].maxIntensity = intensity;
      }
      idx++;
    }
  }

  PrecomputeCorrectionCosSin();
  this->IsCalibrated = true;
  this->CalibrationData->Initialize();
//  // Copy the calibration into a vtkTable
  #define AddToCalibrationDataRowNamed(name, field)                                     \
  auto array##field = vtkSmartPointer<vtkDoubleArray>::New();                           \
  array##field->SetName(name);                                                          \
  for (int i = 0; i < this->CalibrationReportedNumLasers; i++)                          \
  {                                                                                     \
    array##field->InsertNextTuple1(this->laser_corrections_[i].field);                  \
  }                                                                                     \
  this->CalibrationData->AddColumn(array##field);

  AddToCalibrationDataRowNamed("rotationalCorrection",      rotationalCorrection)
  AddToCalibrationDataRowNamed("verticalCorrection",        verticalCorrection)
  AddToCalibrationDataRowNamed("distanceCorrection",        distanceCorrection)
  AddToCalibrationDataRowNamed("distanceCorrectionX",       distanceCorrectionX)
  AddToCalibrationDataRowNamed("distanceCorrectionY",       distanceCorrectionY)
  AddToCalibrationDataRowNamed("verticalOffsetCorrection",  verticalOffsetCorrection)
  AddToCalibrationDataRowNamed("horizontalOffsetCorrection",horizontalOffsetCorrection)
  AddToCalibrationDataRowNamed("focalDistance",             focalDistance)
  AddToCalibrationDataRowNamed("focalSlope",                focalSlope)
  AddToCalibrationDataRowNamed("closeSlope",                closeSlope)
  AddToCalibrationDataRowNamed("minIntensity",              minIntensity)
  AddToCalibrationDataRowNamed("maxIntensity",              maxIntensity)
  AddToCalibrationDataRowNamed("sinRotationalCorrection",   sinRotationalCorrection)
  AddToCalibrationDataRowNamed("cosRotationalCorrection",   cosRotationalCorrection)
  AddToCalibrationDataRowNamed("sinVertCorrection",         sinVertCorrection)
  AddToCalibrationDataRowNamed("cosVertCorrection",         cosVertCorrection)
  AddToCalibrationDataRowNamed("sinVertOffsetCorrection",   sinVertOffsetCorrection)
  AddToCalibrationDataRowNamed("cosVertOffsetCorrection",   cosVertOffsetCorrection)
}

//-----------------------------------------------------------------------------
void VelodyneCalibrationData::ComputeCorrectedValues(
    const RawValues & rawValues,
    const unsigned int correctionIndex,
    CorrectedValues & correctedValues,
    bool correctIntensity
  )
{
  HDLLaserCorrection * correction = &(this->laser_corrections_[correctionIndex]);

  correctedValues.intensity = rawValues.intensity;
  correctedValues.elevation = static_cast<double>(rawValues.elevation) * 0.01 + correction->verticalCorrection;

  double cosAzimuth, sinAzimuth;
  if (correction->rotationalCorrection == 0)
  {
    cosAzimuth = this->cos_lookup_table_[rawValues.azimuth];
    sinAzimuth = this->sin_lookup_table_[rawValues.azimuth];
  }
  else
  {
    // realAzimuth = rawValues.azimuth/100 - rotationalCorrection
    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    cosAzimuth = this->cos_lookup_table_[rawValues.azimuth] * correction->cosRotationalCorrection +
      this->sin_lookup_table_[rawValues.azimuth] * correction->sinRotationalCorrection;
    sinAzimuth = this->sin_lookup_table_[rawValues.azimuth] * correction->cosRotationalCorrection -
      this->cos_lookup_table_[rawValues.azimuth] * correction->sinRotationalCorrection;
  }

  double cosVertCorrection = correction->cosVertCorrection;
  double sinVertCorrection = correction->sinVertCorrection;
  double sinVertOffsetCorrection = correction->sinVertOffsetCorrection;
  if (rawValues.elevation != 0)
  { /*
     if (rawValues.elevation < this->sin_lookup_table_1000_.size())
     {
       cosVertCorrection = correction->cosVertCorrection * this->cos_lookup_table_1000_[rawValues.elevation] -
         correction->sinVertCorrection * this->sin_lookup_table_1000_[rawValues.elevation];
       sinVertCorrection = correction->sinVertCorrection * this->cos_lookup_table_1000_[rawValues.elevation] +
         correction->cosVertCorrection * this->sin_lookup_table_1000_[rawValues.elevation];
     }
     else*/
    {
      double vertAngleRad =
        vtkMath::Pi() / 180.0 * (correction->verticalCorrection + static_cast<double>(rawValues.elevation) / 100.0);
      cosVertCorrection = std::cos(vertAngleRad);
      sinVertCorrection = std::sin(vertAngleRad);
    }
    sinVertOffsetCorrection = correction->verticalOffsetCorrection * sinVertCorrection;
  }

  // Compute the distance in the xy plane (w/o accounting for rotation)
  /**the new term of sinVertOffsetCorrection
   * was added to the expression due to the mathemathical
   * model we used.(c
   */
  double distanceMRaw = rawValues.distance * this->DistanceResolutionM;
  double distanceM = distanceMRaw + correction->distanceCorrection;
  double xyDistance = distanceM * cosVertCorrection - sinVertOffsetCorrection;

  correctedValues.distance = distanceM;

  correctedValues.position[0] = xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth;
  correctedValues.position[1] = xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth;
  correctedValues.position[2] = distanceM * sinVertCorrection + correction->verticalOffsetCorrection;

  if (correctIntensity && (correction->minIntensity < correction->maxIntensity))
  {
    // Compute corrected intensity

    /* Please refer to the manual:
      "Velodyne, Inc. ©2013  63‐HDL64ES3 REV G" Appendix F. Pages 45-46
      PLease note: in the manual, focalDistance is in centimeters, distance is the raw short from
      the laser
      & the graph is in meter */

    // Casting the input values to double for the computation

    double computedIntensity = static_cast<double>(correctedValues.intensity);
    double minIntensity = static_cast<double>(correction->minIntensity);
    double maxIntensity = static_cast<double>(correction->maxIntensity);

    // Rescale the intensity between 0 and 255
    computedIntensity = (computedIntensity - minIntensity) / (maxIntensity - minIntensity) * 255.0;

    if (computedIntensity < 0)
    {
      computedIntensity = 0;
    }

    double focalOffset = 256 * pow(1.0 - correction->focalDistance / 131.0, 2);
    double insideAbsValue = std::abs(
      focalOffset - 256 * pow(1.0 - static_cast<double>(rawValues.distance) / 65535.0f, 2));

    if (insideAbsValue > 0)
    {
      computedIntensity = computedIntensity + correction->focalSlope * insideAbsValue;
    }
    else
    {
      computedIntensity = computedIntensity + correction->closeSlope * insideAbsValue;
    }
    computedIntensity = std::max(std::min(computedIntensity, 255.0), 1.0);

     correctedValues.intensity = static_cast<decltype(correctedValues.intensity)>(computedIntensity);
  }
}

//-----------------------------------------------------------------------------
void VelodyneCalibrationData::GetXMLColorTable(double XMLColorTable[4 * HDL_MAX_NUM_LASERS])
{
  for (int i = 0; i < HDL_MAX_NUM_LASERS; ++i)
  {
    XMLColorTable[i * 4] = static_cast<double>(i) / 63.0 * 255.0;
    for (int j = 0; j < 3; ++j)
    {
      XMLColorTable[i * 4 + j + 1] = this->XMLColorTable[i][j];
    }
  }
}

