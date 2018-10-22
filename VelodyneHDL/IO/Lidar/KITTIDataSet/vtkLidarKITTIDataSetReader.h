#ifndef VTKLIDARKITTIDATASETREADER_H
#define VTKLIDARKITTIDATASETREADER_H

#include "vtkLidarReader.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#define notImpementedBody \
std::cerr << typeid(this).name() << "::" << __func__ << " is not implemented" << std::endl;

//
// Set built-in type.  Creates member Set"name"() (e.g., SetVisibility());
//
#define notImplementedSetMacro(name_,type) \
void Set##name_ (const type _arg) override\
  { \
    notImpementedBody \
  }

//
// Get built-in type.  Creates member Get"name"() (e.g., GetVisibility());
//
#define notImplementedGetMacro(name_,type) \
type Get##name_ () override\
  { \
    notImpementedBody \
    return type(); \
  }


/**
 * @brief The vtkLidarKITTIDataSetReader class
 * @warning Only a small subset of method are implemented
 */
class VTK_EXPORT vtkLidarKITTIDataSetReader : public vtkLidarReader
{
public:
  static vtkLidarKITTIDataSetReader* New();
  vtkTypeMacro(vtkLidarKITTIDataSetReader, vtkLidarReader)
  void PrintSelf(ostream& os, vtkIndent indent);

  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrames = 0) override;

  vtkGetMacro(FileName, std::string);
  void SetFileName(const std::string& filename) override;

  int GetNumberOfTrailingFrames() override { return this->NumberOfFrames; }
  void SetNumberOfTrailingFrames(const int nbTrailingFrames) override;

  int GetNumberOfFrames() override;

  //! Not implemented
  notImplementedGetMacro(CalibrationFileName, std::string)
  void SetCalibrationFileName(const std::string& filename) override {notImpementedBody}

  //! Not implemented
  notImplementedGetMacro(IsCalibrated, bool)

  //! Not implemented
  notImplementedGetMacro(NumberOfChannels, int)

  //! Not implemented
  notImplementedGetMacro(Frequency, double)

  //! Not implemented
  notImplementedGetMacro(DistanceResolutionM, double)

  //! Not implemented
  notImplementedGetMacro(IgnoreZeroDistances, bool)
  notImplementedSetMacro(IgnoreZeroDistances, bool)

  //! Not implemented
  notImplementedGetMacro(IgnoreEmptyFrames, bool)
  notImplementedSetMacro(IgnoreEmptyFrames, bool)

  //! Not implemented
  notImplementedSetMacro(CropMode, int)

  //! Not implemented
  notImplementedGetMacro(CropReturns, bool)
  notImplementedSetMacro(CropReturns, bool)

  //! Not implemented
  notImplementedGetMacro(CropOutside, bool)
  notImplementedSetMacro(CropOutside, bool)

  //! Not implemented
  notImplementedGetMacro(ApplyTransform, bool)
  notImplementedSetMacro(ApplyTransform, bool)

  //! Not implemented
  void SetLaserSelection(bool laserSelection[]) override {notImpementedBody}
  void GetLaserSelection(bool laserSelection[]) override {notImpementedBody}

  //! Not implemented
  void SetCropRegion(double region[6]) override {notImpementedBody}
  void SetCropRegion(const double v0, const double v1,
                     const double v2, const double v3,
                     const double v4, const double v5) override {notImpementedBody}

  //! Not implemented
  vtkVelodyneTransformInterpolator* GetInterpolator() const override {notImpementedBody return nullptr;}
  void SetInterpolator(vtkVelodyneTransformInterpolator* interpolator) override {notImpementedBody}

  //! Not implemented
  void Open() override {notImpementedBody}

  //! Not implemented
  void Close() override {notImpementedBody}

  //! Not implemented
  void SaveFrame(int startFrame, int endFrame, const std::string& filename) override {notImpementedBody}

  //! Not implemented
  std::string GetSensorInformation() override {notImpementedBody return "";}

private:
  vtkLidarKITTIDataSetReader();

  int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*);

  //! folder containing all the .bin file for a sequence
  //! this should be named FolderName but to keep the same API we will keep FileName
  std::string FileName;

  //! Number of previous frames to display with the current frame (concatenation of frames)
  int NumberOfTrailingFrames;

  //! Number of frame in this sequence
  int NumberOfFrames;


  vtkLidarKITTIDataSetReader(const vtkLidarKITTIDataSetReader&); // not implemented
  void operator=(const vtkLidarKITTIDataSetReader&); // not implemented
};

#endif // VTKLIDARKITTIDATASETREADER_H
