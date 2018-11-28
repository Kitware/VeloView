#ifndef VTKLIDARKITTIDATASETREADER_H
#define VTKLIDARKITTIDATASETREADER_H

#include "vtkLidarReader.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#ifndef _WIN32
#define notImpementedBody \
std::cerr << typeid(this).name() << "::" << __func__ << " is not implemented" << std::endl;
#else
#define notImpementedBody \
std::cerr << typeid(this).name() << "::" << __FUNCTION__ << " is not implemented" << std::endl;
#endif

/**
 * @brief The vtkLidarKITTIDataSetReader class
 * @warning Only a small subset of method are implemented
 */
class VTK_EXPORT vtkLidarKITTIDataSetReader : public vtkLidarReader
{
public:
  static vtkLidarKITTIDataSetReader* New();
  vtkTypeMacro(vtkLidarKITTIDataSetReader, vtkLidarReader)

  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber) override;

  vtkGetMacro(FileName, std::string)
  void SetFileName(const std::string& filename) override;

  vtkGetMacro(NumberOfFrames, int)

  //! Not implemented
  void SetCalibrationFileName(const std::string& vtkNotUsed(filename)) override {notImpementedBody}

  //! Not implemented
  void Open() override {notImpementedBody}

  //! Not implemented
  void Close() override {notImpementedBody}

  //! Not implemented
  void SaveFrame(int vtkNotUsed(startFrame), int vtkNotUsed(endFrame),
                 const std::string& vtkNotUsed(filename)) override {notImpementedBody}

  //! Not implemented
  std::string GetSensorInformation();

  // return the number of channels
  vtkGetMacro(NbrLaser, int)

private:
  vtkLidarKITTIDataSetReader() = default;

  int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector) override;

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  //! folder containing all the .bin file for a sequence
  //! this should be named FolderName but to keep the same API we will keep FileName
  std::string FileName = "";

  //! Number of previous frames to display with the current frame (concatenation of frames)
  int NumberOfTrailingFrames;

  //! Number of frame in this sequence
  int NumberOfFrames = 0;

  int NbrLaser = 64;

  vtkLidarKITTIDataSetReader(const vtkLidarKITTIDataSetReader&) = delete;
  void operator=(const vtkLidarKITTIDataSetReader&) = delete;
};

#endif // VTKLIDARKITTIDATASETREADER_H
