#ifndef VTKLIDARSOURCE_H
#define VTKLIDARSOURCE_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

class vtkLidarSourceInternal;

class vtkLidarSource : public vtkPolyDataAlgorithm
{
public:
  static vtkLidarSource* New();
  vtkTypeMacro(vtkLidarSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);


//  const std::string& GetFileName();
//  void SetFileName(const std::string& filename);

  // Description:
  // Number of frames behind current frame to read.  Zero indicates only
  // show the current frame.  Negative numbers are invalid.
  virtual void SetNumberOfTrailingFrames(int numberTrailing);

//  void SetLaserSelection(int LaserSelection[HDL_MAX_NUM_LASERS]);
//  void GetLaserSelection(int LaserSelection[HDL_MAX_NUM_LASERS]);

  // Croping
  void SetCropMode(int);
  void SetCropReturns(int);
  void SetCropOutside(int);
  void SetCropRegion(double[6]);
  void SetCropRegion(double, double, double, double, double, double);


//  void GetLaserCorrections(double verticalCorrection[HDL_MAX_NUM_LASERS],
//    double rotationalCorrection[HDL_MAX_NUM_LASERS], double distanceCorrection[HDL_MAX_NUM_LASERS],
//    double distanceCorrectionX[HDL_MAX_NUM_LASERS], double distanceCorrectionY[HDL_MAX_NUM_LASERS],
//    double verticalOffsetCorrection[HDL_MAX_NUM_LASERS],
//    double horizontalOffsetCorrection[HDL_MAX_NUM_LASERS], double focalDistance[HDL_MAX_NUM_LASERS],
//    double focalSlope[HDL_MAX_NUM_LASERS], double minIntensity[HDL_MAX_NUM_LASERS],
//    double maxIntensity[HDL_MAX_NUM_LASERS]);

//  int GetOutputPacketProcessingDebugInfo() const;
//  void SetOutputPacketProcessingDebugInfo(int);

//  int GetIgnoreZeroDistances() const;
//  void SetIgnoreZeroDistances(int);

//  int GetIgnoreEmptyFrames() const;
//  void SetIgnoreEmptyFrames(int);

//  int GetIntraFiringAdjust() const;
//  void SetIntraFiringAdjust(int);

//  unsigned int GetDualReturnFilter() const;
//  void SetDualReturnFilter(unsigned int);
protected:
  vtkLidarSource();
  vtkLidarSource(vtkLidarSourceInternal* internal);
  virtual ~vtkLidarSource();

  void SetPimpInternal(vtkLidarSourceInternal* internal) {this->Internal = internal;};

  // Description:
  // This is called by the superclass.
  // This is the method you should override.
  virtual int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);


private:
  vtkLidarSource(const vtkLidarSource&);
  void operator=(const vtkLidarSource&);

  vtkLidarSourceInternal* Internal;
};

#endif // VTKLIDARSOURCE_H
