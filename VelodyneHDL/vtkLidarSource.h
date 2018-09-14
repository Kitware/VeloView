#ifndef VTKLIDARSOURCE_H
#define VTKLIDARSOURCE_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

class vtkLidarSourceInternal;
class vtkTransform;

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
  void SetNumberOfTrailingFrames(int numberTrailing);

  // vtk doesn't handle virtual pure function
  void ProcessPacket(unsigned char* data, unsigned int bytesReceived);

  virtual std::string GetSensorInformation() {};


  double GetCurrentRpm();
  int GetNumberOfFrames();

  // Laser
  int GetNumberOfChannels();
  void SetLaserSelection(bool LaserSelection[]);
  void GetLaserSelection(bool LaserSelection[]);


  // Croping
  void SetCropMode(int);
  void SetCropReturns(int);
  void SetCropOutside(int);
  void SetCropRegion(double[6]);
  void SetCropRegion(double, double, double, double, double, double);

  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrame = 0);

  bool getCorrectionsInitialized();

  void SetSensorTransform(vtkTransform*);
  int GetApplyTransform();
  void SetApplyTransform(int apply);


//  int GetOutputPacketProcessingDebugInfo() const;
//  void SetOutputPacketProcessingDebugInfo(int);

  int GetIgnoreZeroDistances() const;
  void SetIgnoreZeroDistances(int);

  int GetIgnoreEmptyFrames() const;
  void SetIgnoreEmptyFrames(int);

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
