#ifndef VTKLIDARREADER_H
#define VTKLIDARREADER_H

//#include "vtkLidarSource.h"
#include "vtkLidarProvider.h"

class vtkLidarReaderInternal;
class vtkTransform;

class VTK_EXPORT vtkLidarReader : public vtkLidarProvider
{
public:
  vtkTypeMacro(vtkLidarReader, vtkLidarProvider);
  void PrintSelf(ostream& os, vtkIndent indent);

  std::string GetFileName();
  virtual void SetFileName(const std::string& filename);

  int GetNumberOfFrames() override;

  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrames) override;
  vtkPolyData* GetFramePointer(int frameNumber, int wantedNumberOfTrailingFrames) override;

  // TODO Should be private!!
  void Open();
  void Close();

  void ProcessPacket(unsigned char* data, unsigned int bytesReceived);

  ///
  /// \brief SaveFrame save the packet corresponding to the desired frame in a pcap file.
  /// Because we are saving network packet, part of previous and/or next frames could be included in generated the pcap
  /// \param startFrame
  /// \param endFrame
  /// \param filename including the file extension
  ///
  void SaveFrame(int startFrame, int endFrame, const std::string& filename);

protected:
  vtkLidarReader();
  vtkLidarReader(vtkLidarReaderInternal* internal);

  void SetPimpInternal(vtkLidarReaderInternal* internal);

  // Description:
  // This is called by the superclass.
  // This is the method you should override.
  int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*);


private:
  vtkLidarReader(const vtkLidarReader&);
  void operator=(const vtkLidarReader&);

  void SetTimestepInformation(vtkInformation* info);

  vtkLidarReaderInternal* Internal;
};

#endif // VTKLIDARREADER_H
