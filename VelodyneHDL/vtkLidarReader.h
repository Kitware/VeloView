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

  /**
   * @copydoc vtkLidarReaderInternal::FileName
   */
  std::string GetFileName();
  virtual void SetFileName(const std::string& filename);

  int GetNumberOfFrames() override;

  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber, int wantedNumberOfTrailingFrames) override;

  /**
   * @copydoc vtkLidarReaderInternal::Open()
   * @todo should be move to Internal eventually
   */
  void Open();

  /**
   * @copydoc vtkLidarReaderInternal::Close()
   * @todo should be move to Internal eventually
   */
  void Close();

  /**
   * @copydoc vtkLidarReaderInternal::ProcessPacket()
   */
  void ProcessPacket(unsigned char* data, unsigned int bytesReceived);

  /**
   * @brief SaveFrame save the packet corresponding to the desired frames in a pcap file.
   * Because we are saving network packet, part of previous and/or next frames could be included in generated the pcap
   * @param startFrame first frame to record
   * @param endFrame last frame to record, this frame is included
   * @param filename where to save the generate pcap file
   */
  void SaveFrame(int startFrame, int endFrame, const std::string& filename);

protected:
  vtkLidarReader();
  vtkLidarReader(vtkLidarReaderInternal* internal);

  /**
   * @brief SetPimpInternal method used to switch the opaque pointer
   */
  void SetPimpInternal(vtkLidarReaderInternal* internal);

  // Description:
  // This is called by the superclass.
  // This is the method you should override.
  int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*);


private:
  vtkLidarReader(const vtkLidarReader&); // not implemented
  void operator=(const vtkLidarReader&); // not implemented

  vtkLidarReaderInternal* Internal;
};

#endif // VTKLIDARREADER_H
