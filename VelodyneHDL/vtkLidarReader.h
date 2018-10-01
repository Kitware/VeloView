#ifndef VTKLIDARREADER_H
#define VTKLIDARREADER_H

#include "vtkLidarProvider.h"

//! @todo a decition should be made if the opening/closing of the pcap should be handle by
//! the class itself of the class user. Currently this is not clear

class vtkLidarReaderInternal;
class LidarPacketInterpretor;

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
   * @todo a decition should be made if the opening/closing of the pcap should be handle by
   * the class itself of the class user. Currently this is not clear
   */
  void Open();

  /**
   * @copydoc vtkLidarReaderInternal::Close()
   * @todo a decition should be made if the opening/closing of the pcap should be handle by
   * the class itself of the class user. Currently this is not clear
   */
  void Close();

  /**
   * @brief SaveFrame save the packet corresponding to the desired frames in a pcap file.
   * Because we are saving network packet, part of previous and/or next frames could be included in generated the pcap
   * @param startFrame first frame to record
   * @param endFrame last frame to record, this frame is included
   * @param filename where to save the generate pcap file
   */
  void SaveFrame(int startFrame, int endFrame, const std::string& filename);

  friend class vtkLidarReaderInternal;
protected:
  vtkLidarReader();

  int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*);

  /**
   * @brief SetPimpInternal method used to switch the opaque pointers
   */
  void SetPimpInternal(vtkLidarReaderInternal* internal, LidarPacketInterpretor* interpretor);
  vtkLidarReaderInternal* Internal;

private:
  vtkLidarReader(const vtkLidarReader&); // not implemented
  void operator=(const vtkLidarReader&); // not implemented
};

#endif // VTKLIDARREADER_H
