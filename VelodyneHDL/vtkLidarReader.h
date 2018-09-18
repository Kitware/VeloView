#ifndef VTKLIDARREADER_H
#define VTKLIDARREADER_H

//#include "vtkLidarSource.h"
#include "vtkLidarProvider.h"

class vtkLidarReaderInternal;
class vtkTransform;

class vtkLidarReader : public vtkLidarProvider
{
public:
//  static vtkLidarReader* New();
  vtkTypeMacro(vtkLidarReader, vtkLidarProvider);
  void PrintSelf(ostream& os, vtkIndent indent);

  std::string GetFileName();
  void SetFileName(const std::string& filename);

  void Open();
  void Close();

  void ProcessPacket(unsigned char* data, unsigned int bytesReceived);

protected:
  vtkLidarReader();
  vtkLidarReader(vtkLidarReaderInternal* internal);
  ~vtkLidarReader() override;

  void SetPimpInternal(vtkLidarReaderInternal* internal);

  // Description:
  // This is called by the superclass.
  // This is the method you should override.
  int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);


private:
  vtkLidarReader(const vtkLidarReader&);
  void operator=(const vtkLidarReader&);

  vtkLidarReaderInternal* Internal;
};

#endif // VTKLIDARREADER_H
