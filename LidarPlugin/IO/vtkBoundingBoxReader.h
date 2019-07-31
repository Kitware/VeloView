#ifndef VTKBOUNDINGBOXREADER_H
#define VTKBOUNDINGBOXREADER_H

#include <string>

#include <vtkMultiBlockDataSetAlgorithm.h>

/**
 * @brief The vtkBoundingBoxReader create Bounding boxes from a specific yaml format. Right now only 2D bounding boxes are supported.
 */
class VTK_EXPORT vtkBoundingBoxReader : public vtkMultiBlockDataSetAlgorithm
{
public:
  static vtkBoundingBoxReader* New();
  vtkTypeMacro(vtkBoundingBoxReader, vtkMultiBlockDataSetAlgorithm)

  vtkSetMacro(FileName, std::string)

  vtkSetMacro(ImageHeight, int)

protected:
  vtkBoundingBoxReader();

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

  //! yaml annotation file containnig bounding box informations
  std::string FileName = "";
  //! image height needed to go from the image referential to the vtk referential
  int ImageHeight = 0;

private:
  vtkBoundingBoxReader(const vtkBoundingBoxReader&) = delete;
  void operator =(const vtkBoundingBoxReader&) = delete;
};

#endif // VTKBOUNDINGBOXREADER_H
