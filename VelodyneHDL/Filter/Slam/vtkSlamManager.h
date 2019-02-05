#ifndef VTKSLAMMANAGER_H
#define VTKSLAMMANAGER_H

#include <vtkSetGet.h>
#include "vtkSlam.h"

class vtkSlamManager : public vtkSlam
{
public:
  static vtkSlamManager *New();
  vtkTypeMacro(vtkSlamManager, vtkSlam)
  void PrintSelf(ostream& os, vtkIndent indent) override;

  //! @{ @copydoc StartFrame
  vtkGetMacro(StartFrame, int)
  vtkCustomSetMacro(StartFrame, int)
  //! @}

  //! @{ @copydoc EndFrame
  vtkGetMacro(EndFrame, int)
  vtkCustomSetMacro(EndFrame, int)
  //! @}

  //! @{ @copydoc StepSize
  vtkGetMacro(StepSize, int)
  vtkCustomSetMacro(StepSize, int)
  //! @}

  //! @{ @copydoc AllFrame
  vtkGetMacro(AllFrame, bool)
  vtkCustomSetMacro(AllFrame, bool)
  //! @}

protected:
  vtkSlamManager();
  int RequestUpdateExtent(vtkInformation*,
                          vtkInformationVector**,
                          vtkInformationVector*) override;
  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector) override;

  //! Overwrite StartFrame and EndFrame to process all the frame
  bool AllFrame = true;

  //! First frame to be process
  int StartFrame = 0;

  //! No frame after this frame will be process. In case StepSize is not 1 it correspond to
  //! the last frame process
  int EndFrame = 0;

  //! To speed up this enable to not process all the frame but only a regular sampling
  int StepSize = 1;

private:
  vtkSlamManager(const vtkSlamManager&) = delete;
  void operator = (const vtkSlamManager&) = delete;

  bool FirstIteration = true;
  int CurrentFrame = 0;
  vtkMTimeType LastModifyTime = 0;
  std::vector<vtkSmartPointer<vtkPolyData>> Cache;
};

#endif // VTKSLAMMANAGER_H
