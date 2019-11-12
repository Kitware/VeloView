#ifndef VTKTRAILINGFRAME_H
#define VTKTRAILINGFRAME_H

#include <queue>

#include "vtkPolyDataAlgorithm.h"
#include <vtkNew.h>
#include <vtkMultiBlockDataSet.h>

/**
 * @brief The vtkTrailingFrame class is a filter that combine consecutive timestep
 * of its input to produce a multiblock.
 * The input of this filter must produce only consecutive interger timestep.
 */
class VTK_EXPORT vtkTrailingFrame : public vtkPolyDataAlgorithm
{
public:
  static vtkTrailingFrame* New();
  vtkTypeMacro(vtkTrailingFrame, vtkPolyDataAlgorithm)

  //! @{
  //! @copydoc NumberOfTrailingFrames
  vtkGetMacro(NumberOfTrailingFrames, unsigned int)
  void SetNumberOfTrailingFrames(const unsigned int value);
  //! @}

  //! @{
  //! @copydoc UseCache
  vtkGetMacro(UseCache, bool)
  vtkSetMacro(UseCache, bool)
  //! @}

protected:
  vtkTrailingFrame() = default;

  int FillOutputPortInformation(int port, vtkInformation* info) override;
  int RequestUpdateExtent(vtkInformation*,
                          vtkInformationVector**,
                          vtkInformationVector*) override;
  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector) override;

private:
  //! Number of previous timestep to display
  unsigned int NumberOfTrailingFrames = 0;
  //! Should the internal cache be used for speed
  bool UseCache = true;

  //! Original pipeline time which must be restored after modifying the input filter time
  double PipelineTime = 0;
  //! Index of time step corresponding to PipelineTime
  int PipelineIndex = 0;
  //! Time index range that should be in the cache, it's e right half-open interval : [Tstart, Tend[
  int CacheTimeRange[2] = {-1, -1};
  //! Last Time index required from the filter to its input filter
  int LastTimeProcessedIndex = -1;
  //! Indicate if the next time to process is after or before the last processed
  int Direction = 1;
  //! Cache to save ouput previously produced by the filter
  vtkNew<vtkMultiBlockDataSet> Cache;
  //! List of available time steps from the source
  std::vector<double> TimeSteps;

  //! Help variable
  bool FirstFilterIteration = true;

  vtkTrailingFrame(const vtkTrailingFrame&); // not implemented
  void operator=(const vtkTrailingFrame&); // not implemented
};

#endif // VTKTRAILINGFRAME_H
