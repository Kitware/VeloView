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

protected:
  vtkTrailingFrame();

  int FillOutputPortInformation(int port, vtkInformation* info) override;
  int RequestUpdateExtent(vtkInformation*,
                          vtkInformationVector**,
                          vtkInformationVector*) override;
  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector) override;

private:
  //! Number of previous timestep to display
  unsigned int NumberOfTrailingFrames;

  //! Original pipeline time which must be restored after modifying the input filter time
  double PipelineTime;
  //! Index of time step corresponding to PipelineTime
  int PipelineIndex;
  //! Time index range that should be in the cache, it's e right half-open interval : [Tstart, Tend[
  int CacheTimeRange[2];
  //! Last Time index required from the filter to its input filter
  int LastTimeProcessedIndex;
  //! Indicate if the next time to process is after or before the last processed
  int Direction;
  //! Cache to save ouput previously produced by the filter
  vtkNew<vtkMultiBlockDataSet> Cache;
  //! List of available time steps from the source
  std::vector<double> TimeSteps;

  //! Help variable
  bool FirstFilterIteration;

  vtkTrailingFrame(const vtkTrailingFrame&); // not implemented
  void operator=(const vtkTrailingFrame&); // not implemented
};

#endif // VTKTRAILINGFRAME_H
