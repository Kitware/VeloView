#ifndef vtkTemporalTransformsRemapper_H
#define vtkTemporalTransformsRemapper_H

#include <vtkPolyDataAlgorithm.h>

class VTK_EXPORT vtkTemporalTransformsRemapper : public vtkPolyDataAlgorithm
{
public:
  static vtkTemporalTransformsRemapper *New();
  vtkTypeMacro(vtkTemporalTransformsRemapper, vtkPolyDataAlgorithm);

protected:
  vtkTemporalTransformsRemapper();
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;


private:
  vtkTemporalTransformsRemapper(const vtkTemporalTransformsRemapper&) = delete;
  void operator = (const vtkTemporalTransformsRemapper&) = delete;
};

#endif // vtkTemporalTransformsRemapper_H
