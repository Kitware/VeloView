#ifndef VTKTEMPORALTRANSFORMS_H
#define VTKTEMPORALTRANSFORMS_H

#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkVelodyneTransformInterpolator.h>

/**
 * @brief The vtkTemporalTransforms class store some vtkTransform associated with time
 * as a polyData containing a polyline where each point correspond to timestamp
 * sensor pos.
 * The sensor position is store as a point for vizualisation purpose.
 * The sensor orientation is store in a 4D array named "Orientation(AxisAngle):
 *  - the angle w (radian) is store at index 0;
 *  - the axis (x,y,z) is store at index 1,2,3
 * The sensor timestamp (second) is stored in a array named "Timestamp"
 *
 * It can be used to pass an vtkTransformInterpolator between 2 filter: a filter inherit from
 * vtkAlgorithm, and it can only take a vtkDataObject as input/output.
 */
class VTK_EXPORT vtkTemporalTransforms : public vtkPolyData
{
public:
  static vtkTemporalTransforms *New();
  vtkTypeMacro(vtkTemporalTransforms,vtkPolyData)

  static vtkSmartPointer<vtkTemporalTransforms> CreateFromPolyData(vtkPolyData* poly);
//  static vtkSmartPointer<vtkTemporalTransforms> CreateFromInterpolator(const vtkVelodyneTransformInterpolator*);

  vtkSmartPointer<vtkVelodyneTransformInterpolator> CreateInterpolator();

  //@{
  /// Get/Set the orientation array
  vtkDataArray* GetOrientationArray() { return this->GetPointData()->GetArray(OrientationArrayName); }
  void SetOrientationArray(vtkDoubleArray *array);
  //@}


  //@{
  /// Get/Set the translation array
  vtkDataArray* GetTranslationArray() { return this->GetPoints()->GetData(); }
  void SetTranslationArray(vtkDataArray *array);
  //@}


  //@{
  /// Get/Set the time array, this will modify it's name
  vtkDataArray* GetTimeArray() { return this->GetPointData()->GetArray(TimeArrayName); }
  void SetTimeArray(vtkDoubleArray *array);
  //@}

protected:
  vtkTemporalTransforms() = default;

private:
  char const* OrientationArrayName = "Orientation(AxisAngle)";
  char const* TimeArrayName = "Time";

  vtkTemporalTransforms(const vtkTemporalTransforms&) /*= delete*/;
  void operator =(const vtkTemporalTransforms&) /*= delete*/;
};

#endif // VTKTEMPORALTRANSFORMS_H
