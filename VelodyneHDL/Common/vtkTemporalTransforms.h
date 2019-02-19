#ifndef VTKTEMPORALTRANSFORMS_H
#define VTKTEMPORALTRANSFORMS_H

#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkVelodyneTransformInterpolator.h>

#include <Eigen/Geometry>

/**
 * @brief The vtkTemporalTransforms class store some vtkTransform associated with time
 * as a polyData containing a polyline where each point correspond to timestamp
 * sensor pos.
 * The sensor position is store as a point for vizualisation purpose.
 * The sensor orientation is store in a 4D array named "Orientation(AxisAngle):
 *  - the axis (x,y,z) is store at index 0,1,2
 *  - the angle w (radian) is store at index 3
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

  vtkSmartPointer<vtkTransform> GetTransform(unsigned int transformNumber);

  vtkSmartPointer<vtkVelodyneTransformInterpolator> CreateInterpolator();

  /**
  * \brief This function apply an isometric affine
  *        transform to the poses trajectory:
  *        Rout(t) = R0 * Rin(t)
  *        Tout(t) = R0 * Tin(t) + T0
  *
  *        with H = [R0 | T0]
  *
  * \@param H transform to apply
  */
  vtkSmartPointer<vtkTemporalTransforms> IsometricTransform(vtkSmartPointer<vtkTransform> H);

  /**
  * \brief This function apply a "cycloidic" transform.
  *        Let's suppose you know the Orientation Rin(t) and
  *        the position Tin(t) of a sensor1. You also know the
  *        the pose (R0, T0) of the sensor2 according to the sensor1
  *        reference coordinate frame. Then, the orientation
  *        Rout(t) and Tout(t) of the sensor2 can be computed:
  *
  *        Rout(t) = Rin(t) * R0
  *        Tout(t) = Rin(t) * T0 + Tin(t)
  *
  * \@param H transform to apply
  */
  vtkSmartPointer<vtkTemporalTransforms> CycloidicTransform(vtkSmartPointer<vtkTransform> H);

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

  /// Add a temporal transform to the end
  void PushBack(double time, const Eigen::AngleAxisd& orientation , const Eigen::Vector3d translation);

  vtkSmartPointer<vtkTemporalTransforms> ExtractTimes(double tstart, double tend);
  vtkSmartPointer<vtkTemporalTransforms> Subsample(int N);
  vtkSmartPointer<vtkTemporalTransforms> ApplyTimeshift(double shift);
  vtkSmartPointer<vtkTemporalTransforms> ApplyScale(double scale);

protected:

  vtkTemporalTransforms();

private:
  char const* OrientationArrayName = "Orientation(AxisAngle)";
  char const* TimeArrayName = "Time";

  vtkTemporalTransforms(const vtkTemporalTransforms&) /*= delete*/;
  void operator =(const vtkTemporalTransforms&) /*= delete*/;
};

#endif // VTKTEMPORALTRANSFORMS_H
