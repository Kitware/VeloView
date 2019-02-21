/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVelodyneTransformInterpolator.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkVelodyneTransformInterpolator.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkProp3D.h"
#include "vtkPatchVeloView/vtkVeloViewQuaternion.h"
#include "vtkPatchVeloView/vtkVeloViewQuaternionInterpolator.h"
#include "vtkTransform.h"
#include "vtkPatchVeloView/vtkVeloViewTupleInterpolator.h"
#include <list>
#include <vector>
#include <set>
#include <algorithm>
#include <iterator>

vtkStandardNewMacro(vtkVelodyneTransformInterpolator);

// PIMPL STL encapsulation for list of transforms, and list of
// quaternions. This just keeps track of all the data the user specifies,
// which is later dumped into the interpolators.
struct vtkQTransform
{
  double Time;
  double P[3];
  double S[3];
  vtkVeloViewQuaterniond Q;

  vtkQTransform()
  {
    this->Time = 0.0;
    this->P[0] = this->P[1] = this->P[2] = 0.0;
    this->S[0] = this->S[1] = this->S[2] = 0.0;
  }
  vtkQTransform(double t, vtkTransform* xform)
  {
    this->Time = t;
    if (xform)
    {
      xform->GetPosition(this->P);
      xform->GetScale(this->S);
      double q[4];
      xform->GetOrientationWXYZ(q); // Rotation (in degrees) around unit vector
      q[0] = vtkMath::RadiansFromDegrees(q[0]);
      this->Q.SetRotationAngleAndAxis(q[0], q + 1);

      if (this->Q.GetW() < 0.0)
      {
        this->Q = this->Q * -1;
      }
    }
    else
    {
      this->P[0] = this->P[1] = this->P[2] = 0.0;
      this->S[0] = this->S[1] = this->S[2] = 0.0;
    }
  }
};

class vtkQTransformComparator
{
public:
  bool operator () ( const vtkQTransform& transform1,
                     const vtkQTransform& transform2 )
    {
      return transform1.Time < transform2.Time;
    }
};

// The list is arranged in increasing order in T
class vtkTransformList : public std::list<vtkQTransform>
{
};
typedef vtkTransformList::iterator TransformListIterator;

//----------------------------------------------------------------------------
std::vector<std::vector<double> > vtkVelodyneTransformInterpolator::GetTransformList()
{
  this->InitializeInterpolation();

  std::vector<std::vector<double> > transforms;
  // Okay, insert in sorted order
  for (TransformListIterator iter = this->TransformList->begin(); iter != this->TransformList->end(); ++iter)
  {
    std::vector<double> currentTransform(7, 0);
    // time
    currentTransform[0] = iter->Time;
    // position
    currentTransform[4] = iter->P[0];
    currentTransform[5] = iter->P[1];
    currentTransform[6] = iter->P[2];
    // orientation
    double A[3][3];
    iter->Q.ToMatrix3x3(A);
    currentTransform[1] = std::atan2(A[2][1], A[2][2]);
    currentTransform[2] = -std::asin(A[2][0]);
    currentTransform[3] = std::atan2(A[1][0], A[0][0]);

    transforms.push_back(currentTransform);
  }

  return transforms;
}

//----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator::vtkVelodyneTransformInterpolator()
{
  // Set up the interpolation
  this->InterpolationType = INTERPOLATION_TYPE_SPLINE;

  // Spline interpolation
  this->PositionInterpolator = vtkVeloViewTupleInterpolator::New();
  this->ScaleInterpolator = vtkVeloViewTupleInterpolator::New();
  this->RotationInterpolator = vtkVeloViewQuaternionInterpolator::New();

  // Quaternion interpolation
  this->TransformList = new vtkTransformList;
  this->Initialized = 0;
}

//----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator::~vtkVelodyneTransformInterpolator()
{
  delete this->TransformList;

  if (this->PositionInterpolator)
  {
    this->PositionInterpolator->Delete();
  }
  if (this->ScaleInterpolator)
  {
    this->ScaleInterpolator->Delete();
  }
  if (this->RotationInterpolator)
  {
    this->RotationInterpolator->Delete();
  }
}

//----------------------------------------------------------------------------
vtkMTimeType vtkVelodyneTransformInterpolator::GetMTime()
{
  unsigned long mTime = this->Superclass::GetMTime();
  unsigned long posMTime, scaleMTime, rotMTime;

  if (this->PositionInterpolator)
  {
    posMTime = this->PositionInterpolator->GetMTime();
    mTime = (posMTime > mTime ? posMTime : mTime);
  }
  if (this->ScaleInterpolator)
  {
    scaleMTime = this->ScaleInterpolator->GetMTime();
    mTime = (scaleMTime > mTime ? scaleMTime : mTime);
  }
  if (this->RotationInterpolator)
  {
    rotMTime = this->RotationInterpolator->GetMTime();
    mTime = (rotMTime > mTime ? rotMTime : mTime);
  }

  return mTime;
}

//----------------------------------------------------------------------------
int vtkVelodyneTransformInterpolator::GetNumberOfTransforms()
{
  return static_cast<int>(this->TransformList->size());
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::GetSample(int n,
                                              vtkTransform *xform,
                                              double& xformTime)
{
  if (this->TransformList->empty())
  {
    return;
  }

  this->InitializeInterpolation();

  if (n < 0 || n >= static_cast<int>(this->TransformVector.size()))
  {
    return;
  }

  // Get the transform
  xform->Identity();
  xform->Translate(this->TransformVector[n].P);
  double Q[4];
  Q[0] = vtkMath::DegreesFromRadians(this->TransformVector[n].Q.GetRotationAngleAndAxis(Q+1));
  xform->RotateWXYZ(Q[0],Q+1);
  xform->Scale(this->TransformVector[n].S);

  xformTime = this->TransformVector[n].Time;
}

//----------------------------------------------------------------------------
double vtkVelodyneTransformInterpolator::GetMinimumT()
{
  if (this->TransformList->empty())
  {
    return -VTK_FLOAT_MAX;
  }
  else
  {
    return this->TransformList->front().Time;
  }
}

//----------------------------------------------------------------------------
double vtkVelodyneTransformInterpolator::GetMaximumT()
{
  if (this->TransformList->empty())
  {
    return VTK_FLOAT_MAX;
  }
  else
  {
    return this->TransformList->back().Time;
  }
}

//----------------------------------------------------------------------------
double vtkVelodyneTransformInterpolator::GetPeriod()
{
  return (this->GetMaximumT() - this->GetMinimumT()) / (this->GetNumberOfTransforms() - 1);
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::Initialize()
{
  this->TransformList->clear();
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::AddTransform(double t, vtkTransform* xform)
{
  int size = static_cast<int>(this->TransformList->size());

  // Check special cases: t at beginning or end of list
  if (size <= 0 || t < this->TransformList->front().Time)
  {
    this->TransformList->push_front(vtkQTransform(t, xform));
    return;
  }
  else if (t > this->TransformList->back().Time)
  {
    this->TransformList->push_back(vtkQTransform(t, xform));
    return;
  }
  else if (size == 1 && t == this->TransformList->back().Time)
  {
    this->TransformList->front() = vtkQTransform(t, xform);
    return;
  }

  // Okay, insert in sorted order
  TransformListIterator iter = this->TransformList->begin();
  TransformListIterator nextIter = ++(this->TransformList->begin());
  for (int i = 0; i < (size - 1); i++, ++iter, ++nextIter)
  {
    if (t == iter->Time)
    {
      (*iter) = vtkQTransform(t, xform);
    }
    else if (t > iter->Time && t < nextIter->Time)
    {
      this->TransformList->insert(nextIter, vtkQTransform(t, xform));
    }
  }

  this->Modified();
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::AddTransform(double t, vtkMatrix4x4* matrix)
{
  vtkTransform* xform = vtkTransform::New();
  xform->SetMatrix(matrix);
  this->AddTransform(t, xform);
  xform->Delete();
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::AddTransform(double t, vtkProp3D* prop3D)
{
  this->AddTransform(t, prop3D->GetMatrix());
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::RemoveTransform(double t)
{
  if (t < this->TransformList->front().Time || t > this->TransformList->back().Time)
  {
    return;
  }

  TransformListIterator iter = this->TransformList->begin();
  for (; iter->Time != t && iter != this->TransformList->end(); ++iter)
  {
  }
  if (iter != this->TransformList->end())
  {
    this->TransformList->erase(iter);
  }
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::SetPositionInterpolator(vtkVeloViewTupleInterpolator* pi)
{
  if (this->PositionInterpolator != pi)
  {
    if (this->PositionInterpolator != NULL)
    {
      this->PositionInterpolator->Delete();
    }
    this->PositionInterpolator = pi;
    if (this->PositionInterpolator != NULL)
    {
      this->PositionInterpolator->Register(this);
    }
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::SetScaleInterpolator(vtkVeloViewTupleInterpolator* si)
{
  if (this->ScaleInterpolator != si)
  {
    if (this->ScaleInterpolator != NULL)
    {
      this->ScaleInterpolator->Delete();
    }
    this->ScaleInterpolator = si;
    if (this->ScaleInterpolator != NULL)
    {
      this->ScaleInterpolator->Register(this);
    }
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::SetRotationInterpolator(vtkVeloViewQuaternionInterpolator* ri)
{
  if (this->RotationInterpolator != ri)
  {
    if (this->RotationInterpolator != NULL)
    {
      this->RotationInterpolator->Delete();
    }
    this->RotationInterpolator = ri;
    if (this->RotationInterpolator != NULL)
    {
      this->RotationInterpolator->Register(this);
    }
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::InitializeInterpolation()
{
  if (this->TransformList->empty())
  {
    return;
  }

  // Set up the interpolators if we need to
  if (!this->Initialized || this->GetMTime() > this->InitializeTime)
  {
    if (!this->PositionInterpolator)
    {
      this->PositionInterpolator = vtkVeloViewTupleInterpolator::New();
    }
    if (!this->ScaleInterpolator)
    {
      this->ScaleInterpolator = vtkVeloViewTupleInterpolator::New();
    }
    if (!this->RotationInterpolator)
    {
      this->RotationInterpolator = vtkVeloViewQuaternionInterpolator::New();
    }

    if (this->InterpolationType == INTERPOLATION_TYPE_LINEAR
             || this->InterpolationType == INTERPOLATION_TYPE_NEAREST
             || this->InterpolationType == INTERPOLATION_TYPE_NEAREST_LOW_BOUNDED)
    {
      this->PositionInterpolator->SetInterpolationTypeToLinear();
      this->ScaleInterpolator->SetInterpolationTypeToLinear();
      this->RotationInterpolator->SetInterpolationTypeToLinear();
      this->TransformVector.clear();
      this->TransformVector.resize(0);
      std::list<vtkQTransform>::iterator transform;
      for (transform = this->TransformList->begin(); transform != this->TransformList->end(); ++transform)
      {
        this->TransformVector.push_back(*transform);
      }
    }
    else if (this->InterpolationType == INTERPOLATION_TYPE_SPLINE)
    {
      this->PositionInterpolator->SetInterpolationTypeToSpline();
      this->ScaleInterpolator->SetInterpolationTypeToSpline();
      this->RotationInterpolator->SetInterpolationTypeToSpline();
    }
    else
    {
      ; // manual override, user manipulates interpolators directly
    }

    this->PositionInterpolator->Initialize();
    this->ScaleInterpolator->Initialize();
    this->RotationInterpolator->Initialize();

    this->PositionInterpolator->SetNumberOfComponents(3);
    this->ScaleInterpolator->SetNumberOfComponents(3);

    // Okay, now we can load the interpolators with data
    // Initialize the data pointers
    int nb = this->TransformList->size();
    double *time = new double[nb];
    double **Position = new double*[3];
    double **Scale = new double*[3];
    for (int k = 0; k < 3; ++k)
    {
      Position[k] = new double[nb];
      Scale[k] = new double[nb];
    }

    // Fill the data pointers
    TransformListIterator iter = this->TransformList->begin();
    int count = 0;
    for ( ; iter != this->TransformList->end(); ++iter)
    {
      /*this->PositionInterpolator->AddTuple(iter->Time,iter->P);
      this->ScaleInterpolator->AddTuple(iter->Time,iter->S);*/
      Position[0][count] = iter->P[0];
      Position[1][count] = iter->P[1];
      Position[2][count] = iter->P[2];
      Scale[0][count] = iter->S[0];
      Scale[1][count] = iter->S[1];
      Scale[2][count] = iter->S[2];
      time[count] = iter->Time;
      this->RotationInterpolator->AddQuaternion(iter->Time,iter->Q);
      count++;
    }

    // Fill the interpolators
    this->PositionInterpolator->FillFromData(nb, time, Position);
    this->ScaleInterpolator->FillFromData(nb, time, Scale);

    for (int k = 0; k < 3; ++k)
    {
      delete [] Position[k];
      delete [] Scale[k];
    }

    delete [] Position;
    delete [] Scale;

    this->Initialized = 1;
    this->InitializeTime.Modified();
  }
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::InterpolateTransform(double t, vtkTransform* xform)
{
  if (this->TransformList->empty())
  {
    return;
  }

  if (this->InterpolationType == INTERPOLATION_TYPE_NEAREST
      || this->InterpolationType == INTERPOLATION_TYPE_NEAREST_LOW_BOUNDED)
  {
    this->InterpolateTransformNearest(t, xform);
    return;
  }

  // Make sure the xform and this class are initialized properly
  xform->Identity();
  this->InitializeInterpolation();

  // Evaluate the interpolators
  if (t < this->TransformList->front().Time)
  {
    t = this->TransformList->front().Time;
  }

  else if (t > this->TransformList->back().Time)
  {
    t = this->TransformList->back().Time;
  }

  double P[3], S[3], Q[4];
  vtkVeloViewQuaterniond q;
  this->PositionInterpolator->InterpolateTupleDichotomic(t, P);
  this->ScaleInterpolator->InterpolateTupleDichotomic(t, S);
  this->RotationInterpolator->InterpolateQuaternion(t, q);
  Q[0] = vtkMath::DegreesFromRadians(q.GetRotationAngleAndAxis(Q + 1));

  xform->Translate(P);
  xform->RotateWXYZ(Q[0], Q + 1);
  xform->Scale(S);
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::InterpolateTransformNearest(double t,
                                                    vtkTransform *xform)
{
  if (this->TransformList->empty())
  {
    return;
  }

  // Make sure the xform and this class are initialized properly
  xform->Identity();
  this->InitializeInterpolation();

  if (this->TransformVector.size() < 2)
  {
    return;
  }

  // vtkQTransform order relation based on the Time
  vtkQTransformComparator comparatorTimeTransform;

  // Get the low bound to procees to a nearest
  // low bounded interpolator
  vtkQTransform transform;
  transform.Time = t;
  std::vector<vtkQTransform>::iterator lowerBound;
  lowerBound = std::lower_bound(this->TransformVector.begin(), this->TransformVector.end(), transform, comparatorTimeTransform);

  if (this->InterpolationType == INTERPOLATION_TYPE_NEAREST_LOW_BOUNDED)
  {
    // Are we before the first node? If not take the
    // previous transform to have a low bounded nearest
    // interpolator.
    if (!(lowerBound == this->TransformVector.begin()))
    {
      lowerBound--;
    }
  }
  else // i.e. this->InterpolationType == INTERPOLATION_TYPE_NEAREST
  {
    // Unless the lowerBound is the first one, we have to compare it with
    // its predecessor to keep the closest in time to t.

    // Because t has already been clamped,
    // lowerBound->Time - t should be positive
    // but adding std::abs makes the code more robust.
    if (lowerBound != this->TransformVector.begin() &&
        t - (lowerBound - 1)->Time <= std::abs(lowerBound->Time - t))
    {
      lowerBound--;
    }
  }

  // Get the transform
  xform->Identity();
  xform->Translate(lowerBound->P);
  double Q[4];
  Q[0] = vtkMath::DegreesFromRadians(lowerBound->Q.GetRotationAngleAndAxis(Q+1));
  xform->RotateWXYZ(Q[0],Q+1);
  xform->Scale(lowerBound->S);
}

//----------------------------------------------------------------------------
void vtkVelodyneTransformInterpolator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "There are " << this->GetNumberOfTransforms()
     << " transforms to be interpolated\n";

  os << indent << "Interpolation Type: ";
  if (this->InterpolationType == INTERPOLATION_TYPE_LINEAR)
  {
    os << "Linear\n";
  }
  else if (this->InterpolationType == INTERPOLATION_TYPE_SPLINE)
  {
    os << "Spline\n";
  }
  else if (this->InterpolationType == INTERPOLATION_TYPE_MANUAL)
  {
    os << "Manual\n";
  }
  else if (this->InterpolationType == INTERPOLATION_TYPE_NEAREST)
  {
    os << "Nearest\n";
  }
  else if (this->InterpolationType == INTERPOLATION_TYPE_NEAREST_LOW_BOUNDED)
  {
    os << "Nearest low bounded\n";
  }
  else
  {
    os << "Unknown\n";
  }

  os << indent << "Position Interpolator: ";
  if (this->PositionInterpolator)
  {
    os << this->PositionInterpolator << "\n";
  }
  else
  {
    os << "(null)\n";
  }

  os << indent << "Scale Interpolator: ";
  if (this->ScaleInterpolator)
  {
    os << this->ScaleInterpolator << "\n";
  }
  else
  {
    os << "(null)\n";
  }

  os << indent << "Rotation Interpolator: ";
  if (this->RotationInterpolator)
  {
    os << this->RotationInterpolator << "\n";
  }
  else
  {
    os << "(null)\n";
  }
}
