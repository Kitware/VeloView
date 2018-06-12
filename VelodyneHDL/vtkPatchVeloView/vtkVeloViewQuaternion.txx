/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVeloViewQuaternion.txx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkVeloViewQuaternion.h"

#ifndef vtkVeloViewQuaternion_txx
#define vtkVeloViewQuaternion_txx

#include "vtkMath.h"

#include <cmath>

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T>::vtkVeloViewQuaternion()
{
  this->ToIdentity();
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T>
::vtkVeloViewQuaternion(const T& w, const T& x, const T& y, const T& z)
{
  this->Data[0] = w;
  this->Data[1] = x;
  this->Data[2] = y;
  this->Data[3] = z;
}

//----------------------------------------------------------------------------
template<typename T> T vtkVeloViewQuaternion<T>::SquaredNorm() const
{
  T result = 0.0;
  for (int i = 0; i < 4; ++i)
    {
    result += this->Data[i] * this->Data[i];
    }
  return result;
}

//----------------------------------------------------------------------------
template<typename T> T vtkVeloViewQuaternion<T>::Norm() const
{
  return sqrt(this->SquaredNorm());
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::ToIdentity()
{
  this->Set(1.0, 0.0 ,0.0, 0.0);
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::Identity()
{
  vtkVeloViewQuaternion<T> identity(1.0, 0.0, 0.0, 0.0);
  return identity;
}

//----------------------------------------------------------------------------
template<typename T> T vtkVeloViewQuaternion<T>::Normalize()
{
  T norm = this->Norm();
  if (norm != 0.0)
    {
    for (int i = 0; i < 4; ++i)
      {
      this->Data[i] /= norm;
      }
    }
  return norm;
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::Normalized() const
{
  vtkVeloViewQuaternion<T> temp(*this);
  temp.Normalize();
  return temp;
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::Conjugate()
{
  for (int i = 1; i < 4; ++i)
    {
    this->Data[i] *= -1.0;
    }
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::Conjugated() const
{
  vtkVeloViewQuaternion<T> ret(*this);
  ret.Conjugate();
  return ret;
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::Invert()
{
  T squareNorm = this->SquaredNorm();
  if (squareNorm != 0.0)
    {
    this->Conjugate();
    for (int i = 0; i < 4; ++i)
      {
      this->Data[i] /= squareNorm;
      }
    }
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::Inverse() const
{
  vtkVeloViewQuaternion<T> ret(*this);
  ret.Invert();
  return ret;
}

//----------------------------------------------------------------------------
template<typename T>
template<typename CastTo> vtkVeloViewQuaternion<CastTo> vtkVeloViewQuaternion<T>::Cast() const
{
  vtkVeloViewQuaternion<CastTo> result;
  for (int i = 0; i < 4; ++i)
    {
    result[i] = static_cast<CastTo>(this->Data[i]);
    }
  return result;
}

//----------------------------------------------------------------------------
template<typename T>
void vtkVeloViewQuaternion<T>::Set(const T& w, const T& x, const T& y, const T& z)
{
  this->Data[0] = w;
  this->Data[1] = x;
  this->Data[2] = y;
  this->Data[3] = z;
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::Set(T quat[4])
{
  for(int i = 0; i < 4; ++i)
    {
    this->Data[i] = quat[i];
    }
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::Get(T quat[4]) const
{
  for(int i = 0; i < 4; ++i)
    {
    quat[i] = this->Data[i];
    }
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::SetW(const T& w)
{
  this->Data[0] = w;
}

//----------------------------------------------------------------------------
template<typename T> const T& vtkVeloViewQuaternion<T>::GetW() const
{
  return this->Data[0];
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::SetX(const T& x)
{
  this->Data[1] = x;
}

//----------------------------------------------------------------------------
template<typename T> const T& vtkVeloViewQuaternion<T>::GetX() const
{
  return this->Data[1];
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::SetY(const T& y)
{
  this->Data[2] = y;
}

//----------------------------------------------------------------------------
template<typename T> const T& vtkVeloViewQuaternion<T>::GetY() const
{
  return this->Data[2];
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::SetZ(const T& z)
{
  this->Data[3] = z;
}

//----------------------------------------------------------------------------
template<typename T> const T& vtkVeloViewQuaternion<T>::GetZ() const
{
  return this->Data[3];
}

//----------------------------------------------------------------------------
template<typename T>
T vtkVeloViewQuaternion<T>::GetRotationAngleAndAxis(T axis[3]) const
{
  vtkVeloViewQuaternion<T> normedQuat(*this);
  normedQuat.Normalize();

  T angle = acos(normedQuat.GetW()) * 2.0;
  T f = sin( angle * 0.5 );
  if (f != 0.0)
    {
    axis[0] = normedQuat.GetX() / f;
    axis[1] = normedQuat.GetY() / f;
    axis[2] = normedQuat.GetZ() / f;
    }
  else
    {
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 0.0;
    }

  return angle;
}

//----------------------------------------------------------------------------
template<typename T>
void vtkVeloViewQuaternion<T>::SetRotationAngleAndAxis (T angle, T axis[3])
{
  this->SetRotationAngleAndAxis(angle, axis[0], axis[1], axis[2]);
}

//----------------------------------------------------------------------------
template<typename T> void
vtkVeloViewQuaternion<T>::SetRotationAngleAndAxis (const T& angle,
                                           const T& x,
                                           const T& y,
                                           const T& z)
{
  T axisNorm = x*x + y*y + z*z;
  if (axisNorm != 0.0)
    {
    T w = cos(angle / 2.0);
    this->SetW(w);

    T f = sin( angle / 2.0);
    this->SetX((x / axisNorm) * f);
    this->SetY((y / axisNorm) * f);
    this->SetZ((z / axisNorm) * f);
    }
  else
    {
    this->Set(0.0, 0.0, 0.0, 0.0);
    }
}

//----------------------------------------------------------------------------
template<typename T>
vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::operator+(const vtkVeloViewQuaternion<T>& q) const
{
  vtkVeloViewQuaternion<T> ret;
  for (int i = 0; i < 4; ++i)
    {
    ret[i] = this->Data[i] + q[i];
    }
  return ret;
}

//----------------------------------------------------------------------------
template<typename T>
vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::operator-(const vtkVeloViewQuaternion<T>& q) const
{
  vtkVeloViewQuaternion<T> ret;
  for (int i = 0; i < 4; ++i)
    {
    ret[i] = this->Data[i] - q[i];
    }
  return ret;
}

//----------------------------------------------------------------------------
template<typename T>
vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::operator*(const vtkVeloViewQuaternion<T>& q) const
{
  vtkVeloViewQuaternion<T> ret;
  T ww = this->Data[0]*q[0];
  T wx = this->Data[0]*q[1];
  T wy = this->Data[0]*q[2];
  T wz = this->Data[0]*q[3];

  T xw = this->Data[1]*q[0];
  T xx = this->Data[1]*q[1];
  T xy = this->Data[1]*q[2];
  T xz = this->Data[1]*q[3];

  T yw = this->Data[2]*q[0];
  T yx = this->Data[2]*q[1];
  T yy = this->Data[2]*q[2];
  T yz = this->Data[2]*q[3];

  T zw = this->Data[3]*q[0];
  T zx = this->Data[3]*q[1];
  T zy = this->Data[3]*q[2];
  T zz = this->Data[3]*q[3];

  ret[0] = ww-xx-yy-zz;
  ret[1] = wx+xw+yz-zy;
  ret[2] = wy-xz+yw+zx;
  ret[3] = wz+xy-yx+zw;
  return ret;
}

//----------------------------------------------------------------------------
template<typename T>
vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::operator*(const T& scalar) const
{
  vtkVeloViewQuaternion<T> ret;
  for (int i = 0; i < 4; ++i)
    {
    ret[i] = this->Data[i] * scalar;
    }
  return ret;
}

//----------------------------------------------------------------------------
template<typename T>
void vtkVeloViewQuaternion<T>::operator*=(const T& scalar) const
{
  for (int i = 0; i < 4; ++i)
    {
    this->Data[i] *= scalar;
    }
}

//----------------------------------------------------------------------------
template<typename T>
vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::operator/(const vtkVeloViewQuaternion<T>& q) const
{
  vtkVeloViewQuaternion<T> inverseQuaternion = q.Inverse();
  return (*this)*inverseQuaternion;
}

//----------------------------------------------------------------------------
template<typename T>
vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::operator/(const T& scalar) const
{
  vtkVeloViewQuaternion<T> ret;
  for (int i = 0; i < 4; ++i)
    {
    ret[i] = this->Data[i] / scalar;
    }
  return ret;
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::operator/=(const T& scalar)
{
  for (int i = 0; i < 4; ++i)
    {
    this->Data[i] /= scalar;
    }
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::ToMatrix3x3(T A[3][3]) const
{
  T ww = this->Data[0]*this->Data[0];
  T wx = this->Data[0]*this->Data[1];
  T wy = this->Data[0]*this->Data[2];
  T wz = this->Data[0]*this->Data[3];

  T xx = this->Data[1]*this->Data[1];
  T yy = this->Data[2]*this->Data[2];
  T zz = this->Data[3]*this->Data[3];

  T xy = this->Data[1]*this->Data[2];
  T xz = this->Data[1]*this->Data[3];
  T yz = this->Data[2]*this->Data[3];

  T rr = xx + yy + zz;
  // normalization factor, just in case quaternion was not normalized
  T f;
  if (ww + rr == 0.0) //means the quaternion is (0, 0, 0, 0)
    {
    A[0][0] = 0.0;  A[1][0] = 0.0;  A[2][0] = 0.0;
    A[0][1] = 0.0;  A[1][1] = 0.0;  A[2][1] = 0.0;
    A[0][2] = 0.0;  A[1][2] = 0.0;  A[2][2] = 0.0;
    return;
    }
  f = 1.0/(ww + rr);

  T s = (ww - rr)*f;
  f *= 2.0;

  A[0][0] = xx*f + s;
  A[1][0] = (xy + wz)*f;
  A[2][0] = (xz - wy)*f;

  A[0][1] = (xy - wz)*f;
  A[1][1] = yy*f + s;
  A[2][1] = (yz + wx)*f;

  A[0][2] = (xz + wy)*f;
  A[1][2] = (yz - wx)*f;
  A[2][2] = zz*f + s;
}

//----------------------------------------------------------------------------
//  The solution is based on
//  Berthold K. P. Horn (1987),
//  "Closed-form solution of absolute orientation using unit quaternions,"
//  Journal of the Optical Society of America A, 4:629-642
template<typename T> void vtkVeloViewQuaternion<T>::FromMatrix3x3(const T A[3][3])
{
  T n[4][4];

  // on-diagonal elements
  n[0][0] =  A[0][0]+A[1][1]+A[2][2];
  n[1][1] =  A[0][0]-A[1][1]-A[2][2];
  n[2][2] = -A[0][0]+A[1][1]-A[2][2];
  n[3][3] = -A[0][0]-A[1][1]+A[2][2];

  // off-diagonal elements
  n[0][1] = n[1][0] = A[2][1]-A[1][2];
  n[0][2] = n[2][0] = A[0][2]-A[2][0];
  n[0][3] = n[3][0] = A[1][0]-A[0][1];

  n[1][2] = n[2][1] = A[1][0]+A[0][1];
  n[1][3] = n[3][1] = A[0][2]+A[2][0];
  n[2][3] = n[3][2] = A[2][1]+A[1][2];

  T eigenvectors[4][4];
  T eigenvalues[4];

  // convert into format that JacobiN can use,
  // then use Jacobi to find eigenvalues and eigenvectors
  T* nTemp[4];
  T* eigenvectorsTemp[4];
  for (int i = 0; i < 4; ++i)
    {
    nTemp[i] = n[i];
    eigenvectorsTemp[i] = eigenvectors[i];
    }
  vtkMath::JacobiN(nTemp,4,eigenvalues,eigenvectorsTemp);

  // the first eigenvector is the one we want
  for (int i = 0; i < 4; ++i)
    {
    this->Data[i] = eigenvectors[i][0];
    }
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>
::Slerp(T t, const vtkVeloViewQuaternion<T>& q1) const
{
  T axis0[3], axis1[3];
  this->GetRotationAngleAndAxis(axis0);
  q1.GetRotationAngleAndAxis(axis1);

  // Canonical scalar product on quaternion

  T dot = this->GetW() * q1.GetW() +
          this->GetX() * q1.GetX() +
          this->GetY() * q1.GetY() +
          this->GetZ() * q1.GetZ();

  // To prevent the SLERP interpolation to take the long path
  // we first check their relative orientation. If the angle
  // is superior to 90 we take the opposite quaternion which
  // is closer and represents the same rotation

  vtkVeloViewQuaternion<T> qClosest = q1;

  if(dot < 0)
  {
    dot = -dot;
    qClosest = qClosest*(-1);
  }

  // To avoid division by zero, perform a linear interpolation (LERP), if our
  // quarternions are nearly in the same direction, otherwise resort
  // to spherical linear interpolation. In the limiting case (for small
  // angles), SLERP is equivalent to LERP.

  T t1, t2;

  if ((1.0 - fabs(dot)) < 1e-6)
  {
    t1 = 1.0-t;
    t2 = t;
  }
  else
  {
    // Angle (defined by the canonical scalar product for quaternions)
    // between the two quaternions
    const T theta = acos( dot );
    t1 = sin((1.0-t)*theta)/sin(theta);
    t2 = sin(t*theta)/sin(theta);
  }

  return (*this)*t1 + qClosest*t2;
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>
::InnerPoint(const vtkVeloViewQuaternion<T>& q1, const vtkVeloViewQuaternion<T>& q2) const
{
  vtkVeloViewQuaternion<T> qInv = q1.Inverse();
  vtkVeloViewQuaternion<T> qL = qInv*q2;
  vtkVeloViewQuaternion<T> qR = qInv*(*this);

  vtkVeloViewQuaternion<T> qLLog = qL.UnitLog();
  vtkVeloViewQuaternion<T> qRLog = qR.UnitLog();
  vtkVeloViewQuaternion<T> qSum = qLLog + qRLog;
  T w = qSum.GetW();
  qSum /= -4.0;
  qSum.SetW(w);

  vtkVeloViewQuaternion<T> qExp = qSum.UnitExp();
  return q1*qExp;
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::ToUnitLog()
{
  T axis[3];
  T angle = this->GetRotationAngleAndAxis(axis);
  T sinAngle = sin(angle);

  this->Set(0.0, sinAngle*axis[0], sinAngle*axis[1], sinAngle*axis[2]);
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::UnitLog() const
{
  vtkVeloViewQuaternion<T> unitLog(*this);
  unitLog.ToUnitLog();
  return unitLog;
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::ToUnitExp()
{
  T axis[3];
  T angle = this->GetRotationAngleAndAxis(axis);
  T sinAngle = sin(angle);

  this->Set(cos(angle), sinAngle*axis[0], sinAngle*axis[1], sinAngle*axis[2]);
}

//----------------------------------------------------------------------------
template<typename T> vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::UnitExp() const
{
  vtkVeloViewQuaternion<T> unitExp(*this);
  unitExp.ToUnitExp();
  return unitExp;
}

//----------------------------------------------------------------------------
template<typename T> void vtkVeloViewQuaternion<T>::NormalizeWithAngleInDegrees()
{
  this->Normalize();
  this->SetW( vtkMath::DegreesFromRadians(this->GetW()) );
}

//----------------------------------------------------------------------------
template<typename T>
vtkVeloViewQuaternion<T> vtkVeloViewQuaternion<T>::NormalizedWithAngleInDegrees() const
{
  vtkVeloViewQuaternion<T> unitVTK(*this);
  unitVTK.Normalize();
  unitVTK.SetW( vtkMath::DegreesFromRadians( unitVTK.GetW() ) );
  return unitVTK;
}

#endif
