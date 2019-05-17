/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkQuaternion.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkQuaternion - templated base type for storage of quaternions.
// .SECTION Description
// This class is a templated data type for storing and manipulating
// quaternions. The quaternions have the form [w, x, y, z].
// Given a rotation of angle theta and axis v, the corresponding
// quaternion is [w, x, y, z] = [cos(theta/2), v*sin(theta/2)]
//
// This class implements the Spherical Linear interpolation (SLERP)
// and the Spherical Spline Quaternion interpolation (SQUAD).
// It is advised to use the vtkQuaternionInterpolator when dealing
// with multiple quaternions and or interpolations.
//
// .SECTION See also
// vtkQuaternionInterpolator

#ifndef vtkCustomQuaternion_h
#define vtkCustomQuaternion_h

#include "vtkTuple.h"

template<typename T> class vtkCustomQuaternion : public vtkTuple<T, 4>
{
public:
  // Description:
  // Default constructor. Creates an identity quaternion.
  vtkCustomQuaternion();

  // Description:
  // Initialize all of the quaternion's elements with the supplied scalar.
  explicit vtkCustomQuaternion(const T& scalar) : vtkTuple<T, 4>(scalar) {}

  // Description:
  // Initalize the quaternion's elements with the elements of the supplied array.
  // Note that the supplied pointer must contain at least as many elements as
  // the quaternion, or it will result in access to out of bounds memory.
  explicit vtkCustomQuaternion(const T* init) : vtkTuple<T, 4>(init) {}

  // Description:
  // Initialize the quaternion element explicitly.
  vtkCustomQuaternion(const T& w, const T& x, const T& y, const T& z);

  // Description:
  // Get the squared norm of the quaternion.
  T SquaredNorm() const;

  // Description:
  // Get the norm of the quaternion, i.e. its length.
  T Norm() const;

  // Description:
  // Set the quaternion to identity in place.
  void ToIdentity();

  // Description:
  // Return the identity quaternion.
  // Note that the default constructor also creates an identity quaternion.
  static vtkCustomQuaternion<T> Identity();

  // Description:
  // Normalize the quaternion in place.
  // Return the norm of the quaternion.
  T Normalize();

  // Description:
  // Return the normalized form of this quaternion.
  vtkCustomQuaternion<T> Normalized() const;

  // Description:
  // Conjugate the quaternion in place.
  void Conjugate();

  // Description:
  // Return the conjugate form of this quaternion.
  vtkCustomQuaternion<T> Conjugated() const;

  // Description:
  // Invert the quaternion in place.
  // This is equivalent to conjugate the quaternion and then divide
  // it by its squared norm.
  void Invert();

  // Description:
  // Return the inverted form of this quaternion.
  vtkCustomQuaternion<T> Inverse() const;

  // Description:
  // Convert this quaternion to a unit log quaternion.
  // The unit log quaternion is defined by:
  // [w, x, y, z] =  [0.0, v*sin(theta)].
  void ToUnitLog();

  // Description:
  // Return the unit log version of this quaternion.
  // The unit log quaternion is defined by:
  // [w, x, y, z] =  [0.0, v*sin(theta)].
  vtkCustomQuaternion<T> UnitLog() const;

  // Description:
  // Convert this quaternion to a unit exponential quaternion.
  // The unit exponential quaternion is defined by:
  // [w, x, y, z] =  [cos(theta), v*sin(theta)].
  void ToUnitExp();

  // Description:
  // Return the unit exponential version of this quaternion.
  // The unit exponential quaternion is defined by:
  // [w, x, y, z] =  [cos(theta), v*sin(theta)].
  vtkCustomQuaternion<T> UnitExp() const;

  // Description:
  // Normalize a quaternion in place and transform it to
  // so its angle is in degrees and its axis normalized.
  void NormalizeWithAngleInDegrees();

  // Description:
  // Returns a quaternion normalized and transformed
  // so its angle is in degrees and its axis normalized.
  vtkCustomQuaternion<T> NormalizedWithAngleInDegrees() const;

  // Description:
  // Set/Get the w, x, y and z components of the quaternion.
  void Set(const T& w, const T& x, const T& y, const T& z);
  void Set(T quat[4]);
  void Get(T quat[4]) const;

  // Description:
  // Set/Get the w component of the quaternion, i.e. element 0.
  void SetW(const T& w);
  const T& GetW() const;

  // Description:
  // Set/Get the x component of the quaternion, i.e. element 1.
  void SetX(const T& x);
  const T& GetX() const;

  // Description:
  // Set/Get the y component of the quaternion, i.e. element 2.
  void SetY(const T& y);
  const T& GetY() const;

  // Description:
  // Set/Get the y component of the quaternion, i.e. element 3.
  void SetZ(const T& z);
  const T& GetZ() const;

  // Description:
  // Set/Get the angle (in radians) and the axis corresponding to
  // the axis-angle rotation of this quaternion.
  T GetRotationAngleAndAxis(T axis[3]) const;
  void SetRotationAngleAndAxis(T angle, T axis[3]);
  void SetRotationAngleAndAxis(
    const T& angle, const T& x, const T& y, const T& z);

  // Description:
  // Cast the quaternion to the specified type and return the result.
  template<typename CastTo> vtkCustomQuaternion<CastTo> Cast() const;

  // Description:
  // Convert a quaternion to a 3x3 rotation matrix. The quaternion
  // does not have to be normalized beforehand.
  // @sa FromMatrix3x3()
  void ToMatrix3x3(T A[3][3]) const;

  // Description:
  // Convert a 3x3 matrix into a quaternion.  This will provide the
  // best possible answer even if the matrix is not a pure rotation matrix.
  // The method used is that of B.K.P. Horn.
  // @sa ToMatrix3x3()
  void FromMatrix3x3(const T A[3][3]);

  // Description:
  // Interpolate quaternions using spherical linear interpolation between
  // this quaternion and q1 to produce the output.
  // The parametric coordinate t belongs to [0,1] and lies between (this,q1).
  // @sa vtkCustomQuaternionInterpolator
  vtkCustomQuaternion<T> Slerp(T t, const vtkCustomQuaternion<T>& q) const;

  // Description:
  // Interpolates between quaternions, using spherical quadrangle
  // interpolation.
  // @sa vtkCustomQuaternionInterpolator
  vtkCustomQuaternion<T> InnerPoint(const vtkCustomQuaternion<T>& q1,
    const vtkCustomQuaternion<T>& q2) const;

  // Description:
  // Performs addition of quaternion of the same basic type.
  vtkCustomQuaternion<T> operator+(const vtkCustomQuaternion<T>& q) const;

  // Description:
  // Performs subtraction of quaternions of the same basic type.
  vtkCustomQuaternion<T> operator-(const vtkCustomQuaternion<T>& q) const;

  // Description:
  // Performs multiplication of quaternion of the same basic type.
  vtkCustomQuaternion<T> operator*(const vtkCustomQuaternion<T>& q) const;

  // Description:
  // Performs multiplication of the quaternions by a scalar value.
  vtkCustomQuaternion<T> operator*(const T& scalar) const;

  // Description:
  // Performs in place multiplication of the quaternions by a scalar value.
  void operator*=(const T& scalar) const;

  // Description:
  // Performs division of quaternions of the same type.
  vtkCustomQuaternion<T> operator/(const vtkCustomQuaternion<T>& q) const;

  // Description:
  // Performs division of the quaternions by a scalar value.
  vtkCustomQuaternion<T> operator/(const T& scalar) const;

  // Description:
  // Performs in place division of the quaternions by a scalar value.
  void operator/=(const T& scalar);
};

// Description:
// Several macros to define the various operator overloads for the quaternions.
// These are necessary for the derived classes that are commonly used.
#define vtkCustomQuaternionIdentity(quaternionType, type) \
quaternionType Identity() const \
{ \
  return quaternionType(vtkCustomQuaternion<type>::Identity().GetData()); \
}
#define vtkCustomQuaternionNormalized(quaternionType, type) \
quaternionType Normalized() const \
{ \
  return quaternionType(vtkCustomQuaternion<type>::Normalized().GetData()); \
}
#define vtkCustomQuaternionConjugated(quaternionType, type) \
quaternionType Conjugated() const \
{ \
  return quaternionType(vtkCustomQuaternion<type>::Conjugated().GetData()); \
}
#define vtkCustomQuaternionInverse(quaternionType, type) \
quaternionType Inverse() const \
{ \
  return quaternionType(vtkCustomQuaternion<type>::Inverse().GetData()); \
}
#define vtkCustomQuaternionUnitLog(quaternionType, type) \
quaternionType UnitLog() const \
{ \
  return quaternionType( \
    vtkCustomQuaternion<type>::UnitLog().GetData()); \
}
#define vtkCustomQuaternionUnitExp(quaternionType, type) \
quaternionType UnitExp() const \
{ \
  return quaternionType( \
    vtkCustomQuaternion<type>::UnitExp().GetData()); \
}
#define vtkCustomQuaternionNormalizedWithAngleInDegrees(quaternionType, type) \
quaternionType NormalizedWithAngleInDegrees() const \
{ \
  return quaternionType( \
    vtkCustomQuaternion<type>::NormalizedWithAngleInDegrees().GetData()); \
}
#define vtkCustomQuaternionSlerp(quaternionType, type) \
quaternionType Slerp(type t, const quaternionType& q) const \
{ \
  return quaternionType( \
    vtkCustomQuaternion<type>::Slerp(t, q).GetData()); \
}
#define vtkCustomQuaternionInnerPoint(quaternionType, type) \
quaternionType InnerPoint(const quaternionType& q1, \
                          const quaternionType& q2) const \
{ \
  return quaternionType( \
    vtkCustomQuaternion<type>::InnerPoint(q1, q2).GetData()); \
}
#define vtkCustomQuaternionOperatorPlus(quaternionType, type) \
inline quaternionType operator+(const quaternionType& q) const \
{ \
  return quaternionType( ( \
    static_cast< vtkCustomQuaternion<type> > (*this) + \
    static_cast< vtkCustomQuaternion<type> > (q)).GetData()); \
}
#define vtkCustomQuaternionOperatorMinus(quaternionType, type) \
inline quaternionType operator-(const quaternionType& q) const \
{ \
  return quaternionType( ( \
    static_cast< vtkCustomQuaternion<type> > (*this) - \
    static_cast< vtkCustomQuaternion<type> > (q)).GetData()); \
}
#define vtkCustomQuaternionOperatorMultiply(quaternionType, type) \
inline quaternionType operator*(const quaternionType& q) const \
{ \
  return quaternionType( ( \
    static_cast< vtkCustomQuaternion<type> > (*this) * \
    static_cast< vtkCustomQuaternion<type> > (q)).GetData()); \
}
#define vtkCustomQuaternionOperatorMultiplyScalar(quaternionType, type) \
inline quaternionType operator*(const type& scalar) const \
{ \
  return quaternionType( ( \
    static_cast< vtkCustomQuaternion<type> > (*this) * \
    scalar).GetData()); \
}
#define vtkCustomQuaternionOperatorDivide(quaternionType, type) \
inline quaternionType operator/(const quaternionType& q) const \
{ \
  return quaternionType( ( \
    static_cast< vtkCustomQuaternion<type> > (*this) / \
    static_cast< vtkCustomQuaternion<type> > (q)).GetData()); \
}
#define vtkCustomQuaternionOperatorDivideScalar(quaternionType, type) \
inline quaternionType operator/(const type& scalar) const \
{ \
  return quaternionType( ( \
    static_cast< vtkCustomQuaternion<type> > (*this) / \
    scalar).GetData()); \
}

#define vtkCustomQuaternionOperatorMacro(quaternionType, type) \
vtkCustomQuaternionIdentity(quaternionType, type) \
vtkCustomQuaternionNormalized(quaternionType, type) \
vtkCustomQuaternionConjugated(quaternionType, type) \
vtkCustomQuaternionInverse(quaternionType, type) \
vtkCustomQuaternionUnitLog(quaternionType, type) \
vtkCustomQuaternionUnitExp(quaternionType, type) \
vtkCustomQuaternionNormalizedWithAngleInDegrees(quaternionType, type) \
vtkCustomQuaternionSlerp(quaternionType, type) \
vtkCustomQuaternionInnerPoint(quaternionType, type) \
vtkCustomQuaternionOperatorPlus(quaternionType, type) \
vtkCustomQuaternionOperatorMinus(quaternionType, type) \
vtkCustomQuaternionOperatorMultiply(quaternionType, type) \
vtkCustomQuaternionOperatorMultiplyScalar(quaternionType, type) \
vtkCustomQuaternionOperatorDivide(quaternionType, type) \
vtkCustomQuaternionOperatorDivideScalar(quaternionType, type)

// .NAME vtkCustomQuaternionf - Float quaternion type.
//
// .SECTION Description
// This class is uses vtkCustomQuaternion with float type data.
// For futher description, see the templated class vtkCustomQuaternion.
// @sa vtkCustomQuaterniond vtkCustomQuaternion
class vtkCustomQuaternionf : public vtkCustomQuaternion<float>
{
public:
  vtkCustomQuaternionf() {}
  explicit vtkCustomQuaternionf(float w, float x, float y, float z)
    : vtkCustomQuaternion<float>(w, x, y, z) {}
  explicit vtkCustomQuaternionf(float scalar) : vtkCustomQuaternion<float>(scalar) {}
  explicit vtkCustomQuaternionf(const float *init) : vtkCustomQuaternion<float>(init) {}
  vtkCustomQuaternionOperatorMacro(vtkCustomQuaternionf, float)
};

// .NAME vtkCustomQuaterniond - Double quaternion type.
//
// .SECTION Description
// This class is uses vtkCustomQuaternion with double type data.
// For futher description, seethe templated class vtkCustomQuaternion.
// @sa vtkCustomQuaternionf vtkCustomQuaternion
class vtkCustomQuaterniond : public vtkCustomQuaternion<double>
{
public:
  vtkCustomQuaterniond() {}
  explicit vtkCustomQuaterniond(double w, double x, double y, double z)
    : vtkCustomQuaternion<double>(w, x, y, z) {}
  explicit vtkCustomQuaterniond(double scalar) : vtkCustomQuaternion<double>(scalar) {}
  explicit vtkCustomQuaterniond(const double *init) : vtkCustomQuaternion<double>(init) {}
  vtkCustomQuaternionOperatorMacro(vtkCustomQuaterniond, double);
};

#include "vtkCustomQuaternion.txx"

#endif // vtkQuaternion_h
// VTK-HeaderTest-Exclude: vtkQuaternion.h
