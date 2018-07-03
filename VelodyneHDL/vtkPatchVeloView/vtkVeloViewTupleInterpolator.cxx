/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTupleInterpolator.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkVeloViewTupleInterpolator.h"
#include "vtkObjectFactory.h"
#include "vtkSpline.h"
#include "vtkKochanekSpline.h"
#include "vtkVeloViewPiecewiseFunction.h"
#include "vtkMath.h"

vtkStandardNewMacro(vtkVeloViewTupleInterpolator);

//----------------------------------------------------------------------------
vtkVeloViewTupleInterpolator::vtkVeloViewTupleInterpolator()
{
  // Set up the interpolation
  this->NumberOfComponents = 0;
  this->InterpolationType = INTERPOLATION_TYPE_SPLINE;
  this->InterpolatingSpline = NULL;

  this->Spline = NULL;
  this->Linear = NULL;
}

//----------------------------------------------------------------------------
vtkVeloViewTupleInterpolator::~vtkVeloViewTupleInterpolator()
{
  this->Initialize();
  if ( this->InterpolatingSpline )
    {
    this->InterpolatingSpline->Delete();
    }
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::SetNumberOfComponents(int numComp)
{
  numComp = (numComp < 1 ? 1 : numComp);
  if ( numComp != this->NumberOfComponents )
    {
    this->Initialize(); //wipe out data
    this->NumberOfComponents = numComp;
    this->InitializeInterpolation();
    this->Modified();
    }
}


//----------------------------------------------------------------------------
int vtkVeloViewTupleInterpolator::GetNumberOfTuples()
{
  if ( this->Spline )
    {
    return this->Spline[0]->GetNumberOfPoints();
    }
  else if ( this->Linear )
    {
    return this->Linear[0]->GetSize();
    }
  else
    {
    return 0;
    }
}


//----------------------------------------------------------------------------
double vtkVeloViewTupleInterpolator::GetMinimumT()
{
  if ( this->Spline )
    {
    double range[2];
    this->Spline[0]->GetParametricRange(range);
    return range[0];
    }
  else if ( this->Linear )
    {
    return this->Linear[0]->GetRange()[0];
    }
  else
    {
    return 0.0;
    }
}


//----------------------------------------------------------------------------
double vtkVeloViewTupleInterpolator::GetMaximumT()
{
  if ( this->Spline )
    {
    double range[2];
    this->Spline[0]->GetParametricRange(range);
    return range[1];
    }
  else if ( this->Linear )
    {
    return this->Linear[0]->GetRange()[1];
    }
  else
    {
    return 1.0;
    }
}


//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::Initialize()
{
  int i;

  // Wipe out old data
  if ( this->Spline )
    {
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Spline[i]->Delete();
      }
    delete [] this->Spline;
    this->Spline = NULL;
    }
  if ( this->Linear )
    {
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Linear[i]->Delete();
      }
    delete [] this->Linear;
    this->Linear = NULL;
    }

  this->NumberOfComponents = 0;
}


//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::InitializeInterpolation()
{
  // Prepare for new data
  if ( this->NumberOfComponents <= 0 )
    {
    return;
    }

  int i;
  if ( this->InterpolationType == INTERPOLATION_TYPE_LINEAR )
    {
    this->Linear = new vtkVeloViewPiecewiseFunction* [this->NumberOfComponents];
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Linear[i] = vtkVeloViewPiecewiseFunction::New();
      }
    }

  else // this->InterpolationType == INTERPOLATION_TYPE_SPLINE
    {
    this->Spline = new vtkSpline* [this->NumberOfComponents];
    if ( ! this->InterpolatingSpline )
      {
      this->InterpolatingSpline = vtkKochanekSpline::New();
      }
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Spline[i] = this->InterpolatingSpline->NewInstance();
      this->Spline[i]->DeepCopy(this->InterpolatingSpline);
      this->Spline[i]->RemoveAllPoints();
      }
    }
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::SetInterpolationType(int type)
{
  type = (type < INTERPOLATION_TYPE_LINEAR ? INTERPOLATION_TYPE_LINEAR :
         (type > INTERPOLATION_TYPE_SPLINE ? INTERPOLATION_TYPE_SPLINE : type));
  if ( type != this->InterpolationType )
    {
    this->Initialize(); //wipe out data
    this->InterpolationType = type;
    this->InitializeInterpolation();
    this->Modified();
    }
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::SetInterpolatingSpline(vtkSpline *spline)
{
  if ( this->InterpolatingSpline == spline )
    {
    return;
    }
  if ( this->InterpolatingSpline )
    {
    this->InterpolatingSpline->UnRegister(this);
    this->InterpolatingSpline = NULL;
    }
  if ( spline )
    {
    spline->Register(this);
    }
  this->InterpolatingSpline = spline;
  this->Modified();
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::FillFromData(int nb, double *t, double **data)
{
  int i;
  if ( this->InterpolationType == INTERPOLATION_TYPE_LINEAR )
    {
    double *ptr = new double[2*nb];
    for (i=0; i<this->NumberOfComponents; i++)
      {
      double *dimData = data[i];
      int ind = 0;
      for(int j = 0; j < 2 * nb; j = j+2)
      {
        ptr[j] = t[ind];
        ptr[j + 1] = dimData[ind]; 
        ind++;
      }
      this->Linear[i]->FillFromDataPointer(nb, ptr);
      }
    delete [] ptr;
    }
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::AddTuple(double t, double tuple[])
{
  int i;
  if ( this->InterpolationType == INTERPOLATION_TYPE_LINEAR )
    {
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Linear[i]->AddPoint(t,tuple[i]);
      }
    }

  else // this->InterpolationType == INTERPOLATION_TYPE_SPLINE
    {
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Spline[i]->AddPoint(t,tuple[i]);
      }
    }

  this->Modified();
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::RemoveTuple(double t)
{
  int i;
  if ( this->InterpolationType == INTERPOLATION_TYPE_LINEAR )
    {
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Linear[i]->RemovePoint(t);
      }
    }

  else // this->InterpolationType == INTERPOLATION_TYPE_SPLINE
    {
    for (i=0; i<this->NumberOfComponents; i++)
      {
      this->Spline[i]->RemovePoint(t);
      }
    }

  this->Modified();
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::InterpolateTuple(double t, double tuple[])
{
  if ( this->NumberOfComponents <= 0 )
    {
    return;
    }

  int i;
  if ( this->InterpolationType == INTERPOLATION_TYPE_LINEAR )
    {
    double *range=this->Linear[0]->GetRange();
    t = (t < range[0] ? range[0] : (t > range[1] ? range[1] : t));
    for (i=0; i<this->NumberOfComponents; i++)
      {
      tuple[i] = this->Linear[i]->GetValue(t);
      }
    }

  else // this->InterpolationType == INTERPOLATION_TYPE_SPLINE
    {
    for (i=0; i<this->NumberOfComponents; i++)
      {
      tuple[i] = this->Spline[i]->Evaluate(t);
      }
    }
}

void vtkVeloViewTupleInterpolator::InterpolateTupleDichotomic(double t, double tuple[])
{
  if ( this->NumberOfComponents <= 0 )
  {
    return;
  }

  int i;
  if ( this->InterpolationType == INTERPOLATION_TYPE_LINEAR )
  {
    double *range=this->Linear[0]->GetRange();
    t = (t < range[0] ? range[0] : (t > range[1] ? range[1] : t));
    for (i=0; i<this->NumberOfComponents; i++)
    {
      tuple[i] = this->Linear[i]->GetValueDichotomic(t);
    }
  }

  else // this->InterpolationType == INTERPOLATION_TYPE_SPLINE
  {
    for (i=0; i<this->NumberOfComponents; i++)
    {
      tuple[i] = this->Spline[i]->Evaluate(t);
    }
  }
}

//----------------------------------------------------------------------------
void vtkVeloViewTupleInterpolator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "There are " << this->GetNumberOfTuples()
     << " tuples to be interpolated\n";

  os << indent << "Number of Components: " << this->NumberOfComponents << "\n";

  os << indent << "Interpolation Type: "
     << (this->InterpolationType == INTERPOLATION_TYPE_LINEAR ?
         "Linear\n" : "Spline\n");

  os << indent << "Interpolating Spline: ";
  if ( this->InterpolatingSpline )
    {
    os << this->InterpolatingSpline << "\n";
    }
  else
    {
    os << "(null)\n";
    }
}



