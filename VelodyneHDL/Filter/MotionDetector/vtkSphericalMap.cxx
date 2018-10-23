#include "vtkSphericalMap.h"

// VTK
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>
#include <vtkPNGWriter.h>

//----------------------------------------------------------------------------
Gaussian::Gaussian()
{
  this->N = 0;
  this->Sigma = 0.20; // 10 cm accuracy
  this->MaxTTL = 25;
  this->TTL = this->MaxTTL;
}

//----------------------------------------------------------------------------
double Gaussian::operator()(double x)
{
  return 1.0 / (this->Sigma * sqrt(2 * vtkMath::Pi())) * exp(-std::pow(x - this->Mean, 2) / (2.0 * std::pow(this->Sigma, 2)));
}

//----------------------------------------------------------------------------
void Gaussian::SetMean(double mean)
{
  this->Mean = mean;
  this->N = 1;
}

//----------------------------------------------------------------------------
double Gaussian::GetSigma()
{
  return this->Sigma;
}

//----------------------------------------------------------------------------
void Gaussian::UpdateParam(double x)
{
  // update the mean
  double oldMean = this->Mean;
  this->Mean = (static_cast<double>(N) * this->Mean + x) / (static_cast<double>(N) + 1);

  // update the standard deviation
  this->Sigma = std::sqrt((this->N * std::pow(this->Sigma, 2) + (x - oldMean) * (x - this->Mean)) / (this->N + 1));

  // reset the TTL to its maximum
  this->TTL = this->MaxTTL;

  // update the number of sample
  this->N += 1;
}

//----------------------------------------------------------------------------
bool Gaussian::UpdateTTL()
{
  this->TTL -= 1;
  return (this->TTL >= 0);
}

//----------------------------------------------------------------------------
GaussianMixture::GaussianMixture()
{

}

//----------------------------------------------------------------------------
double GaussianMixture::Evaluate(double x)
{
  if (this->Gaussians.size() == 0)
  {
    return 0.0;
  }

  double proba = 0;
  for (std::list<Gaussian>::iterator it = this->Gaussians.begin(); it != this->Gaussians.end(); ++it)
  {
    // divide by the max
    proba = std::max(proba, (*it)(x));
  }

  return proba;
}

//----------------------------------------------------------------------------
double GaussianMixture::operator()(double x)
{
  return 1.0 / (this->sigma * sqrt(2 * vtkMath::Pi())) * exp(-std::pow(x - this->mean, 2) / (2.0 * std::pow(this->sigma, 2)));
}

//----------------------------------------------------------------------------
void GaussianMixture::AddPoint(double x)
{
  double maxProba = 0.0;
  std::list<Gaussian>::iterator itMaxProba;

  // add the point to the right gaussian
  for (std::list<Gaussian>::iterator it = this->Gaussians.begin(); it != this->Gaussians.end(); ++it)
  {
    double proba = (*it)(x);
    if (proba > maxProba)
    {
      maxProba = proba;
      itMaxProba = it;
    }
  }

  double threshProb = 0.00135; // 3 sigma

  // Create new gaussian
  if (maxProba < threshProb)
  {
    // Create new gaussian, centered on x and with a starting
    // sigma of 2cm (specific to velodyne sensors)
    Gaussian newGaussian;
    newGaussian.SetMean(x);
    this->Gaussians.push_back(newGaussian);
    return;
  }

  itMaxProba->UpdateParam(x);
}

//----------------------------------------------------------------------------
void GaussianMixture::UpdateTTL()
{
  std::list<Gaussian>::iterator it = this->Gaussians.begin();
  while (it != this->Gaussians.end())
  {
    // Get if the gaussian is still alive
    bool stillAlive = it->UpdateTTL();

    // erase the gaussian
    if (!stillAlive)
    {
      // erase current iterator and get next on
      it = this->Gaussians.erase(it);
      // no need to increment iterator here
    }
    else
    {
      it++; // increment iterator here
    }
  }
}

//----------------------------------------------------------------------------
unsigned int GaussianMixture::GetNumberOfPoints()
{
  return this->Gaussians.size();
}

//----------------------------------------------------------------------------
vtkSphericalMap::vtkSphericalMap()
{
  // Phi Bounds
  this->PhiBounds[0] = 0;
  this->PhiBounds[1] = vtkMath::Pi();

  // Theta Bounds
  this->ThetaBounds[0] = -vtkMath::Pi();
  this->ThetaBounds[1] =  vtkMath::Pi();

  // Default sensor rpm
  this->SensorRPM = 600.0;

  // Default NPhi / NTheta values
  this->NPhi = 180;
  this->NTheta = static_cast<int>(std::floor(60.0 / (this->SensorRPM * 55.296 * 1e-6))); // around 904

  // Base of R3
  this->ez << 0, 0, 1;
  this->ey << 0, 1, 0;
  this->ex << 1, 0, 0;

  // center
  this->C << 0, 0, 0;

  // reset internal parameters
  this->ResetMap();
}

//----------------------------------------------------------------------------
void vtkSphericalMap::SetSensorRPM(double rpm)
{
  this->SensorRPM = rpm;
  this->NTheta = static_cast<unsigned int>(std::floor(60.0 / (SensorRPM * 55.296 * 1e-6)));
  this->ResetMap();
}

//----------------------------------------------------------------------------
void vtkSphericalMap::SetNPhi(unsigned int phi)
{
  this->NPhi = phi;
  this->ResetMap();
}

//----------------------------------------------------------------------------
unsigned int vtkSphericalMap::GetNPhi()
{
  return this->NPhi;
  this->ResetMap();
}

//----------------------------------------------------------------------------
void vtkSphericalMap::SetNTheta(unsigned int theta)
{
  this->NTheta = theta;
  this->SensorRPM = 60.0 / (static_cast<double>(this->NTheta) * 55.296 * 1e-6);
  this->ResetMap();
}

//----------------------------------------------------------------------------
unsigned int vtkSphericalMap::GetNTheta()
{
  return this->NTheta;
}

//----------------------------------------------------------------------------
void vtkSphericalMap::ResetMap()
{
  // update quantum
  this->dPhi = (this->PhiBounds[1] - this->PhiBounds[0]) / static_cast<double>(this->NPhi);
  this->dTheta = (this->ThetaBounds[1] - this->ThetaBounds[0]) / static_cast<double>(this->NTheta);

  // reset the map
  this->Map.clear();
  this->Map.resize(this->NPhi * this->NTheta);

  // reset internal parameters
  this->AddedFrames = 0;

  // reset export parameters
  this->ShouldExportAsImage = false;
  this->filenameBase = "noFilename";
}

//----------------------------------------------------------------------------
Eigen::Matrix<double, 3, 1> vtkSphericalMap::GetSphericalCoordinates(const Eigen::Matrix<double, 3, 1>& X)
{
  // Express the current point in the local
  // reference frame designed by the internal
  // base and origin
  Eigen::Matrix<double, 3, 3> R;
  R << this->ex(0), this->ey(0), this->ez(0),
       this->ex(1), this->ey(1), this->ez(1),
       this->ex(2), this->ey(2), this->ez(2);
  Eigen::Matrix<double, 3, 1> CX = R.transpose() * (X - C);

  // compute the vetor length
  double r = CX.norm();

  // If the vector is not null
  // we normalized it
  if (r > 1e-4)
  {
    CX.normalize();
  }

  // Project CX onto the (ex, ey) plane
  Eigen::Matrix<double, 3, 1> projCX = CX.dot(this->ex) * this->ex + CX.dot(this->ey) * this->ey;
  projCX.normalize();

  // Compute Phi angle
  double cosPhi = CX.dot(this->ez);
  double sinPhi = CX.cross(this->ez).transpose() * CX.cross(this->ez).normalized();
  double Phi = std::atan2(sinPhi, cosPhi);

  // Compute theta angle
  double cosTheta = projCX.dot(this->ex);
  double sinTheta = projCX.cross(this->ex).dot(this->ey.cross(this->ex).normalized());
  double Theta = std::atan2(sinTheta, cosTheta);

  Eigen::Matrix<double, 3, 1> sphericalCoords;
  sphericalCoords << r, Theta, Phi;
  return sphericalCoords;
}

//----------------------------------------------------------------------------
unsigned int vtkSphericalMap::GetNumberOfPoints()
{
  unsigned int nbrPoints = 0;
  for (unsigned int k = 0; k < this->Map.size(); ++k)
  {
    nbrPoints += this->Map[k].GetNumberOfPoints();
  }

  return nbrPoints;
}

//----------------------------------------------------------------------------
void vtkSphericalMap::AddPoint(unsigned int idxTheta, unsigned int idxPhi, double valueDepth)
{
  if (idxTheta > this->NTheta || idxPhi > this->NPhi)
  {
    std::cout << "Error, required values out of bounds" << std::endl;
    std::cout << "[" << idxTheta << "," << idxPhi << "] / [" << this->NTheta << "," << this->NPhi << "]" << std::endl;
    return;
  }

  // fill the mixture gaussian
  this->Map[idxTheta + this->NTheta * idxPhi].AddPoint(valueDepth);
}

//----------------------------------------------------------------------------
 void vtkSphericalMap::AddFrame(vtkSmartPointer<vtkPolyData> polydata)
 {
   vtkSmartPointer<vtkDoubleArray> Phi = vtkSmartPointer<vtkDoubleArray>::New();
   vtkSmartPointer<vtkDoubleArray> Theta = vtkSmartPointer<vtkDoubleArray>::New();
   vtkSmartPointer<vtkDoubleArray> Motion = vtkSmartPointer<vtkDoubleArray>::New();
   Phi->SetName("Phi");
   Theta->SetName("Theta");
   Motion->SetName("Motion_Probability");

   double point[3];
   Eigen::Matrix<double, 3, 1> cartesianPoint;
   Eigen::Matrix<double, 3, 1> sphericalPoint;

   for (unsigned int k = 0; k < polydata->GetNumberOfPoints(); ++k)
   {
     // Get point and compute its spherical coordinates
     polydata->GetPoint(k, point);
     cartesianPoint << point[0], point[1], point[2];
     sphericalPoint = this->GetSphericalCoordinates(cartesianPoint);

     // Convert spherical coordinates to
     // spherical map coordinates
     unsigned int idxPhi = static_cast<int>(std::floor((sphericalPoint(2) - this->PhiBounds[0]) / this->dPhi));
     unsigned int idxTheta = static_cast<int>(std::floor((sphericalPoint(1) - this->ThetaBounds[0]) / this->dTheta));

     // Evaluate the mixture model on the current data
     // it return the "probability" of the point to be
     // a point in motion
     double proba = this->Map[idxTheta + this->NTheta * idxPhi].Evaluate(sphericalPoint(0));

     // Add the depth to the correct "pixel"
     this->AddPoint(idxTheta, idxPhi, sphericalPoint(0));

     double theta = this->ThetaBounds[0] + idxTheta * this->dTheta;
     double phi = this->PhiBounds[0] + idxPhi * this->dPhi;
     double r = sphericalPoint(0);

     point[0] = r * std::sin(phi) * std::cos(theta);
     point[1] = r * std::sin(phi) * std::sin(theta);
     point[2] = r * std::cos(phi);
     //polydata->GetPoints()->SetPoint(k, point);

     Theta->InsertNextTuple1(sphericalPoint(1) * 180.0 / vtkMath::Pi());
     Phi->InsertNextTuple1(sphericalPoint(2) * 180.0 / vtkMath::Pi());
     Motion->InsertNextTuple1(proba);
   }

   // Time to live of the gaussians
   this->UpdateTTL();

   polydata->GetPointData()->AddArray(Phi);
   polydata->GetPointData()->AddArray(Theta);
   polydata->GetPointData()->AddArray(Motion);

   this->AddedFrames += 1;
 }

 //----------------------------------------------------------------------------
 void vtkSphericalMap::UpdateTTL()
 {
   for (unsigned int k = 0; k < this->Map.size(); ++k)
   {
     this->Map[k].UpdateTTL();
   }
 }
