#include <iostream>
#include <stdlib.h>

#include "TestHelpers.h"
#include "vtkEigenTools.h"

//-----------------------------------------------------------------------------
int TestSignedAngle()
{
  int nbrErrors = 0;
  int Ntests = 250;
  for (int testIndex = 0; testIndex < Ntests; ++testIndex)
  {
    // Generate a random rotation axis
    Eigen::Vector3d rotationAxis(2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5),
                                 2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5),
                                 2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5));
    rotationAxis.normalize();
    // Generate a random angle
    double angle = 2.0 * vtkMath::Pi() * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5);

    // Generate a random 2d vector
    Eigen::Vector2d u1(2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5),
                       2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5));
    // Generate a random angle
    double groundTruthAngle = 2.0 * vtkMath::Pi() * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5);
    Eigen::Matrix2d R2d;
    R2d << std::cos(groundTruthAngle), -std::sin(groundTruthAngle),
           std::sin(groundTruthAngle),  std::cos(groundTruthAngle);
    Eigen::Vector2d u2 = R2d * u1;

    // Finally, create 3d vectors that are
    // obtained by rotating the 2D vectors
    // by embedding them into a plan
    Eigen::Matrix3d R(Eigen::AngleAxisd(angle, rotationAxis));
    Eigen::Vector3d v1 = R * Eigen::Vector3d(u1(0), u1(1), 0);
    Eigen::Vector3d v2 = R * Eigen::Vector3d(u2(0), u2(1), 0);

    double signedAngle = SignedAngle(v1, v2);
    double signedAngle2 = SignedAngle(v2, v1);
    if (std::abs(std::abs(signedAngle) - std::abs(groundTruthAngle)) > 2.0 * std::numeric_limits<double>::epsilon())
    {
      nbrErrors++;
    }
  }
  return nbrErrors;
}

//-----------------------------------------------------------------------------
int TestGetSphericalCoordinates()
{
  int nbrErrors = 0;
  int Ntests = 250;

  for (int testIndex = 0; testIndex < Ntests; ++testIndex)
  {
    // Generate random phi, theta
    double theta = 2.0 * vtkMath::Pi() * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5);
    double phi = vtkMath::Pi() / 2.0 * static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
    // Generate random radius
    double r = 100.0 * static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);

    Eigen::Vector3d Xref1(r * std::cos(theta) * std::sin(phi),
                          r * std::sin(theta) * std::sin(phi),
                          r * std::cos(phi));

    Eigen::Vector3d spheCoord;
    double testMode = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
    if (testMode > 0.5)
    {
      spheCoord = GetSphericalCoordinates(Xref1);
    }
    else
    {
      // Generate a random reference frame change
      Eigen::Vector3d rotationAxis(2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5),
                                   2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5),
                                   2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5));
      rotationAxis.normalize();
      double angle = 2.0 * vtkMath::Pi() * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5);
      Eigen::Matrix3d R(Eigen::AngleAxisd(angle, rotationAxis));
      Eigen::Vector3d T(100.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5),
                        100.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5),
                        100.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 0.5));
      Eigen::Vector3d Xref2 = R * Xref1 + T;
      spheCoord = GetSphericalCoordinates(Xref2, R, T);
    }

    if ((Eigen::Vector3d(r, theta, phi) - spheCoord).norm() > std::numeric_limits<float>::epsilon())
    {
      nbrErrors++;
    }
  }
  return nbrErrors;
}

//-----------------------------------------------------------------------------
int main()
{
  // initialize the random generator to a fixed seed
  // for test repetability
  std::srand(1992);

  int nbrErrors = 0;
  nbrErrors += TestSignedAngle();
  nbrErrors += TestGetSphericalCoordinates();
  return nbrErrors;
}
