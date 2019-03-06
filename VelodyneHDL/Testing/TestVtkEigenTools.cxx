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
int main()
{
  // initialize the random generator to a fixed seed
  // for test repetability
  std::srand(1992);

  int nbrErrors = 0;
  nbrErrors += TestSignedAngle();
  return nbrErrors;
}
