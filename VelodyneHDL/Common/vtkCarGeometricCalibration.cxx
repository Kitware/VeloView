#include "vtkCarGeometricCalibration.h"

#include <cmath>
#include <numeric>
#include <random>
#include <iostream>
#include <fstream>

#include <vtkMath.h>

#include <Eigen/Geometry>

#include "vtkConversions.h"
#include "vtkEigenTools.h"

Eigen::Vector3d GetXYZ(const vtkSmartPointer<vtkVelodyneTransformInterpolator> trajectory,
                       double time)
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  trajectory->InterpolateTransform(time, transform);
  return PositionVectorFromTransform(transform);
}

Eigen::Matrix3d GetR(const vtkSmartPointer<vtkVelodyneTransformInterpolator> trajectory,
                     double time)
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  trajectory->InterpolateTransform(time, transform);
  return RotationMatrixFromTransform(transform);
}

// method to identify a directions in straight lines
enum class DIRECTION_METHOD
{
  TWO_POINTS,
};

// method to identify the normal of the plane containing the turn
enum class NORMAL_METHOD
{
  CROSS_PRODUCT,
};

// method to optimize the direction after the normal has been computed
enum class DIRECTION_OPTIMIZATION_METHOD
{
  NONE,
};

// method to compute an average orientation
// in a zone where the orientation should be constant
// (but could contain noise)
enum class AVERAGE_ORIENTATION_METHOD
{
  SINGLE_POINT
};

std::vector<std::vector<double>> ComputeTurns(
        const vtkSmartPointer<vtkVelodyneTransformInterpolator> poseTrajectory,
        double timeWindow,
        double curveTreshold,
        bool emptyIntersection,
        std::string debugCSV,
        bool verbose
        )
{
  int minPtsInDirectionSets = 3; // could be as low as 2
  int minPtsInTurn = 3; // could be as low as 1
  std::ofstream debugFile;
  bool debugFileEnabled = !debugCSV.empty();
  if (debugFileEnabled)
  {
    debugFile.open(debugCSV);
  }
  // Compute local curvature, in rad/second
  // for each sample: is it inside a turn ?
  // Using the curveTreshold treshold
  std::vector<int> sampleId =
    std::vector<int>();
  sampleId.reserve(poseTrajectory->GetNumberOfTransforms());
  std::vector<double> sampleTime =
    std::vector<double>();
  sampleTime.reserve(poseTrajectory->GetNumberOfTransforms());
  std::vector<bool> sampleStatus =
    std::vector<bool>();
  sampleStatus.reserve(poseTrajectory->GetNumberOfTransforms());

  if (verbose)
  {
    std::cout << "Computing per-sample status for "
              << poseTrajectory->GetNumberOfTransforms()
              << " samples" << std::endl;
  }
  for (int i = 0; i < poseTrajectory->GetNumberOfTransforms(); i++)
  {
    vtkSmartPointer<vtkTransform> sample = vtkSmartPointer<vtkTransform>::New();
    double t;
    poseTrajectory->GetSample(i, sample, t);
    if (t - 0.5 * timeWindow < poseTrajectory->GetMinimumT()
        || t + 0.5 * timeWindow > poseTrajectory->GetMaximumT())
    {
      continue;
    }
    sampleId.push_back(i);
    sampleTime.push_back(t);

    vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
    poseTrajectory->InterpolateTransform(t - 0.5 * timeWindow, prev);
    vtkSmartPointer<vtkTransform> next = vtkSmartPointer<vtkTransform>::New();
    poseTrajectory->InterpolateTransform(t + 0.5 * timeWindow, next);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(RotationMatrixFromTransform(next)
		    * RotationMatrixFromTransform(prev).transpose());
    double curvature = std::abs(aa.angle()) / timeWindow;
    sampleStatus.push_back(curvature >= curveTreshold);
  }

  if (verbose)
  {
    std::cout << "Aggregating samples" << std::endl;
  }
  // Compute turn limits (exprimed in samples indexes)
  unsigned int i = 0; // ! warning that is not the id of a sample
  // if the first samples are inside a turn, discard this turn
  // and start searching turns after its end.
  while (i < sampleStatus.size() && sampleStatus[i] == true)
  {
    i = i + 1;
  }
  // we are ready to search turns and know that sample i
  // is not inside a turn
  bool inside = false;

  // turn_limits[n][0] will contain the time of the first sample inside turn n
  // turn_limits[n][1] will contain the time of the last sample inside turn n
  // We have turn_limits[n][0] <= turn_limits[n][1]
  std::vector<std::vector<int>> turnLimits = std::vector<std::vector<int>>();
  for (unsigned int j = i; j < sampleStatus.size(); j++)
  {
    if (!inside && sampleStatus[j])
    {
      // entering turn
      inside = true;
      turnLimits.push_back(std::vector<int>());
      turnLimits[turnLimits.size() - 1].push_back(j);
    }
    else if (inside && !sampleStatus[j])
    {
      // leaving turn
      inside = false;
      turnLimits[turnLimits.size() - 1].push_back(j - 1);
    }
  }
  // if the end of last turn is not seen, discard this turn
  if (inside)
  {
    if (verbose)
    {
      std::cout << "pop_back" << std::endl;
    }
    turnLimits.pop_back();
  }

  // convert indexes to times:
  if (verbose)
  {
    std::cout << "Converting indexes to time" << std::endl;
  }
  std::vector<std::vector<double>> turnTimes = std::vector<std::vector<double>>();

  for (unsigned int i = 0; i < turnLimits.size(); i++) {
    if (turnLimits[i][1] - turnLimits[i][0] + 1 < minPtsInTurn)
    {
      if (verbose)
      {
        std::cout << "warning: skipped one turn that did not contain enough points" << std::endl;
      }
      continue;
    }
    std::vector<double> times = std::vector<double>(2);
    times[0] = sampleTime[turnLimits[i][0]];
    times[1] = sampleTime[turnLimits[i][1]];
    turnTimes.push_back(times);
  }


  // Now we want to get a direction vector before and after the curve,
  // if that is possible.
  // ! we cannot use the time to look N seconds before/after,
  // because turns are ofen preceded by a traffic light
  // which can be red, in this case there is a stop in the trajectory,
  // right before the turn. Sometimes there is a stop
  // just at the end of a turn if there is a traffic jam.
  // In both cases the risk is that the set of points selected with time
  // before/after the turn will be just 1 point,
  // or will be a set of points very centered locally.
  // (So this set of points will not permit a reliable estimation of
  // a direction vector)

  // precompute a way to tell the length of the curve at any time
  // (will be used by dichotomy search)
  if (verbose)
  {
    std::cout << "Precomputing curve length" << std::endl;
  }
  double currentLength = 0.0;
  std::vector<double> length = std::vector<double>();
  length.reserve(poseTrajectory->GetNumberOfTransforms());
  std::vector<double> lengthTimes = std::vector<double>();
  lengthTimes.reserve(poseTrajectory->GetNumberOfTransforms());
  vtkSmartPointer<vtkTransform> sample = vtkSmartPointer<vtkTransform>::New();
  double time;
  poseTrajectory->GetSample(0, sample, time);
  Eigen::Vector3d previousPos = PositionVectorFromTransform(sample);
  for (int i = 0; i < poseTrajectory->GetNumberOfTransforms(); i++)
  {
    poseTrajectory->GetSample(i, sample, time);
    Eigen::Vector3d pos = PositionVectorFromTransform(sample);
    currentLength += (pos - previousPos).norm();
    lengthTimes.push_back(time);
    length.push_back(currentLength);
    previousPos = pos;
  }

  if (verbose)
  {
    std::cout << "Placing bounds" << std::endl;
  }
  std::vector<std::vector<double>> bounds = std::vector<std::vector<double>>();
  for (unsigned int i = 0; i < turnTimes.size(); i++)
  {
    int iStart = std::distance(lengthTimes.begin(),
		    std::lower_bound(lengthTimes.begin(), lengthTimes.end(), turnTimes[i][0]));
    int iEnd = std::distance(lengthTimes.begin(),
		    std::lower_bound(lengthTimes.begin(), lengthTimes.end(), turnTimes[i][1]));
    double turnLength = length[iEnd] - length[iStart];
    if (verbose)
    {
      std::cout << "turn length is: " << turnLength << " (scale unknown)" << std::endl;
    }
    int iBefore = std::distance(length.begin(),
		    std::lower_bound(length.begin(), length.end(), length[iStart] - turnLength));
    int iAfter = std::distance(length.begin(),
		    std::lower_bound(length.begin(), length.end(), length[iEnd] + turnLength));
    // if it was not possible to find iBefore/iAfter
    // it means that the turn is either at the very start
    // or very end of the trajectory, so it is hard to tell which
    // was the direction before the turn
    if (iBefore == 0 || iAfter == static_cast<int>(length.size()))
    {
      continue;
    }
    // if there is not enough points in sets "before" or "after",
    // discard this turn:
    if (iStart - iBefore + 1 < minPtsInDirectionSets
        || iAfter - iEnd + 1 < minPtsInDirectionSets)
    {
      continue;
    }
    std::vector<double> bound = std::vector<double>(4);
    bound[0] = lengthTimes[iBefore];
    bound[1] = turnTimes[i][0];
    bound[2] = turnTimes[i][1];
    bound[3] = lengthTimes[iAfter];
    bounds.push_back(bound);
  }

  // look for bug in my algo/my code/the data:
  std::vector<int> misbounded = std::vector<int>();
  for (unsigned int i = 0; i < bounds.size(); i++)
  {
    if (!(bounds[i][0] < bounds[i][1]
          && bounds[i][1] < bounds[i][2]
          && bounds[i][2] < bounds[i][3]))
    {
      if (verbose)
      {
        std::cout << "warning: turn " << i
          << "(" << bounds[i][0] << ", "
          << bounds[i][1] << ", "
          << bounds[i][2] << ", "
          << bounds[i][3] << ") is not bounded correctly, discarding it."
          << std::endl;
      }
      misbounded.push_back(i);
    }
  }
  // delete misbounded turns:
  if (verbose)
  {
    std::cout << "Discarding mis-bounded turns" << std::endl;
  }
  for (int i = misbounded.size() - 1; i >= 0; i--)
  {
    bounds.erase(bounds.begin() + misbounded[i]);
  }

  // handle the cases where two turns are close
  // if possible, make the bounds intersection empty
  // this is not mandatory but it help visualization a lot
  if (emptyIntersection)
  {
    if (verbose)
    {
      std::cout << "Changing bounds to empty intersections" << std::endl;
    }
    std::vector<int> mixedUpTurns = std::vector<int>();
    for (unsigned int i = 1; i < bounds.size(); i++)
    {
      if (bounds[i - 1][3] > bounds[i][0])
      {
        double newLimit = 0.5 * (bounds[i - 1][3] + bounds[i][0]);
        // check if this new_limit solves the problem:
        if (newLimit <= bounds[i - 1][2])
        {
          // cannot solve easily, discard the first turn
          if (verbose)
          {
          std::cout << "warning: could not fix (first) turn " << i - 1
            << " discarding it" << std::endl;
          }
          mixedUpTurns.push_back(i - 1);
        }
        else if (newLimit >= bounds[i][1])
        {
          // cannot solve easily, discard the second turn
          if (verbose)
          {
            std::cout << "warning: could not fix (second) turn " << i
              << " discarding it" << std::endl;
          }
          mixedUpTurns.push_back(i);
        }
        else
        {
          // one could argue that we should not do this fix
          // because the lengths of the curve in the new after/before
          // parts will not be equal to the lengths inside the turns
          if (verbose)
          {
            std::cout << "warning: fixing bounds of turns " << i - 1
              << " and " << i << " that are touching each other" << std::endl;
          }
          bounds[i - 1][3] = newLimit;
          bounds[i][0] = newLimit;
        }
      }
    }

    if (verbose)
    {
      std::cout << "Discarding turns with no empty intersection" << std::endl;
    }
    // delete turns whose bounds intersection could not be made empty:
    for (int i = mixedUpTurns.size() - 1; i >= 0; i--)
    {
      bounds.erase(bounds.begin() + mixedUpTurns[i]);
    }
  }

  // write debug file:
  // may not work correctly if emptyIntersection != true
  if (debugFileEnabled)
  {
    if (verbose)
    {
      std::cout << "Writing debug file" << std::endl;
    }
    int currentTurnToConsider = 0;
    for (int i = 0; i < poseTrajectory->GetNumberOfTransforms(); i++)
    {
      vtkSmartPointer<vtkTransform> sample = vtkSmartPointer<vtkTransform>::New();
      double time;
      poseTrajectory->GetSample(i, sample, time);
      double status;
      if (currentTurnToConsider == static_cast<int>(bounds.size()))
      {
        status = 0.0; // will stay outside from now on
      }
      else if (time < bounds[currentTurnToConsider][0])
      {
        status = 0.0; // before
      }
      else if (time < bounds[currentTurnToConsider][1])
      {
        status = 0.4; // inside "before" set of samples
      }
      else if (time < bounds[currentTurnToConsider][2])
      {
        status = 1.0; // inside turn samples
      }
      else if (time < bounds[currentTurnToConsider][3])
      {
        status = 0.2; // inside "after" set of samples
      }
      else
      {
        status = 0.0; // after
        currentTurnToConsider = currentTurnToConsider + 1;
      }
      if (debugFileEnabled)
      {
        debugFile << PositionVectorFromTransform(sample)[0]
                  << "," << PositionVectorFromTransform(sample)[1]
                  << "," << PositionVectorFromTransform(sample)[2]
                  << "," << status << std::endl;
      }
    }
  }


  if (debugFileEnabled)
  {
    if (verbose)
    {
      std::cout << "Closing debug file" << std::endl;
    }
    debugFile.close();
  }

  return bounds;
}

bool ProcessTurn(
        const vtkSmartPointer<vtkVelodyneTransformInterpolator> reference,
        const vtkSmartPointer<vtkVelodyneTransformInterpolator> aligned,
        double t0, double t1, double t2, double t3,
        DIRECTION_METHOD directionMethod,
        NORMAL_METHOD normalMethod,
        DIRECTION_OPTIMIZATION_METHOD normalBasedDirectionOptimisation,
        AVERAGE_ORIENTATION_METHOD orientationMethod,
        Eigen::Matrix3d& rotationBefore, // 1st result
        Eigen::Matrix3d& rotationAfter, // 2nd result
        double& scaleBefore,
        double& scaleAfter
        )
{
  if (!(t0 < t1 && t1 < t2 && t2 < t3))
  {
    std::cerr << "WARNING: times not valid in ProcessTurn(). Bug in implementation" << std::endl;
    return false; // rotationBefore/After are not valid
  }

  Eigen::Vector3d vBeforeReference = GetXYZ(reference, t1) - GetXYZ(reference, t0);
  Eigen::Vector3d vAfterReference = GetXYZ(reference, t3) - GetXYZ(reference, t2);
  Eigen::Vector3d vBeforeAligned = GetXYZ(aligned, t1) - GetXYZ(aligned, t0);
  Eigen::Vector3d vAfterAligned = GetXYZ(aligned, t3) - GetXYZ(aligned, t2);

  scaleBefore = vBeforeAligned.norm() / vBeforeReference.norm();
  scaleAfter = vAfterAligned.norm() / vAfterReference.norm();

  // depending on options, these values will be recomputed later
  // using a more robust method
  Eigen::Vector3d directionBeforeReference = vBeforeReference.normalized();
  Eigen::Vector3d directionAfterReference = vAfterReference.normalized();
  Eigen::Vector3d directionBeforeAligned = vBeforeAligned.normalized();
  Eigen::Vector3d directionAfterAligned = vAfterAligned.normalized();

  if (directionMethod == DIRECTION_METHOD::TWO_POINTS)
  {
    // no need to recompute direction{Before,After}{Reference,Aligned}
  }
  else
  {
    std::cerr << "WARNING: unimplemented option passed to ProcessTurn()" << std::endl;
    return false; // rotationBefore/After are not valid
  }

  // compute normal to plane containing the trajectory
  Eigen::Vector3d normalOrientationReference;
  Eigen::Vector3d normalOrientationAligned;
  if (normalMethod == NORMAL_METHOD::CROSS_PRODUCT)
  {
    normalOrientationReference = directionBeforeReference.cross(directionAfterReference);
    normalOrientationAligned = directionBeforeAligned.cross(directionAfterAligned);
  }
  else
  {
    std::cerr << "WARNING: unimplemented option passed to ProcessTurn()" << std::endl;
    return false; // rotationBefore/After are not valid
  }

  if (normalBasedDirectionOptimisation == DIRECTION_OPTIMIZATION_METHOD::NONE)
  {
    // nothing to do
  }
  else
  {
    std::cerr << "WARNING: unimplemented option passed to ProcessTurn()" << std::endl;
    return false; // rotationBefore/After are not valid
  }

  // compute orientation of trajectories
  Eigen::Matrix3d trajectoryOrientationBeforeReference;
  // operator "<<" stacks the vectors horizontally
  // (Eigen vectors are vertical i.e. single-column matrices)
  trajectoryOrientationBeforeReference <<
      directionBeforeReference,
      normalOrientationReference,
      directionBeforeReference.cross(normalOrientationReference);

  Eigen::Matrix3d trajectoryOrientationAfterReference;
  trajectoryOrientationAfterReference <<
      directionAfterReference,
      normalOrientationReference,
      directionAfterReference.cross(normalOrientationReference);

  Eigen::Matrix3d trajectoryOrientationBeforeAligned;
  trajectoryOrientationBeforeAligned <<
      directionBeforeAligned,
      normalOrientationAligned,
      directionBeforeAligned.cross(normalOrientationAligned);

  Eigen::Matrix3d trajectoryOrientationAfterAligned;
  trajectoryOrientationAfterAligned <<
      directionAfterAligned,
      normalOrientationAligned,
      directionAfterAligned.cross(normalOrientationAligned);

  Eigen::Matrix3d RRemapBefore = trajectoryOrientationBeforeReference
      * trajectoryOrientationBeforeAligned.transpose();

  Eigen::Matrix3d RRemapAfter = trajectoryOrientationAfterReference
      * trajectoryOrientationAfterAligned.transpose();

  Eigen::Matrix3d orientationBeforeReference;
  Eigen::Matrix3d orientationAfterReference;
  Eigen::Matrix3d orientationBeforeAligned;
  Eigen::Matrix3d orientationAfterAligned;
  // compute remapped orientation of sensors
  if (orientationMethod == AVERAGE_ORIENTATION_METHOD::SINGLE_POINT)
  {
        orientationBeforeReference = GetR(reference, t0);
        orientationAfterReference = GetR(reference, t3);
        orientationBeforeAligned = GetR(aligned, t0);
        orientationAfterAligned = GetR(aligned, t3);
  }
  else
  {
    std::cerr << "WARNING: unimplemented option passed to ProcessTurn()" << std::endl;
    return false; // rotationBefore/After are not valid
  }

  // compute rotation between sensors
  // so that R_rotation * orientation_aligned = orientation_reference
  rotationBefore = orientationBeforeReference.transpose()
      * RRemapBefore * orientationBeforeAligned;
  rotationAfter = orientationAfterReference.transpose()
      * RRemapAfter * orientationAfterAligned;
  return true; // rotations are valid
}

bool ProcessTurn(
        const vtkSmartPointer<vtkVelodyneTransformInterpolator> reference,
        const vtkSmartPointer<vtkVelodyneTransformInterpolator> aligned,
        double t0, double t1, double t2, double t3,
        Eigen::Matrix3d& rotationBefore, // 1st result
        Eigen::Matrix3d& rotationAfter, // 2nd result
        double scaleBefore,
        double scaleAfter
        )
{
  return ProcessTurn(
        reference,
        aligned,
        t0, t1, t2, t3,
        DIRECTION_METHOD::TWO_POINTS,
        NORMAL_METHOD::CROSS_PRODUCT,
        DIRECTION_OPTIMIZATION_METHOD::NONE,
        AVERAGE_ORIENTATION_METHOD::SINGLE_POINT,
        rotationBefore,
        rotationAfter,
        scaleBefore,
        scaleAfter
        );
}

enum class ROTATION_ESTIMATOR
{
  ESTIMATOR_UNDEFINED = 0,
  ESTIMATOR_L2_CHORDAL_SVD = 0
};

void estimateL2ChordalSVD(const std::vector<Eigen::Matrix3d>& rotations,
                    Eigen::Matrix3d& result,
                    bool& valid)
{
  if (rotations.size() == 0)
  {
    result = Eigen::Matrix3d::Identity();
    valid = false;
  }

  Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
  for (unsigned int i = 0; i < rotations.size(); i++)
  {
    S += rotations[i];
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  // we should have S == U * D * V.transpose();

  Eigen::Matrix3d R = U *  V.transpose(); // not using singular values
  if (R.determinant() < 0.0) // i.e. close to -1
  {
    Eigen::Vector3d D;
    D << 1.0, 1.0, -1.0;
    R = U * D.asDiagonal() * V.transpose();
  }

  result = R;
  valid = true;
}

void RansacRotation(const std::vector<Eigen::Matrix3d>& rotations,
                    ROTATION_ESTIMATOR estimator,
                    int maxIterations,
                    int sampleToEstimate,
                    int sampleToValidate,
                    float maxAngleToFit,
                    Eigen::Matrix3d& result,
                    bool& valid,
                    int& samplesUsed)
{
  std::default_random_engine rng = std::default_random_engine();
  std::vector<int> shuffled(rotations.size());
  std::iota(std::begin(shuffled), std::end(shuffled), 0); //0 is the starting number

  if (estimator != ROTATION_ESTIMATOR::ESTIMATOR_L2_CHORDAL_SVD)
  {
    std::cerr << "Unknown estimator passed to RansacRotation" << std::endl;
    result = Eigen::Matrix3d::Identity();
    valid = false;
    return;
  }
  for (int i = 0; i < maxIterations; i++)
  {
    // re-shuffle
    std::shuffle(std::begin(shuffled), std::end(shuffled), rng);
    std::vector<Eigen::Matrix3d> samples = std::vector<Eigen::Matrix3d>(sampleToEstimate);
    for (int j = 0; j < sampleToEstimate; j++)
    {
      samples[j] = rotations[shuffled[j]];
    }

    Eigen::Matrix3d estimation;
    bool estimationValid = false;
    estimateL2ChordalSVD(samples, estimation, estimationValid);
    if (!estimationValid)
    {
      continue;
    }

    // count how many rotations are close enough:
    std::vector<Eigen::Matrix3d> samplesFitting = std::vector<Eigen::Matrix3d>();
    samplesFitting.reserve(rotations.size());
    for (unsigned int j = 0; j < rotations.size(); j++)
    {
      double angle = Eigen::AngleAxisd(estimation.transpose() * rotations[j]).angle();
      if (angle <= maxAngleToFit)
      {
        samplesFitting.push_back(rotations[j]);
      }
    }

    if (static_cast<int>(samplesFitting.size()) >= sampleToValidate)
    {
      // do a final estimation, then return
      samplesUsed = samplesFitting.size();
      Eigen::Matrix3d estimation;
      bool estimationValid = false;
      estimateL2ChordalSVD(samplesFitting, estimation, estimationValid);
      result = estimation;
      valid = estimationValid;
      return;
    }
  }

  result = Eigen::Matrix3d::Identity();
  valid = false;
}

void ComputeCarCalibrationRotationScale(
        const vtkSmartPointer<vtkTemporalTransforms> reference,
        const vtkSmartPointer<vtkTemporalTransforms> aligned,
        double curveTreshold,
        int ransacMaxIter,
        double ransacMaxAngleToFit,
        double ransacFittingRatio,
        double ransacValidationRatio,
        DIRECTION_METHOD directionMethod,
        NORMAL_METHOD normalMethod,
        DIRECTION_OPTIMIZATION_METHOD normalBasedDirectionOptimisation,
        AVERAGE_ORIENTATION_METHOD orientationMethod,
        Eigen::Matrix3d& result,
        double& scale,
        bool& validResult,
        bool verbose
        )
{
  vtkSmartPointer<vtkVelodyneTransformInterpolator> referenceI
      = reference->CreateInterpolator();
  referenceI->SetInterpolationTypeToLinear();
  vtkSmartPointer<vtkVelodyneTransformInterpolator> alignedI
      = aligned->CreateInterpolator();
  alignedI->SetInterpolationTypeToLinear();
  if (verbose)
  {
    std::cout << "Detecting turns" << std::endl;
  }
  std::vector<std::vector<double>> turns =
      ComputeTurns(referenceI, 0.8, curveTreshold, false, "", verbose);
  if (verbose)
  {
    std::cout << "Processing " << turns.size() << " turns" << std::endl;
  }

  std::vector<Eigen::Matrix3d> rotations = std::vector<Eigen::Matrix3d>();
  std::vector<double> scales;
  for (unsigned int i = 0; i < turns.size(); i++)
  {
    if (turns[i][0] < alignedI->GetMinimumT()
                    || turns[i][3] > alignedI->GetMaximumT())
    {
      // if this turn (that was seen in "reference")
      // is not contained in aligned, skip it
      continue;
    }
    Eigen::Matrix3d RBefore;
    Eigen::Matrix3d RAfter;
    double scaleBefore, scaleAfter;
    ProcessTurn(referenceI, alignedI,
                turns[i][0], turns[i][1], turns[i][2], turns[i][3],
                    directionMethod,
                    normalMethod,
                    normalBasedDirectionOptimisation,
                    orientationMethod,
                    RBefore,
                    RAfter,
                    scaleBefore,
                    scaleAfter);

    scales.push_back(scaleBefore);
    scales.push_back(scaleAfter);

    Eigen::Vector3d yprBefore = (180.0 / vtkMath::Pi()) * RBefore.eulerAngles(2,1,0);
    Eigen::Vector3d yprAfter = (180.0 / vtkMath::Pi()) * RAfter.eulerAngles(2,1,0);
    if (verbose)
    {
      std::cout << "rotation before: " << yprBefore[2]
                << ", " << yprBefore[1]
                << ", " << yprBefore[0] << std::endl;
      std::cout << "rotation after: " << yprAfter[2]
                << ", " << yprAfter[1]
                << ", " << yprAfter[0] << std::endl;
    }

    if (RBefore.allFinite())
    {
      rotations.push_back((RBefore));
    }
    if (RAfter.allFinite())
    {
      rotations.push_back((RAfter));
    }
  }

  if (verbose)
  {
    // must be displayed before any call of median on the std::vector, else the
    // order is changed
    for (unsigned int i = 0; i < scales.size() / 2; i++)
    {
      std::cout << "scale before: " << scales[2*i]
                << ", scale after: " << scales[2*i+1] << std::endl;
    }
  }

  scale = ComputeMedian(scales);

  if (verbose)
  {
    std::cout << "median of scales is: " << ComputeMedian(scales) << std::endl;
  }


  Eigen::Matrix3d R;
  bool valid;
  int sampleUsed = 0;
  RansacRotation(rotations,
                 ROTATION_ESTIMATOR::ESTIMATOR_L2_CHORDAL_SVD,
                 ransacMaxIter,
                 std::max(1, static_cast<int>(vtkMath::Round(ransacFittingRatio * static_cast<double>(rotations.size())))),
                 std::max(1, static_cast<int>(vtkMath::Round(ransacValidationRatio * static_cast<double>(rotations.size())))),
                 (vtkMath::Pi() / 180.0) * ransacMaxAngleToFit,
                 R,
                 valid,
                 sampleUsed);

  if (verbose)
  {
    if (valid)
    {
      std::cout << "Ransac result is valid" << std::endl;
    }
    else
    {
      std::cout << "Ransac result not valid" << std::endl;
    }
  }
  Eigen::Vector3d rotationYPR = (180.0 / vtkMath::Pi()) * R.eulerAngles(2,1,0);
  if (verbose)
  {
    std::cout << "Rotation found using " << sampleUsed << " samples among " << rotations.size() << ": "
              << rotationYPR[2] << ", " << rotationYPR[1] << ", " << rotationYPR[0] << std::endl;
  }

  if (valid)
  {
    result = R;
  }
  else
  {
    result = Eigen::Matrix3d::Identity();
  }

  validResult = valid;
}

void ComputeCarCalibrationRotationScale(
        const vtkSmartPointer<vtkTemporalTransforms> reference,
        const vtkSmartPointer<vtkTemporalTransforms> aligned,
        double curveTreshold,
        int ransacMaxIter,
        double ransacMaxAngleToFit,
        double ransacFittingRatio,
        double ransacValidationRatio,
        Eigen::Matrix3d& result,
        double& scale,
        bool& validResult,
        bool verbose
        )
{
  ComputeCarCalibrationRotationScale(reference, aligned, curveTreshold,
        ransacMaxIter, ransacMaxAngleToFit,
        ransacFittingRatio, ransacValidationRatio,
        DIRECTION_METHOD::TWO_POINTS,
        NORMAL_METHOD::CROSS_PRODUCT,
        DIRECTION_OPTIMIZATION_METHOD::NONE,
        AVERAGE_ORIENTATION_METHOD::SINGLE_POINT,
        result, scale, validResult, verbose);
}
