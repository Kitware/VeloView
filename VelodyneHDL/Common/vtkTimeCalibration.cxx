#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

#include <vtkMath.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkTransform.h>
#include <vtkMath.h>

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include "vtkConversions.h"
#include "vtkVelodyneTransformInterpolator.h"
#include "vtkTimeCalibration.h"
#include "statistics.h"
#include "eigenFFTCorrelation.h"
#include "interpolator1D.h"

std::string ToString(CorrelationStrategy correlationStrategy)
{
  switch (correlationStrategy)
  {
    case CorrelationStrategy::DPOS:
      return "dpos";
    case CorrelationStrategy::SPEED_WINDOW:
      return "speed_window";
    case CorrelationStrategy::ACC_WINDOW:
      return "acc_window";
    case CorrelationStrategy::JERK_WINDOW:
      return "jerk_window";
    case CorrelationStrategy::LENGTH:
      return "length";
    case CorrelationStrategy::DERIVATED_LENGTH:
      return "derivated_length";
    case CorrelationStrategy::DROT:
      return "drot";
    case CorrelationStrategy::TRAJECTORY_ANGLE:
      return "trajectory_angle";
    case CorrelationStrategy::ORIENTATION_ANGLE:
      return "orientation_angle";
    case CorrelationStrategy::DERIVATED_ORIENTATION_ARC:
      return "derivated_orientation_arc";
    default:
      return "unkown";
  }
}

Interpolator1D<double> compute_speed_window(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform,
    double window_width)
{
  std::vector<double> times = std::vector<double>();
  std::vector<double> speeds = std::vector<double>();
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> next = vtkSmartPointer<vtkTransform>::New();

  double minMidWindowTime = transform->GetMinimumT() + 0.5 * window_width;
  double maxMidWindowTime = transform->GetMaximumT() - 0.5 * window_width;
  double period = transform->GetPeriod();
  double time = minMidWindowTime;
  while (time < maxMidWindowTime)
  {
    transform->InterpolateTransform(time - 0.5 * window_width, prev);
    transform->InterpolateTransform(time + 0.5 * window_width, next);
    speeds.push_back((PositionVectorFromTransform(next)
			    - PositionVectorFromTransform(prev)).norm() / window_width);
    times.push_back(time);
    time = time + period;
  }

  return Interpolator1D<double>(times, speeds);
}

Interpolator1D<double> compute_acc_window(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform,
    double window_width)
{
  std::vector<double> times = std::vector<double>();
  std::vector<double> accs = std::vector<double>();
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> next = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> curr = vtkSmartPointer<vtkTransform>::New();

  double minMidWindowTime = transform->GetMinimumT() + 0.5 * window_width;
  double maxMidWindowTime = transform->GetMaximumT() - 0.5 * window_width;
  double period = transform->GetPeriod();
  double time = minMidWindowTime;
  while (time < maxMidWindowTime)
  {
    transform->InterpolateTransform(time - 0.5 * window_width, prev);
    transform->InterpolateTransform(time, curr);
    transform->InterpolateTransform(time + 0.5 * window_width, next);
    Eigen::Vector3d a = (PositionVectorFromTransform(next)
		    + PositionVectorFromTransform(prev)
		    - 2 * PositionVectorFromTransform(curr)) / (window_width * window_width);
    accs.push_back(a.norm());
    times.push_back(time);
    time = time + period;
  }

  return Interpolator1D<double>(times, accs);
}

Interpolator1D<double> compute_jerk_window(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform,
    double window_width)
{
  std::vector<double> times = std::vector<double>();
  std::vector<double> jerks = std::vector<double>();
  vtkSmartPointer<vtkTransform> t1 = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> t2 = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> t3 = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> t4 = vtkSmartPointer<vtkTransform>::New();

  double minMidWindowTime = transform->GetMinimumT() + 0.5 * window_width;
  double maxMidWindowTime = transform->GetMaximumT() - 0.5 * window_width;
  double period = transform->GetPeriod();
  double time = minMidWindowTime;
  while (time < maxMidWindowTime)
  {
    transform->InterpolateTransform(time - 0.5 * window_width, t1);
    transform->InterpolateTransform(time + (- 0.5 + 1.0/3.0) * window_width, t2);
    transform->InterpolateTransform(time + (- 0.5 + 2.0/3.0) * window_width, t3);
    transform->InterpolateTransform(time + (- 0.5 + 3.0/3.0) * window_width, t4);
    Eigen::Vector3d j = (PositionVectorFromTransform(t4)
		   - 3 * PositionVectorFromTransform(t3)
		   + 3 * PositionVectorFromTransform(t2)
		   - PositionVectorFromTransform(t1))
	    / std::pow(window_width, 3.0);
    jerks.push_back(j.norm());
    times.push_back(time);
    time = time + period;
  }

  return Interpolator1D<double>(times, jerks);
}

Interpolator1D<double> compute_dPos(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform)
{
  std::vector<std::vector<double>> transforms = transform->GetTransformList();
  std::vector<double> t = std::vector<double>(transforms.size() - 1);
  std::vector<double> x = std::vector<double>(transforms.size() - 1);
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> curr = vtkSmartPointer<vtkTransform>::New();
  for (unsigned int i = 0; i < transforms.size() - 1; i++)
  {
    double t0 = transforms[i][0];
    double t1 = transforms[i+1][0];
    t[i] = 0.5 * (t0 + t1);
    transform->InterpolateTransform(t0, prev);
    transform->InterpolateTransform(t1, curr);
    if (std::abs(t1 - t0) < 0.0001) {
      x[i] = 0.0;
    } else {
      x[i] = (PositionVectorFromTransform(curr)
		      - PositionVectorFromTransform(prev)).norm()
	      / (t1 - t0);
    }
  }
  return Interpolator1D<double>(t, x);
}

Interpolator1D<double> compute_length(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform)
{
  std::vector<std::vector<double>> transforms = transform->GetTransformList();
  std::vector<double> t = std::vector<double>(transforms.size());
  std::vector<double> x = std::vector<double>(transforms.size());
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> curr = vtkSmartPointer<vtkTransform>::New();
  t[0] = transforms[0][0];
  x[0] = 0.0;
  for (unsigned int i = 1; i < transforms.size(); i++)
  {
    t[i] = transforms[i][0];
    transform->InterpolateTransform(t[i-1], prev);
    transform->InterpolateTransform(t[i], curr);
    x[i] = x[i - 1] + (PositionVectorFromTransform(curr)
		    - PositionVectorFromTransform(prev)).norm();
  }

  return Interpolator1D<double>(t, x);
}


Interpolator1D<double> compute_derivated_length(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform,
    double window_width)
{
  Interpolator1D<double> length = compute_length(transform);
  double tMin = transform->GetMinimumT() + 0.5 * window_width;
  double tMax = transform->GetMaximumT() - 0.5 * window_width;
  int steps = (tMax - tMin) / transform->GetPeriod() + 1;
  std::vector<double> derivated_length = std::vector<double>(steps);
  std::vector<double> times = std::vector<double>(steps);
  for (int i = 0; i < steps; i++)
  {
    double time = tMin + i * transform->GetPeriod();
    times[i] = time;
    // length is an interpolator so no need to check that the sample instants
    // are not the same (they are not, even if the interpolation mode of
    // this->Reference/Aligned is "NEAREST")
    derivated_length[i] = (length.Get(time + 0.5 * window_width)
                           - length.Get(time - 0.5 * window_width))
                    / window_width;
  }

  return Interpolator1D<double>(times, derivated_length);
}

Interpolator1D<double> compute_dRot(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform)
{
  std::vector<std::vector<double>> transforms = transform->GetTransformList();
  std::vector<double> t = std::vector<double>(transforms.size() - 1);
  std::vector<double> x = std::vector<double>(transforms.size() - 1);
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> curr = vtkSmartPointer<vtkTransform>::New();
  for (int i = 0; i < static_cast<int>(transforms.size()) - 1; i++)
  {
    double t0 = transforms[i][0];
    double t1 = transforms[i+1][0];
    transform->InterpolateTransform(t0, prev);
    transform->InterpolateTransform(t1, curr);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(RotationMatrixFromTransform(curr)
		    * RotationMatrixFromTransform(prev).transpose());
    t[i] = 0.5 * (t0 + t1);
    if (std::abs(t1 - t0) < 0.0001) {
      x[i] = 0.0;
    } else {
      x[i] = std::abs(aa.angle()) / (t1 - t0);
    }
  }
  return Interpolator1D<double>(t, x);
}

// TODO: handle case where real_sample_time(next) == real_sample_time(prev)
Interpolator1D<double> compute_trajectory_angle(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform,
    double window_width)
{
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> curr = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> next = vtkSmartPointer<vtkTransform>::New();
  double tMin = transform->GetMinimumT() + 0.5 * window_width;
  double tMax = transform->GetMaximumT() - 0.5 * window_width;
  int steps = (tMax - tMin) / transform->GetPeriod() + 1;
  std::vector<double> trajectory_angle = std::vector<double>(steps);
  std::vector<double> times = std::vector<double>(steps);
  for (int i = 0; i < steps; i++)
  {
    double time = tMin + i * transform->GetPeriod();
    transform->InterpolateTransform(time - 0.5 * window_width, prev);
    transform->InterpolateTransform(time, curr);
    transform->InterpolateTransform(time + 0.5 * window_width, next);
    times[i] = time;
    trajectory_angle[i] = SignedAngle(PositionVectorFromTransform(curr)
		    - PositionVectorFromTransform(prev),
		    PositionVectorFromTransform(next)
		    - PositionVectorFromTransform(curr));
  }

  return Interpolator1D<double>(times, trajectory_angle);
}


Interpolator1D<double> compute_orientation_arc(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform)
{
  std::vector<std::vector<double>> transforms = transform->GetTransformList();
  std::vector<double> t = std::vector<double>(transforms.size());
  std::vector<double> x = std::vector<double>(transforms.size());
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> curr = vtkSmartPointer<vtkTransform>::New();
  t[0] = transforms[0][0];
  x[0] = 0.0;
  for (unsigned int i = 1; i < transforms.size(); i++)
  {
    t[i] = transforms[i][0];
    transform->InterpolateTransform(t[i-1], prev);
    transform->InterpolateTransform(t[i], curr);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(RotationMatrixFromTransform(curr)
		    * RotationMatrixFromTransform(prev).transpose());
    x[i] = x[i - 1] + std::abs(aa.angle());
  }

  return Interpolator1D<double>(t, x);
}


Interpolator1D<double> compute_derivated_orientation_arc(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform,
    double window_width)
{
  Interpolator1D<double> orientation_arc = compute_orientation_arc(transform);
  double tMin = transform->GetMinimumT() + 0.5 * window_width;
  double tMax = transform->GetMaximumT() - 0.5 * window_width;
  int steps = (tMax - tMin) / transform->GetPeriod() + 1;
  std::vector<double> derivated_orientation_arc = std::vector<double>(steps);
  std::vector<double> times = std::vector<double>(steps);
  for (int i = 0; i < steps; i++)
  {
    double time = tMin + i * transform->GetPeriod();
    times[i] = time;
    // length is an interpolator so no need to check that the sample instants
    // are not the same (they are not, even if the interpolation mode of
    // this->Reference/Aligned is "NEAREST")
    derivated_orientation_arc[i] =
                    (orientation_arc.Get(time + 0.5 * window_width)
                     - orientation_arc.Get(time - 0.5 * window_width))
                    / window_width;
  }

  return Interpolator1D<double>(times, derivated_orientation_arc);
}


// TODO: handle case where real_sample_time(next) == real_sample_time(prev)
Interpolator1D<double> compute_orientation_angle(
    const vtkSmartPointer<vtkVelodyneTransformInterpolator>& transform,
    double window_width)
{
  vtkSmartPointer<vtkTransform> prev = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> next = vtkSmartPointer<vtkTransform>::New();
  double tMin = transform->GetMinimumT() + 0.5 * window_width;
  double tMax = transform->GetMaximumT() - 0.5 * window_width;
  int steps = (tMax - tMin) / transform->GetPeriod() + 1;
  std::vector<double> orientation_angle = std::vector<double>(steps);
  std::vector<double> times = std::vector<double>(steps);
  for (int i = 0; i < steps; i++)
  {
    double time = tMin + i * transform->GetPeriod();
    transform->InterpolateTransform(time - 0.5 * window_width, prev);
    transform->InterpolateTransform(time + 0.5 * window_width, next);
    times[i] = time;
    Eigen::AngleAxisd angleAxis(RotationMatrixFromTransform(next)
		    * RotationMatrixFromTransform(prev).transpose());
    orientation_angle[i] = angleAxis.angle();
  }

  return Interpolator1D<double>(times, orientation_angle);
}

double ComputeTimeShift(vtkSmartPointer<vtkTemporalTransforms> reference,
                      vtkSmartPointer<vtkTemporalTransforms> aligned,
                      CorrelationStrategy correlationStrategy,
                      double time_window_width,
                      bool substract_mean)
{
  vtkSmartPointer<vtkVelodyneTransformInterpolator> referenceInterpolator
      = reference->CreateInterpolator();
  referenceInterpolator->SetInterpolationTypeToLinear();
  vtkSmartPointer<vtkVelodyneTransformInterpolator> alignedInterpolator
      = aligned->CreateInterpolator();
  alignedInterpolator->SetInterpolationTypeToLinear();
  Interpolator1D<double> sig_reference;
  Interpolator1D<double> sig_aligned;
  // first, compute the signals using the chosen method
  switch (correlationStrategy)
  {
    case CorrelationStrategy::DPOS:
      sig_reference = compute_dPos(referenceInterpolator);
      sig_aligned = compute_dPos(alignedInterpolator);
      break;
    case CorrelationStrategy::SPEED_WINDOW:
      sig_reference = compute_speed_window(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_speed_window(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::ACC_WINDOW:
      sig_reference = compute_acc_window(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_acc_window(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::JERK_WINDOW:
      sig_reference = compute_jerk_window(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_jerk_window(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::LENGTH:
      sig_reference = compute_length(referenceInterpolator);
      sig_aligned = compute_length(alignedInterpolator);
      break;
    case CorrelationStrategy::DERIVATED_LENGTH:
      sig_reference = compute_derivated_length(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_derivated_length(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::DROT:
      sig_reference = compute_dRot(referenceInterpolator);
      sig_aligned = compute_dRot(alignedInterpolator);
      break;
    case CorrelationStrategy::TRAJECTORY_ANGLE:
      sig_reference = compute_trajectory_angle(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_trajectory_angle(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::ORIENTATION_ANGLE:
      sig_reference = compute_orientation_angle(referenceInterpolator,
                                                time_window_width);
      sig_aligned = compute_orientation_angle(alignedInterpolator,
                                              time_window_width);
      break;
    case CorrelationStrategy::DERIVATED_ORIENTATION_ARC:
      sig_reference = compute_derivated_orientation_arc(
                              referenceInterpolator,
                              time_window_width);
      sig_aligned = compute_derivated_orientation_arc(
                              alignedInterpolator,
                              time_window_width);
      break;
    default:
      std::cerr << "unknown correlation strategy" << std::endl;
      return 0.0;
  }

  if (substract_mean)
  {
    sig_reference.ApplyValueShift(- sig_reference.Mean());
    sig_aligned.ApplyValueShift(- sig_aligned.Mean());
  }

  // We prefere the two signals to start at t = 0, so we time shift them,
  // but before that we save the information that we would lose otherwise.
  double pre_resample = sig_aligned.GetMinimumT() - sig_reference.GetMinimumT();
  sig_aligned.ApplyTimeShift(- sig_aligned.GetMinimumT());
  sig_reference.ApplyTimeShift(- sig_reference.GetMinimumT());

  // by construction we now have tMin == 0.0;
  double tMax = std::max(sig_reference.GetMaximumT(), sig_aligned.GetMaximumT());
  double period = std::min(sig_reference.GetAveragePeriod(),
		  sig_aligned.GetAveragePeriod());
  int steps = std::floor(tMax / period);
  std::vector<double> reference_resampled = std::vector<double>(steps);
  std::vector<double> aligned_resampled = std::vector<double>(steps);
  for (int i = 0; i < steps; i++)
  {
    double time = 0.0 + i * period;
    reference_resampled[i] = sig_reference.Get(time);
    aligned_resampled[i] = sig_aligned.Get(time);
  }

  std::vector<double> test = fftcorrelate(reference_resampled, aligned_resampled);

  int correlation = max_fftcorrelation(reference_resampled, aligned_resampled);
  double correction = correlation * period;
  double delta_t = pre_resample - correction;
  return delta_t;
}

void ShowTrajectoryInfo(vtkSmartPointer<vtkTemporalTransforms> reference, vtkSmartPointer<vtkTemporalTransforms> aligned)
{
  vtkSmartPointer<vtkVelodyneTransformInterpolator> referenceI = reference->CreateInterpolator();
  vtkSmartPointer<vtkVelodyneTransformInterpolator> alignedI = aligned->CreateInterpolator();
  std::cout << std::fixed;
  std::cout << std::setprecision(4);
  std::cout << "reference:                 " << referenceI->GetMaximumT() - referenceI->GetMinimumT()
            << "s from " << referenceI->GetMinimumT() << " to " << referenceI->GetMaximumT()
            << ", period is " << referenceI->GetPeriod() << std::endl;
  std::cout << "aligned:                   " << alignedI->GetMaximumT() - alignedI->GetMinimumT()
            << "s from " << alignedI->GetMinimumT() << " to " << alignedI->GetMaximumT()
            << ", period is " << alignedI->GetPeriod() << std::endl;
}

void DemoAllTimesyncMethods(vtkSmartPointer<vtkTemporalTransforms> reference, vtkSmartPointer<vtkTemporalTransforms> aligned) {
  vtkSmartPointer<vtkVelodyneTransformInterpolator> referenceI = reference->CreateInterpolator();
  vtkSmartPointer<vtkVelodyneTransformInterpolator> alignedI = aligned->CreateInterpolator();

  std::cout << std::fixed;
  std::cout << std::setprecision(4);
  ShowTrajectoryInfo(reference, aligned);
  std::cout << std::endl;
  std::cout << "dPos:                      " << ComputeTimeShift(reference, aligned, CorrelationStrategy::DPOS, 1.0) << std::endl;
  std::cout << "speed window:              " << ComputeTimeShift(reference, aligned, CorrelationStrategy::SPEED_WINDOW, 1.0) << std::endl;
  std::cout << "acceleration window:       " << ComputeTimeShift(reference, aligned, CorrelationStrategy::ACC_WINDOW, 3) << std::endl;
  std::cout << "jerk window:               " << ComputeTimeShift(reference, aligned, CorrelationStrategy::JERK_WINDOW, 6) << std::endl;
  std::cout << "derivated length:          " << ComputeTimeShift(reference, aligned, CorrelationStrategy::DERIVATED_LENGTH, 1.0) << std::endl;
  std::cout << "dRot:                      " << ComputeTimeShift(reference, aligned, CorrelationStrategy::DROT, 1.0) << std::endl;
  std::cout << "trajectory angle:          " << ComputeTimeShift(reference, aligned, CorrelationStrategy::TRAJECTORY_ANGLE, 10.0) << std::endl;
  std::cout << "orientation angle:         " << ComputeTimeShift(reference, aligned, CorrelationStrategy::ORIENTATION_ANGLE, 1.0) << std::endl;
  std::cout << "derivated orientation arc: " << ComputeTimeShift(reference, aligned, CorrelationStrategy::DERIVATED_ORIENTATION_ARC, 1.0) << std::endl;
}


double ComputeScale(vtkSmartPointer<vtkTemporalTransforms> reference,
                  vtkSmartPointer<vtkTemporalTransforms> aligned,
                  CorrelationStrategy correlationStrategy,
                  double time_window_width)
{
  const double div_epsilon = 1e-4;
  vtkSmartPointer<vtkVelodyneTransformInterpolator> referenceInterpolator = reference->CreateInterpolator();
  referenceInterpolator->SetInterpolationTypeToLinear();
  vtkSmartPointer<vtkVelodyneTransformInterpolator> alignedInterpolator = aligned->CreateInterpolator();
  alignedInterpolator->SetInterpolationTypeToLinear();
  Interpolator1D<double> sig_reference;
  Interpolator1D<double> sig_aligned;
  switch (correlationStrategy)
  {
    case CorrelationStrategy::DPOS:
      sig_reference = compute_dPos(referenceInterpolator);
      sig_aligned = compute_dPos(alignedInterpolator);
      break;
    case CorrelationStrategy::SPEED_WINDOW:
      sig_reference = compute_speed_window(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_speed_window(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::ACC_WINDOW:
      sig_reference = compute_acc_window(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_acc_window(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::JERK_WINDOW:
      sig_reference = compute_jerk_window(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_jerk_window(alignedInterpolator,
                                             time_window_width);
      break;
    case CorrelationStrategy::LENGTH:
      sig_reference = compute_length(referenceInterpolator);
      sig_aligned = compute_length(alignedInterpolator);
      break;
    case CorrelationStrategy::DERIVATED_LENGTH:
      sig_reference = compute_derivated_length(referenceInterpolator,
                                               time_window_width);
      sig_aligned = compute_derivated_length(alignedInterpolator,
                                             time_window_width);
      break;
    default:
      std::cerr << "unsuported correlation strategy" << std::endl;
      return 0.0;
  }

  double tMin = std::min(sig_reference.GetMinimumT(), sig_aligned.GetMaximumT());
  double tMax = std::max(sig_reference.GetMaximumT(), sig_aligned.GetMaximumT());
  double period = std::min(sig_reference.GetAveragePeriod(),
		  sig_aligned.GetAveragePeriod());
  int steps = std::floor((tMax - tMin) / period);
  std::vector<double> ratios;
  for (int i = 0; i < steps; i++)
  {
    double time = tMin + i * period;
    double reference_resampled = sig_reference.Get(time);
    double aligned_resampled = sig_aligned.Get(time);
    if (reference_resampled < div_epsilon)
    {
      continue;
    }
    else
    {
      ratios.push_back(aligned_resampled / reference_resampled);
    }
  }

  return ComputeMedian(ratios);
}
