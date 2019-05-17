#include "KalmanFilter.h"

#include <iostream>


//-----------------------------------------------------------------------------
KalmanFilter::KalmanFilter()
{
  this->ResetKalmanFilter();
}

//-----------------------------------------------------------------------------
void KalmanFilter::ResetKalmanFilter()
{
  // set the number of measures observed
  this->NbrMeasures = 6;
  if (this->mode > 0)
  {
    this->NbrMeasures = 7;
  }

  // Fill motion Model diagonal
  this->MotionModel = Eigen::Matrix<double, 12, 12>::Zero();
  for (unsigned int i = 0; i < 12; ++i)
  {
      this->MotionModel(i, i) = 1.0;
  }

  // Fill Estimator covariance
  // Set to zero because without
  // any other information
  this->EstimatorCovariance = Eigen::Matrix<double, 12, 12>::Zero();

  // Fill measure model
  this->MeasureModel = Eigen::MatrixXd(this->NbrMeasures, 12);// ::Matrix<double, 6, 12>::Zero();
  for (unsigned int j = 0; j < 12; ++j)
  {
    for (unsigned int i = 0; i < this->NbrMeasures; ++i)
    {
      this->MeasureModel(i, j) = 0.0;
    }
  }
  for (unsigned int i = 0; i < 6; ++i)
  {
    this->MeasureModel(i, i) = 1.0;
  }

  // Fill Motion model covariance
  this->ModelCovariance = Eigen::Matrix<double, 12, 12>::Zero();

  // Fill vector state
  this->VectorState << 0,0,0,0,0,0,0,0,0,0,0,0;

  // Set the maximale acceleration
  // Settle to 10 m.s-2 (= 1g). it is
  // the maximal acceleration that a "normal"
  // car can endorsed. Moreover, it the acceleration
  // of a falling drone. Seems a good limit
  // Reducing the maximal acceleration will
  // reduce the motion model covariance matrix
  // We can take an additional 20% error
  this->MaxAcceleration = 1.2 * 10.0;

  // Maximal acceleration settled to
  // 1080 degrees / s-2. To have an image
  // the maximale acceleration corresponds
  // to an none-mobile object than can goes
  // up to 3 rotation per secondes (180 per min)
  // in one second. This seems reasonable
  // We can take an additional 20% error
  this->MaxAngleAcceleration = 1.2 * 1080.0 / 180.0 * M_PI;
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetInitialStatevector(Eigen::Matrix<double, 12, 1> iniVector, Eigen::Matrix<double, 12, 12> iniCov)
{
  this->VectorState = iniVector;
  this->EstimatorCovariance = iniCov;
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetCurrentTime(double time)
{
  // Update time
  this->PreviousTime = this->CurrentTime;
  this->CurrentTime = time;
  this->DeltaTime = this->CurrentTime - this->PreviousTime;

  // Update motion model matrix
  for (unsigned int i = 0; i <= 5; ++i)
  {
    this->MotionModel(i, i + 6) = this->DeltaTime;
  }

  // Update Motion model covariance matrix
  // angle
  for (unsigned int i = 0; i < 3; ++i)
  {
    this->ModelCovariance(i, i) = std::pow(0.5 * this->MaxAngleAcceleration * std::pow(this->DeltaTime, 2), 2);
  }
  // Position
  for (unsigned int i = 3; i < 6; ++i)
  {
    this->ModelCovariance(i, i) = std::pow(0.5 * this->MaxAcceleration * std::pow(this->DeltaTime, 2), 2);
  }
  // Angle speed
  for (unsigned int i = 6; i < 9; ++i)
  {
    this->ModelCovariance(i, i) = std::pow(this->MaxAngleAcceleration * this->DeltaTime, 2);
  }
  // Velocity
  for (unsigned int i = 9; i < 12; ++i)
  {
    this->ModelCovariance(i, i) = std::pow(this->MaxAcceleration * this->DeltaTime, 2);
  }
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetMeasureCovariance(Eigen::MatrixXd argCov)
{
  this->MeasureCovariance = argCov;
}

//-----------------------------------------------------------------------------
void KalmanFilter::Prediction()
{
  // Prediction using motion model and motion covariance
  // Vector state prediction
  this->VectorStatePredicted = this->MotionModel * this->VectorState;
  // Estimator covariance update
  this->EstimatorCovariance = this->MotionModel * this->EstimatorCovariance * this->MotionModel.transpose() + this->ModelCovariance;
}

//-----------------------------------------------------------------------------
void KalmanFilter::Correction(Eigen::MatrixXd Measure)
{
  // Update the measure model, since we have a non
  // linear link between the state vector and the measure
  // (norm of the velocity) we need to compute the jacobian
  // of the measure function at the current point in the
  // state vector space
  double normV = std::sqrt(std::pow(this->VectorStatePredicted(9), 2) + std::pow(this->VectorStatePredicted(10), 2) + std::pow(this->VectorStatePredicted(11), 2));
  if (this->mode > 0)
  {
    // check that the norm is not null
    double nv = normV;
    if (nv < 1e-6)
      nv = 1.0;

    this->MeasureModel(6, 9) = this->VectorStatePredicted(9) / nv;
    this->MeasureModel(6, 10) = this->VectorStatePredicted(10) / nv;
    this->MeasureModel(6, 11) = this->VectorStatePredicted(11) / nv;
    std::cout << "Vector State: " << std::endl << this->VectorStatePredicted.transpose() << std::endl;
    std::cout << "Measure: " << std::endl << Measure.transpose() << std::endl;
    std::cout << "Measure model: " << std::endl << this->MeasureModel << std::endl;
  }

  // Update using the measure and its covariance
  Eigen::MatrixXd novelty = (this->MeasureModel * this->EstimatorCovariance * this->MeasureModel.transpose() + this->MeasureCovariance);
  Eigen::MatrixXd gain = this->EstimatorCovariance * this->MeasureModel.transpose() * novelty.inverse();

  // Update the vector state estimation
  Eigen::MatrixXd errVector(this->NbrMeasures, 1);
  if (this->mode > 0)
  {
    for (unsigned int k = 0; k < 6; ++k)
    {
      errVector(k) = this->VectorStatePredicted(k);
    }
    errVector(6) = normV;
    errVector = Measure - errVector;
  }
  else
  {
    errVector = Measure - this->MeasureModel * this->VectorStatePredicted;
  }

  this->VectorState = this->VectorStatePredicted + gain * errVector;

  // Update Estimator covariance
  this->EstimatorCovariance = this->EstimatorCovariance - gain * this->MeasureModel * this->EstimatorCovariance;
}

//-----------------------------------------------------------------------------
Eigen::Matrix<double, 12, 1> KalmanFilter::GetStateVector()
{
  return this->VectorState;
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetMaxAngleAcceleration(double acc)
{
  this->MaxAngleAcceleration = acc / 180.0 * M_PI;
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetMaxVelocityAcceleration(double acc)
{
  this->MaxAcceleration = acc;
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetMode(int argMode)
{
  this->mode = argMode;
  this->ResetKalmanFilter();
}

//-----------------------------------------------------------------------------
int KalmanFilter::GetMode()
{
  return this->mode;
}

//-----------------------------------------------------------------------------
int KalmanFilter::GetNbrMeasure()
{
  return this->NbrMeasures;
}
