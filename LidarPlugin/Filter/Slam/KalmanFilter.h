#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <math.h>

#include <Eigen/Dense>

class KalmanFilter
{
public:
  // default constructor
  KalmanFilter();

  // Reset the class
  void ResetKalmanFilter();

  // Set current time of the algorithm
  void SetCurrentTime(double time);

  // Prediction of the next state vector
  void Prediction();

  // Correction of the prediction using
  // the input measure
  void Correction(Eigen::MatrixXd Measure);

  // Set the measures variance covariance matrix
  void SetMeasureCovariance(Eigen::MatrixXd argCov);

  // Set the maximum angle acceleration
  // use to compute variance covariance matrix
  void SetMaxAngleAcceleration(double acc);

  // Set the maximum velocity acceleration
  // use to compute variance covariance matrix
  void SetMaxVelocityAcceleration(double acc);

  // return the state vector
  Eigen::Matrix<double, 12, 1> GetStateVector();

  // Initialize the state vector and the covariance-variance
  // estimation
  void SetInitialStatevector(Eigen::Matrix<double, 12, 1> iniVector, Eigen::Matrix<double, 12, 12> iniCov);

  // set the kalman filter mode
  void SetMode(int argMode);
  int GetMode();

  // return the number of observed measures
  int GetNbrMeasure();

private:
  // Kalman Filter mode:
  // 0 : Motion Model
  // 1 : Motion Model + GPS velocity
  int mode;

  // Motion model / Prediction Model
  Eigen::Matrix<double, 12, 12> MotionModel;

  // Link between the measures and the state vector
  Eigen::MatrixXd MeasureModel;

  // Variance-Covariance of measures
  Eigen::MatrixXd MeasureCovariance;

  // Variance-Covariance of model
  Eigen::Matrix<double, 12, 12> ModelCovariance;

  // State vector composed like this:
  // -rx, ry, rz
  // -tx, ty, tz
  // -drx/dt, dry/dt, drz/dt
  // -dtx/dt, dty/dt, dtz/dt
  Eigen::Matrix<double, 12, 1> VectorState;
  Eigen::Matrix<double, 12, 1> VectorStatePredicted;

  // Estimator variance covariance
  Eigen::Matrix<double, 12, 12> EstimatorCovariance;

  // delta time for prediction
  double PreviousTime;
  double CurrentTime;
  double DeltaTime;

  // Maximale acceleration endorsed by the vehicule
  double MaxAcceleration;
  double MaxAngleAcceleration;

  // indicate the number of observed measures
  unsigned int NbrMeasures;
};

#endif // KALMANFILTER_H
