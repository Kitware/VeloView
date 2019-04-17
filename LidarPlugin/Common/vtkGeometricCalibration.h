//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Data: 01-23-2019
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//=========================================================================

// This file provides multiple function to estimate the geometric calibration
// of a multi-sensor system composed with Lidar, IMU and RGB-cameras.
// To estimate the geometric calibration we make the assumption that the
// sensors are correctly fixed to the vehicle system; thus, the global system
// is a solid.
//
// Using the geometric constraints that come from the solid-assumption, it is
// possible to estimate the 6-DoF calibration (R in SO(3), T in R^3) between
// two sensors by using the estimation of their poses over the time.
//
// Let's t0 and t1 be two times
// Let's R, T be the pose of the sensor 1 according to the sensor 2 reference frame
// Since we have the solid-assumption, R and T are constant and not time depending
//
// Let's P0 (resp P1) in SO(3) be the orientation of the sensor 1 at time t0 (resp t1)
// Let's V0 (resp V1) in R^3 be the position of the sensor 1 at time t0 (resp t1)
//
// Let's Q0 (resp Q1) in SO(3) be the orientation of the sensor 2 at time t0 (resp t1)
// Let's U0 (resp U1) in R^3 be the position of the sensor 2 at time t0 (resp t1)
//
// From these two temporal points on the poses "trajectory", it is possible to express
// the change of reference frame between the sensor 1 at time t0 and sensor 1 at time t1
// using two differents way
//
// 1- By using the poses of the sensor 1 at time t0 and t1:
//    dR0 = P0' * P1
//    dT0 = P0' * (V1 - V0)
//
// 2- By using the solid-assumption and firstly express the point in the
//    other sensor reference frame using the calibration parameters. Then, using
//    method 1 for the second sensor and finally using the calibration again to
//    dR1 = R' * Q0' * Q1 * R
//    dT1 = R0' * (Q0' * (Q1 * T + (U1 - U0)) - T)
//    
// And finally, we are looking for R and T that satisfies:
// dR1 = dR0
// dT1 = dT0
//
// These two geometrics constraints indicates that one can obtain the trajectory
// of one sensor using a "cycloid" transform of the second sensor trajectory:
//
// T1(t) = T2(t) + R2(t) * T21
//
// This lead to a non-linear least square problem that can be solved using a
// levenberg-marquardt algorithm. The non-linearity comes from R belonging to
// SO(3) manifold. We estimate R using the Euler-Angle mapping between R^3 and SO(3)
// R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)

#ifndef VTK_CALIBRATION_ESTIMATION_TOOLS_H
#define VTK_CALIBRATION_ESTIMATION_TOOLS_H

// LOCAL
#include "vtkTemporalTransforms.h"

// STD
#include <vector>

// EIGEN
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> AnglePositionVector;

/**
* \function EstimaterEulerAngleConvention
* \brief This function will find the correct
*        euler angle convention:
*
*        - angles signs
*        - angles association with corresponding axis
*        - canonic rotation around axis multiplication order
*
*        by brute-forcing the 288* combination and conjointly
*        estimating the geometric calibration that makes the
*        two sensors poses trajectories consistent. As a result,
*        the algorithm will highlight some equivalence classes since
*        some Euler angles conventions are equivalent up to a "calibration"
*        rotation matrix
*
*        *288 = 6 * 6 * 8
*        - 6: number of possible permutations for angles position
*        - 6: number of possible permutation for matrix multiplication order
*        - 8: number of possible sign assigned to the angles
*
* \@param targetSensor Poses trajectory of the first sensor
* \@param Sensor2Poses Poses trajectory of the second sensor
*/
void EstimateEulerAngleConvention(vtkSmartPointer<vtkTemporalTransforms> sourceSensor,
                                  vtkSmartPointer<vtkTemporalTransforms> targetSensor);
void EstimateEulerAngleConvention(const std::string& sourceSensorFilename,
                                  const std::string& targetSensorFilename);


/**
* \function EstimateCalibrationFromPoses
* \brief This function will estimate the geometric
*        calibration of two sensors using the 'solid-system"
*        assumption and the poses trajectories of these two
*        sensors.
*        The geometric calibration is the pose of the sensor source
*        according to the sensor target reference frame
*
* \@param targetSensor Poses trajectory of the first sensor
* \@param sourceSensor Poses trajectory of the second sensor
*/
std::pair<double, AnglePositionVector> EstimateCalibrationFromPoses(
                                            vtkSmartPointer<vtkTemporalTransforms> sourceSensor,
                                            vtkSmartPointer<vtkTemporalTransforms> targetSensor);
std::pair<double, AnglePositionVector> EstimateCalibrationFromPoses(const std::string& sourceSensorFilename,
                                                                    const std::string& targetSensorFilename);
vtkSmartPointer<vtkTemporalTransforms> EstimateCalibrationFromPosesAndApply(
                                            vtkSmartPointer<vtkTemporalTransforms> targetSensor,
                                            vtkSmartPointer<vtkTemporalTransforms> sourceSensor);

/**
* \function MatchTrajectoriesWithIsometry
* \brief This function will compute the best isometry
*        that match the trajectory of the sensor target on the
*        the trajectory of the sensor source. This works when
*        the baseline between the two sensors is null and
*        that the reference frame change bteween the sensors
*        only consist of a difference of orientation. However,
*        when the baseline is not null the trajectories are not
*        linked by a isometry transform. There is a cycloidic
*        that links the two trajectories. It is still possible to
*        use this function to have an approximation of the rotation
*        part of the calibration
*
*
* \@param targetSensor Poses trajectory of the first sensor
* \@param sourceSensor Poses trajectory of the second sensor
*/
std::pair<double, AnglePositionVector> MatchTrajectoriesWithIsometry(
                                                        vtkSmartPointer<vtkTemporalTransforms> sourceSensor,
                                                        vtkSmartPointer<vtkTemporalTransforms> targetSensor);
std::pair<double, AnglePositionVector> MatchTrajectoriesWithIsometry(const std::string& sourceSensorFilename,
                                                                     const std::string& targetSensorFilename);
vtkSmartPointer<vtkTemporalTransforms> MatchTrajectoriesWithIsometryAndApply(
                                                         vtkSmartPointer<vtkTemporalTransforms> targetSensor,
                                                         vtkSmartPointer<vtkTemporalTransforms> sourceSensor);

/**
* \function CreateSyntheticPosesData
* \brief This function creates synthetic data:
*        - A vehicle Poses trajectory expressed
*          in the world reference coordinate frame
*        - A GPS / IMU Poses trajectory expressed
*          in the world reference coordinate frame
*          It differs from the vehicle reference frame
*          due to the gps / imu orientation and position
*          according to the vehicle reference frame
*        - A LiDAR - SLAM poses trajectory expressed
*          in the initial LiDAR pose reference coordinate frame
*          It differs from the vehicle reference frame due to
*          its orientation and position and due to the fact
*          that it is not expressed in the world reference frame
*
*
* \@param vehiclePosesFilename Poses trajectory of the vehicle
* \@param imuPosesFilename Poses trajectory of the IMU - GPS
* \@param sensorPosesFilename Poses trajectory of the LiDAR - SLAM
*/
void CreateSyntheticPosesData(const std::string& vehiclePosesFilename,
                              const std::string& imuPosesFilename,
                              const std::string& sensorPosesFilename);

#endif // VTK_CALIBRATION_ESTIMATION_TOOLS_H
