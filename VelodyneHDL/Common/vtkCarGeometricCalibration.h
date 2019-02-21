//=========================================================================
//
// Copyright 2019 Kitware, Inc.
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


// These functions allows finding the rotation part of the calibration for a
// vehicule that as a "forward facing trajectory", such as a car or a truck that
// would never drive in reverse, using their pose trajecories.
// The orientation of a such vehicules is correlated with their trajectory,
// which is not the case for a drone (for example).
// Pose trajectory: evolution of the pose (computer vision) over time.
// The trajectory should be produced in an urban environnement: straight lines
// alternated with relatively sharp turns (though not necessary Manatthan like).

#include <vtkSmartPointer.h>
#include <vtkVelodyneTransformInterpolator.h>
#include <Eigen/SVD>

#include "vvConfigure.h"
#include "vtkTemporalTransforms.h"
#include "statistics.h"

/**
* \brief This function identifies turns in the pose trajectory of a car-like
* vehicle.
*
* This function assumes that the trajectory will belong to roads, i.e. will
* alternate between "sharp turns" and "straight lines". However there is no
* requirement to drive in a Manhattan-like road network: if some portions of
* the trajectory are neither sharp turns not straight lines, they will not
* produce detection of turns.
*
* The turns returned are described using 4 times: (t1, t2, t3, t4)
*
* \verbatim
*
*    V
*    o
*    o
*    o -> t1
*    o
*    o
*    o
*    o
*    o -> t2
*    o
*    oo
*     oo
*       oo
*        oo
*         oooo
*            ooooo         t3      t4
*                ooooooo   ^       ^
*                       oooooooooooooo> (trajectory of the car)
* \endverbatim
*
* Between times (t1 and t2) and (t3 and t4), the trajectory should be in
* a relative straight line.
*
* \param timeWindow in seconds, parameter used to measure pose trajectory
* curvature.
* In my experiments, 0.8 seconds was a good default value for a car in a
* urban environnement.
* \param curveTreshold in rad/s, parameter used to define what is a turn.
* In my experiments, 0.15 rad/s was a good default value for a car in a
* urban environnement.
* \param emptyIntersection if true, the turns detected will not intersect.
*/
std::vector<std::vector<double>> ComputeTurns(
        const vtkSmartPointer<vtkVelodyneTransformInterpolator> poseTrajectory,
        double timeWindow,
        double curveTreshold,
        bool emptyIntersection,
        std::string debugCSV = ""
        );

/**
* \brief This function compute the rotation and scale part of the geometric
* calibration between two sensors that were used to produce two pose
* trajectories.
*
* This method only requires the two pose trajectories to be time-synced.
* There is no need to have them scaled, which can be useful when calibrating
* against a camera on which a monocular slam was run.
* Hopefully this method will not be too sensitive to a time-sync which is not
* perfect because we detect the turns and use the direction in space before and
* after the turn, not single points of the pose trajectory.
*
* Possible improvement: replacing the ransac by some clustering method and take
* a median of the selected cluster.
*
* \param ransacFittingRatio between 0 and 1, can be taken as low as 0 because
* we max the resulting count of matrices with 1, and because a single rotation
* is sufficient to estimate a rotation.
* \param ransacValidationRatio between 0 and 1, should be taken as big as
* possible but I had to lower it down to 0.15 for some real life datasets.
**/
void VelodyneHDLPlugin_EXPORT ComputeCarCalibrationRotationScale(
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
        bool verbose = false
        );
