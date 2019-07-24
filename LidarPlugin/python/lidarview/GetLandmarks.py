#==============================================================================
#
#  Program:   LidarView
#  Module:    GetLandmarks.py
#  Author:    Pierre Guilbert (pierre.guilbert@kitware.com)
#
#  Copyright (c) Kitware, Inc.
#  All rights reserved.
#  See Copyright.txt or http://www.paraview.org/HTML/Copyright.html for details.
#
#     This software is distributed WITHOUT ANY WARRANTY; without even
#     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#     PURPOSE.  See the above copyright notice for more information.
#
#==============================================================================

#==============================================================================
# From a selection, compute the geometric median of the p-percentil points
# closest to the current camera center location. If the p-percentil points
# cardinal is less than 3, compute the mean instead
#==============================================================================

import numpy as np

# -----------------------------------------------------------------------------
def GeometricMedian(X):
    # initialize the median to the mean
    N = X.shape[1]
    y = np.mean(X, axis=1)

    # norm of the residual vector
    # desired to stop the optimization
    # algorithm. Without units
    epsilon = 0.00000001

    # Refine the median estimation by
    # iteratively re-weight least squares
    shouldIterate = True
    count = 0
    while shouldIterate:
        nextY = np.zeros((3, ), dtype=np.float64)
        sumInvDist = 0.0

        for i in range(N):
            sumInvDist += 1.0 / np.linalg.norm(y - X[:, i])
            nextY += X[:, i] / np.linalg.norm(X[:, i] - y)

        y = nextY / sumInvDist
        residual = np.zeros((3, ), dtype=np.float64)

        for i in range(N):
            residual += (X[:, i] - y) / np.linalg.norm(X[:, i] - y)
        shouldIterate = np.linalg.norm(residual) > epsilon
        count += 1
        shouldIterate = shouldIterate and (count < 200)
    return y

# -----------------------------------------------------------------------------
def GetSelectedLandmark():
    # first, get the selected points
    src = lv.smp.GetActiveSource()
    selection = src.GetSelectionOutput(0)
    extractSelection = lv.smp.ExtractSelection(Input=src,Selection=selection.Selection)
    extractSelection.UpdatePipeline()
    cloud = extractSelection.GetClientSideObject().GetOutput().GetPoints()

    # then, get the camera position and compute
    # the distance to the camera center
    camera = lv.smp.GetActiveCamera()
    center = np.array(camera.GetPosition())
    dist = np.zeros((cloud.GetNumberOfPoints(), 2), dtype=np.float64)
    for i in range(cloud.GetNumberOfPoints()):
        pt = np.array(cloud.GetPoint(i))
        dist[i, 0] = np.linalg.norm(center - pt)
        dist[i, 1] = i
    sortedDist = dist[dist[:,0].argsort()]

    # take the median point of first p-centil
    p = 0.1
    nbrPoints = np.floor(p * cloud.GetNumberOfPoints()) + 1
    pointsList = np.zeros((3, int(nbrPoints)), dtype=np.float64)
    for i in range(int(nbrPoints)):
        pointsList[0, i] = cloud.GetPoint(int(sortedDist[i, 1]))[0]
        pointsList[1, i] = cloud.GetPoint(int(sortedDist[i, 1]))[1]
        pointsList[2, i] = cloud.GetPoint(int(sortedDist[i, 1]))[2]

    y = 0

    if nbrPoints > 3:
        y = GeometricMedian(pointsList)
        print("Median Used")
    else:
        N = pointsList.shape[1]
        y = np.mean(pointsList, axis=1)
        print("Mean Used")

    sphere = lv.smp.Sphere()
    sphere.Center = [y[0], y[1], y[2]]
    sphere.Radius = 0.05
    lv.smp.Show(sphere)
    lv.smp.Render()
    lv.smp.Delete(extractSelection)

    return y
