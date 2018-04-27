#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paraview.simple as smp
import applogic

import PythonQt

from PythonQt import QtGui
from paraview import vtk
from PythonQt.paraview import vvSlamConfigurationDialog

def DisplayKeypointMap():
    return

def DebugAddFrame(n):

    # get data
    source = applogic.getReader()
    source = source.GetClientSideObject()

    # get the  frame
    polyData = source.GetFrame(n)

    # compute the SLAM for the current frame
    slam.AddFrame(polyData)
#    smp.Show(polyData)

    # get the transformation computed
    Tworld = range(6)
    slam.GetWorldTransform(Tworld)
    t = polyData.GetPointData().GetArray("adjustedtime").GetTuple1(0) * 1e-6

    # convert in degree
    rx = Tworld[0] * 180 / vtk.vtkMath.Pi()
    ry = Tworld[1] * 180 / vtk.vtkMath.Pi()
    rz = Tworld[2] * 180 / vtk.vtkMath.Pi()
    # permute the axes
    tx = Tworld[3]
    ty = Tworld[4]
    tz = Tworld[5]

    # add the transform
    source.AddTransform(rx, ry, rz, tx, ty, tz, t)


    # close file
    source.Close()
    # merge them



def launch():

    # get data
    source = applogic.getReader()
    source = source.GetClientSideObject()

    #If no data are available
    if not source :
        return

    # execute the gui
    slamDialog = vvSlamConfigurationDialog(applogic.getMainWindow())
    if not slamDialog.exec_():
        return

    # Instanciation of a new vtkSlamAlgorithm
    slam = smp.Slam()

    # open file
    source.Open()
    # create Linear Interpolator()
    source.CreateLinearInterpolator()

    # initialize the nb of laser and their mapping
    NLaser = source.GetNumberOfChannels()
    laserIdMapping = range(NLaser * 2)
    source.GetLaserIdMapping(laserIdMapping)
    slam.GetClientSideObject().SetSensorCalibration(laserIdMapping, NLaser)

    slam.GetClientSideObject().Set_DisplayMode(False)

    # Set parameter selected by the user

    # mode
    streamMode = False
    if slamDialog.frameMode == vvSlamConfigurationDialog.FRAME_RANGE:
        start = slamDialog.frameStart
        stop = slamDialog.frameStop
    elif slamDialog.frameMode == vvSlamConfigurationDialog.CURRENT_FRAME:
        start = applogic.app.scene.StartTime
        stop = applogic.app.scene.EndTime
        streamMode = True
    elif slamDialog.frameMode == vvSlamConfigurationDialog.ALL_FRAMES:
        start = applogic.app.scene.StartTime
        stop = applogic.app.scene.EndTime

    # General
    slam.GetClientSideObject().Set_RollingGrid_Grid_NbVoxel([slamDialog.NbVoxel,slamDialog.NbVoxel,slamDialog.NbVoxel])
    slam.GetClientSideObject().Set_AngleResolution(slamDialog.AngleResolution * vtk.vtkMath.Pi() / 180)
    # Keypoint
    slam.GetClientSideObject().Set_Keypoint_MaxEdgePerScanLine(slamDialog.Keypoint_MaxEdgePerScanLine)
    slam.GetClientSideObject().Set_Keypoint_MaxPlanarsPerScanLine(slamDialog.Keypoint_MaxPlanarsPerScanLine)
    slam.GetClientSideObject().Set_Keypoint_MinDistanceToSensor(slamDialog.Keypoint_MinDistanceToSensor)
    slam.GetClientSideObject().Set_Keypoint_PlaneCurvatureThreshold(slamDialog.Keypoint_PlaneCurvatureThreshold)
    slam.GetClientSideObject().Set_Keypoint_EdgeCurvatureThreshold(slamDialog.Keypoint_EdgeCurvatureThreshold)
    # Egomotion
    slam.GetClientSideObject().Set_EgoMotionMaxIter(slamDialog.EgoMotion_MaxIter)
    slam.GetClientSideObject().Set_EgoMotionIcpFrequence(slamDialog.EgoMotion_IcpFrequence)
    slam.GetClientSideObject().Set_EgoMotionLineDistanceNbrNeighbors(slamDialog.EgoMotion_LineDistance_k)
    slam.GetClientSideObject().Set_EgoMotionLineDistancefactor(slamDialog.EgoMotion_LineDistance_factor)
    slam.GetClientSideObject().Set_EgoMotionPlaneDistanceNbrNeighbors(slamDialog.EgoMotion_PlaneDistance_k)
    slam.GetClientSideObject().Set_EgoMotionPlaneDistancefactor1(slamDialog.EgoMotion_PlaneDistance_factor1)
    slam.GetClientSideObject().Set_EgoMotionPlaneDistancefactor2(slamDialog.EgoMotion_PlaneDistance_factor2)
    slam.GetClientSideObject().Set_EgoMotionMaxLineDistance(slamDialog.EgoMotion_Line_Max_Distance)
    slam.GetClientSideObject().Set_EgoMotionMaxPlaneDistance(slamDialog.EgoMotion_Plane_Max_Distance)
    # Mapping
    slam.GetClientSideObject().Set_MappingMaxIter(slamDialog.Mapping_MaxIter)
    slam.GetClientSideObject().Set_MappingIcpFrequence(slamDialog.Mapping_IcpFrequence)
    slam.GetClientSideObject().Set_MappingLineDistanceNbrNeighbors(slamDialog.Mapping_LineDistance_k)
    slam.GetClientSideObject().Set_MappingLineDistancefactor(slamDialog.Mapping_LineDistance_factor)
    slam.GetClientSideObject().Set_MappingPlaneDistanceNbrNeighbors(slamDialog.Mapping_PlaneDistance_k)
    slam.GetClientSideObject().Set_MappingPlaneDistancefactor1(slamDialog.Mapping_PlaneDistance_factor1)
    slam.GetClientSideObject().Set_MappingPlaneDistancefactor2(slamDialog.Mapping_PlaneDistance_factor2)
    slam.GetClientSideObject().Set_MappingMaxLineDistance(slamDialog.Mapping_Line_Max_Distance)
    slam.GetClientSideObject().Set_MappingMaxPlaneDistance(slamDialog.Mapping_Plane_Max_Distance)

#    debug = True
#    if debug:
#        return

    # instanciate the progress box
    progressDialog = QtGui.QProgressDialog("Computing slam algorithm...", "Abort Slam", slamDialog.frameStart, slamDialog.frameStart + (slamDialog.frameStop - slamDialog.frameStart), None)
    progressDialog.setModal(True)
    progressDialog.show()

    # iteration
    for i in range(int(start), int(stop)+1):
        # get the current frame
        polyData = source.GetFrame(i)
        # compute the SLAM for the current frame
        slam.GetClientSideObject().AddFrame(polyData)

        # get the transformation computed
        Tworld = range(6)
        slam.GetClientSideObject().GetWorldTransform(Tworld)
        t = polyData.GetPointData().GetArray("adjustedtime").GetTuple1(0) * 1e-6

        # convert in degree
        rx = Tworld[0] * 180 / vtk.vtkMath.Pi()
        ry = Tworld[1] * 180 / vtk.vtkMath.Pi()
        rz = Tworld[2] * 180 / vtk.vtkMath.Pi()
        # permute the axes
        tx = Tworld[3]
        ty = Tworld[4]
        tz = Tworld[5]

        # add the transform
        source.AddTransform(rx, ry, rz, tx, ty, tz, t)

        # update the ui
        if (progressDialog.wasCanceled):
            # TODO pop up a message
            #    //    if(progress.wasCanceled())
            #    //      {
            #    //      progress.close();
            #    //      std::ostringstream message;
            #    //      int nbFrames = k-startFrame + 1;
            #    //      message << "Only " << nbFrames << ((nbFrames > 1) ? " frames" : " frame");
            #    //      message << " over " << endFrame-startFrame + 1 << " frames were computed." << std::endl;
            #    //      message << "The results are still visible.";
            #    //      QMessageBox::information(mainWindow, "Aborting SLAM",
            #    //          QString::fromStdString(message.str()));
            #    //      break;
            #    //      }
            break
        progressDialog.setValue(i)

    # close file
    source.Close()

    slam.GetClientSideObject().Update()

    # Reset Active source tp the data
    smp.SetActiveSource(applogic.getReader())

    # View slam output
    smp.Show(slam, applogic.app.overheadView)
#    slam.GetLocalEdgesKeypointMap();

def launchStreamSlam():
    # get data
    source = applogic.getReader()
    source = source.GetClientSideObject()

    #If no data are available
    if not source :
        return

    # execute the gui
    slamDialog = vvSlamConfigurationDialog(applogic.getMainWindow())
    if not slamDialog.exec_():
        return

    # Instanciation of a new vtkSlamAlgorithm
    slam = smp.Slam()

    # open file
    source.Open()
    # create Linear Interpolator()
    source.CreateLinearInterpolator()

    # initialize the nb of laser and their mapping
    NLaser = source.GetNumberOfChannels()
    laserIdMapping = range(NLaser * 2)
    source.GetLaserIdMapping(laserIdMapping)
    slam.GetClientSideObject().SetSensorCalibration(laserIdMapping, NLaser)

    slam.GetClientSideObject().Set_DisplayMode(False)

    # Set parameter selected by the user
    # General
    slam.GetClientSideObject().Set_RollingGrid_Grid_NbVoxel([slamDialog.NbVoxel,slamDialog.NbVoxel,slamDialog.NbVoxel])
    slam.GetClientSideObject().Set_AngleResolution(slamDialog.AngleResolution * vtk.vtkMath.Pi() / 180)
    # Keypoint
    slam.GetClientSideObject().Set_Keypoint_MaxEdgePerScanLine(slamDialog.Keypoint_MaxEdgePerScanLine)
    slam.GetClientSideObject().Set_Keypoint_MaxPlanarsPerScanLine(slamDialog.Keypoint_MaxPlanarsPerScanLine)
    slam.GetClientSideObject().Set_Keypoint_MinDistanceToSensor(slamDialog.Keypoint_MinDistanceToSensor)
    slam.GetClientSideObject().Set_Keypoint_PlaneCurvatureThreshold(slamDialog.Keypoint_PlaneCurvatureThreshold)
    slam.GetClientSideObject().Set_Keypoint_EdgeCurvatureThreshold(slamDialog.Keypoint_EdgeCurvatureThreshold)
    # Egomotion
    slam.GetClientSideObject().Set_EgoMotionMaxIter(slamDialog.EgoMotion_MaxIter)
    slam.GetClientSideObject().Set_EgoMotionIcpFrequence(slamDialog.EgoMotion_IcpFrequence)
    slam.GetClientSideObject().Set_EgoMotionLineDistanceNbrNeighbors(slamDialog.EgoMotion_LineDistance_k)
    slam.GetClientSideObject().Set_EgoMotionLineDistancefactor(slamDialog.EgoMotion_LineDistance_factor)
    slam.GetClientSideObject().Set_EgoMotionPlaneDistanceNbrNeighbors(slamDialog.EgoMotion_PlaneDistance_k)
    slam.GetClientSideObject().Set_EgoMotionPlaneDistancefactor1(slamDialog.EgoMotion_PlaneDistance_factor1)
    slam.GetClientSideObject().Set_EgoMotionPlaneDistancefactor2(slamDialog.EgoMotion_PlaneDistance_factor2)
    # Mapping
    slam.GetClientSideObject().Set_MappingMaxIter(slamDialog.Mapping_MaxIter)
    slam.GetClientSideObject().Set_MappingIcpFrequence(slamDialog.Mapping_IcpFrequence)
    slam.GetClientSideObject().Set_MappingLineDistanceNbrNeighbors(slamDialog.Mapping_LineDistance_k)
    slam.GetClientSideObject().Set_MappingLineDistancefactor(slamDialog.Mapping_LineDistance_factor)
    slam.GetClientSideObject().Set_MappingPlaneDistanceNbrNeighbors(slamDialog.Mapping_PlaneDistance_k)
    slam.GetClientSideObject().Set_MappingPlaneDistancefactor1(slamDialog.Mapping_PlaneDistance_factor1)
    slam.GetClientSideObject().Set_MappingPlaneDistancefactor2(slamDialog.Mapping_PlaneDistance_factor2)

    return slam
