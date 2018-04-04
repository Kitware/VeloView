#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Get the active source
import os
import csv
import datetime
import time
import math
import sys
import paraview.simple as smp
from paraview import servermanager
from paraview import vtk

import PythonQt
from PythonQt import QtCore, QtGui

from vtkIOXMLPython import vtkXMLPolyDataWriter
import kiwiviewerExporter
import gridAdjustmentDialog
import aboutDialog
import planefit
import slam

from PythonQt.paraview import vvCalibrationDialog, vvCropReturnsDialog, vvSelectFramesDialog
from VelodyneHDLPluginPython import vtkVelodyneHDLReader

def launch():

    # get data
    source = smp.GetActiveSource()

    #If no data are available
    if not source :
        return

#    frameOptions = getFrameSelectionFromUser(framePackVisibility=False, frameTransformVisibility=False)
    frameOptions = type('test', (object,), {})()
    frameOptions.start = 0
    frameOptions.stop = 50

    # Instanciation of a new vtkSlamAlgorithm
    slam = smp.Slam()

    # open file
    source.GetClientSideObject().Open()
    # create Linear Interpolator()
    #    source.GetClientSideObject().CreateLinearInterpolator()

    # initialize the nb of laser and their mapping
    NLaser = source.GetClientSideObject().GetNumberOfChannels()
    laserIdMapping = range(NLaser*2)
    source.GetClientSideObject().GetLaserIdMapping(laserIdMapping)
    slam.GetClientSideObject().SetSensorCalibration(laserIdMapping, NLaser)

    # instanciate the progress box
    progressDialog = QtGui.QProgressDialog("Computing slam algorithm...", "Abort Slam", frameOptions.start, frameOptions.start + (frameOptions.stop - frameOptions.start), None)
    progressDialog.setModal(True)
    progressDialog.show()

    # iteration
    for i in range(frameOptions.start, frameOptions.stop):
        # get the current frame
        polyData = source.GetClientSideObject().GetFrame(i)
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
        source.GetClientSideObject().AddTransform(rx, ry, rz, tx, ty, tz, t)

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
    source.GetClientSideObject().Close()
