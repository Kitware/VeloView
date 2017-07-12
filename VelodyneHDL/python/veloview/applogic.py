# Copyright 2013 Velodyne Acoustics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

from PythonQt.paraview import vvCalibrationDialog, vvCropReturnsDialog, vvSelectFramesDialog
from VelodyneHDLPluginPython import vtkVelodyneHDLReader

_repCache = {}

SAMPLE_PROCESSING_MODE = False

def cachedGetRepresentation(src, view):
    try:
        return _repCache[(src, view)]
    except KeyError:
        rep = smp.GetRepresentation(src, view)
        _repCache[(src, view)] = rep
        return rep

class AppLogic(object):

    def __init__(self):
        self.playing = False
        self.playDirection = 1
        self.seekPlayDirection = 1
        self.seekPlay = False
        self.targetFps = 30
        self.renderIsPending = False
        self.createStatusBarWidgets()
        self.setupTimers()

        self.mousePressed = False

        mainView = smp.GetActiveView()
        views = smp.GetRenderViews()
        otherViews = [v for v in views if v != mainView]
        assert len(otherViews) == 1
        overheadView = otherViews[0]
        self.mainView = mainView
        self.overheadView = overheadView

        self.transformMode = 0
        self.relativeTransform = False

        self.reader = None
        self.position = (None, None, None)
        self.sensor = None

        self.fps = [0,0]

        self.text = None

        self.laserSelectionDialog = None

        self.gridProperties = None

    def setupTimers(self):
        self.playTimer = QtCore.QTimer()
        self.playTimer.setSingleShot(True)
        self.playTimer.connect('timeout()', onPlayTimer)

        self.seekTimer = QtCore.QTimer()
        self.seekTimer.setSingleShot(True)
        self.seekTimer.connect('timeout()', seekPressTimeout)

        self.renderTimer = QtCore.QTimer()
        self.renderTimer.setSingleShot(True)
        self.renderTimer.connect('timeout()', forceRender)

    def createStatusBarWidgets(self):

        self.logoLabel = QtGui.QLabel()
        self.logoLabel.setPixmap(QtGui.QPixmap(":/VelodyneHDLPlugin/bottom_logo.png"))
        self.logoLabel.setScaledContents(True)

        self.filenameLabel = QtGui.QLabel()
        self.statusLabel = QtGui.QLabel()
        self.timeLabel = QtGui.QLabel()


class IconPaths(object):

    trailingFrames = ':/VelodyneHDLPlugin/trailingframes.png'
    play = ':/VelodyneHDLPlugin/media-playback-start.png'
    pause =':/VelodyneHDLPlugin/media-playback-pause.png'
    seekForward = ':/VelodyneHDLPlugin/media-seek-forward.png'
    seekForward2x = ':/VelodyneHDLPlugin/media-seek-forward-2x.png'
    seekForwardHalfx = ':/VelodyneHDLPlugin/media-seek-forward-0.5x.png'
    seekForwardQuarterx = ':/VelodyneHDLPlugin/media-seek-forward-0.25x.png'
    seekForward3x = ':/VelodyneHDLPlugin/media-seek-forward-3x.png'
    seekBackward = ':/VelodyneHDLPlugin/media-seek-backward.png'
    seekBackward2x = ':/VelodyneHDLPlugin/media-seek-backward-2x.png'
    seekBackward3x = ':/VelodyneHDLPlugin/media-seek-backward-3x.png'
    seekBackwardHalfx = ':/VelodyneHDLPlugin/media-seek-backward-0.5x.png'
    seekBackwardQuarterx = ':/VelodyneHDLPlugin/media-seek-backward-0.25x.png'

class GridProperties:

    def __init__(self):
        self.Normal = [0, 0, 0]
        self.Origin = [0, 0, 0]
        self.Scale = 0
        self.GridNbTicks = 0
        self.LineWidth = 0
        self.Color = [0, 0, 0]
        self.Persist = False



def hasArrayName(sourceProxy, arrayName):
    '''
    Returns True if the data has non-zero points and has a point data
    attribute with the given arrayName.
    '''
    if not sourceProxy:
        return False

    info = sourceProxy.GetDataInformation().DataInformation

    if info.GetNumberOfPoints() == 0:
        return False

    info = info.GetAttributeInformation(0)
    for i in xrange(info.GetNumberOfArrays()):
        if info.GetArrayInformation(i).GetName() == arrayName:
            return True
    return False


def openData(filename):

    close()

    reader = smp.OpenDataFile(filename, guiName='Data')

    if not reader:
        return

    rep = smp.Show(reader)
    rep.InterpolateScalarsBeforeMapping = 0
    setDefaultLookupTables(reader)
    colorByIntensity(reader)

    showSourceInSpreadSheet(reader)

    smp.GetActiveView().ViewTime = 0.0

    app.reader = reader
    app.filenameLabel.setText('File: %s' % os.path.basename(filename))

    updateSliderTimeRange()
    enablePlaybackActions()
    enableSaveActions()
    addRecentFile(filename)
    app.actions['actionSavePCAP'].setEnabled(False)
    app.actions['actionChoose_Calibration_File'].setEnabled(False)
    app.actions['actionCropReturns'].setEnabled(False)
    app.actions['actionRecord'].setEnabled(False)
    app.actions['actionDualReturnModeDual'].enabled = True
    app.actions['actionDualReturnDistanceNear'].enabled = True
    app.actions['actionDualReturnDistanceFar'].enabled = True
    app.actions['actionDualReturnIntensityHigh'].enabled = True
    app.actions['actionDualReturnIntensityLow'].enabled = True
    app.actions['actionShowRPM'].enabled = True

    resetCamera()


def planeFit():
    planefit.fitPlane()


def findPresetByName(name):
    presets = servermanager.vtkSMTransferFunctionPresets()

    numberOfPresets = presets.GetNumberOfPresets()

    for i in range(0,numberOfPresets):
        currentName = presets.GetPresetName(i)
        if currentName == name:
            return i

    return -1


def createDSRColorsPreset():

    dsrColorIndex = findPresetByName("DSR Colors")

    if dsrColorIndex == -1:
        rcolor = [0,        0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
                  0,         0,         0,         0,         0,         0,         0,         0,         0,    0.0625,    0.1250,    0.1875,    0.2500,    0.3125,    0.3750,
                  0.4375,    0.5000,    0.5625,    0.6250,    0.6875,    0.7500,    0.8125,    0.8750,    0.9375,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,
                  1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000 ,   1.0000,    1.0000,    0.9375,    0.8750,    0.8125,    0.7500,
                  0.6875,    0.6250,    0.5625,    0.5000]

        gcolor = [0,         0,         0,         0,         0,         0 ,        0,         0,    0.0625,    0.1250,    0.1875,    0.2500,    0.3125,    0.3750,    0.4375,
                  0.5000,    0.5625,    0.6250,    0.6875,    0.7500,    0.8125,    0.8750,    0.9375,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,
                  1.0000,    1.0000,    1.0000,    1.0000,   1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    0.9375,    0.8750,    0.8125,    0.7500,    0.6875,
                  0.6250,    0.5625,    0.5000,    0.4375,    0.3750,    0.3125,    0.2500,    0.1875,    0.1250,    0.0625,         0,         0,         0,         0,         0,
                  0,         0,         0,         0]

        bcolor = [0.5625,    0.6250,    0.6875,    0.7500,    0.8125,    0.8750,    0.9375,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,
                  1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    0.9375,    0.8750,    0.8125,    0.7500,    0.6875,    0.6250,
                  0.5625,    0.5000,    0.4375,    0.3750,    0.3125,    0.2500,   0.1875,    0.1250,    0.0625,         0,         0,         0,         0,         0,         0,
                  0,         0 ,        0,         0,         0,         0,         0,         0,         0 ,        0 ,        0 ,        0,         0 ,        0 ,        0,
                  0,         0,         0,         0]

        intensityColor = [0] * 256

        for i in range(0,63):
            index = i/63.0*255.0

            intensityColor[i*4] = index
            intensityColor[i*4+1] = rcolor[i]
            intensityColor[i*4+2] = gcolor[i]
            intensityColor[i*4+3] = bcolor[i]
            i = i + 1

        presets = servermanager.vtkSMTransferFunctionPresets()

        intensityString = ',\n'.join(map(str, intensityColor))

        intensityJSON = "{\n\"ColorSpace\" : \"RGB\",\n\"Name\" : \"DSR\",\n\"NanColor\" : [ 1, 1, 0 ],\n\"RGBPoints\" : [\n"+ intensityString + "\n]\n}"

        presets.AddPreset("DSR Colors",intensityJSON)


def setDefaultLookupTables(sourceProxy):
    createDSRColorsPreset()

    presets = servermanager.vtkSMTransferFunctionPresets()

    dsrIndex = findPresetByName("DSR Colors")
    presetDSR = presets.GetPresetAsString(dsrIndex)

    # LUT for 'intensity'
    smp.GetLookupTableForArray(
      'intensity', 1,
      ScalarRangeInitialized=1.0,
      ColorSpace='HSV',
      RGBPoints=[0.0, 0.0, 0.0, 1.0,
               100.0, 1.0, 1.0, 0.0,
               256.0, 1.0, 0.0, 0.0])

    # LUT for 'dual_distance'
    smp.GetLookupTableForArray(
      'dual_distance', 1,
      InterpretValuesAsCategories=True, NumberOfTableValues=3,
      RGBPoints=[-1.0, 0.1, 0.5, 0.7,
                  0.0, 0.9, 0.9, 0.9,
                 +1.0, 0.8, 0.2, 0.3],
      Annotations=['-1', 'near', '0', 'dual', '1', 'far'])

    # LUT for 'dual_intensity'
    smp.GetLookupTableForArray(
      'dual_intensity', 1,
      InterpretValuesAsCategories=True, NumberOfTableValues=3,
      RGBPoints=[-1.0, 0.5, 0.2, 0.8,
                  0.0, 0.6, 0.6, 0.6,
                 +1.0, 1.0, 0.9, 0.4],
      Annotations=['-1', 'low', '0', 'dual', '1', 'high'])

    # LUT for 'laser_id'. This LUT is extracted from the XML calibration file
    # which doesn't exist in live stream mode
    if False and getReader() is not None:
        rgbRaw = [0] * 256
        sourceProxy.GetClientSideObject().GetXMLColorTable(rgbRaw)

        smp.GetLookupTableForArray(
          'laser_id', 1,
          ScalarRangeInitialized=1.0,
          ColorSpace='RGB',
          RGBPoints=rgbRaw)

def colorByIntensity(sourceProxy):

    if not hasArrayName(sourceProxy, 'intensity'):
        return False

    setDefaultLookupTables(sourceProxy)
    rep = smp.GetDisplayProperties(sourceProxy)
    rep.ColorArrayName = 'intensity'
    rep.LookupTable = smp.GetLookupTableForArray('intensity', 1)
    return True


def getTimeStamp():
    format = '%Y-%m-%d-%H-%M-%S'
    return datetime.datetime.now().strftime(format)


def getReaderFileName():
    filename = getReader().FileName
    return filename[0] if isinstance(filename, servermanager.FileNameProperty) else filename


def getDefaultSaveFileName(extension, suffix='', appendFrameNumber=False):

    sensor = getSensor()
    reader = getReader()

    if sensor:
        nchannels =  sensor.GetPropertyValue('NumberOfChannels')
        base = 'HDL-'
        if nchannels <= 16:
            base = 'VLP-'
        sensortype = base + str(nchannels)

        return '%s_Velodyne-%s-Data.%s' % (getTimeStamp(), sensortype, extension)

    if reader:
        basename =  os.path.splitext(os.path.basename(getReaderFileName()))[0]
        if appendFrameNumber:
            suffix = '%s (Frame %04d)' % (suffix, int(app.scene.AnimationTime))
        return '%s%s.%s' % (basename, suffix, extension)


def chooseCalibration():

    class Calibration(object):
        def __init__(self, dialog):
            self.calibrationFile = dialog.selectedCalibrationFile()
            self.gpsYaw = dialog.gpsYaw()
            self.gpsRoll = dialog.gpsRoll()
            self.gpsPitch = dialog.gpsPitch()
            self.lidarPort = dialog.lidarPort()
            self.gpsPort = dialog.gpsPort()
            self.gpsForwardingPort = dialog.gpsForwardingPort()
            self.lidarForwardingPort = dialog.lidarForwardingPort()
            self.isForwarding = dialog.isForwarding()
            self.ipAddressForwarding = dialog.ipAddressForwarding()
            self.sensorTransform = vtk.vtkTransform()

            qm = dialog.sensorTransform()
            vm = vtk.vtkMatrix4x4()
            for row in xrange(4):
                vm.SetElement(row, 0, qm.row(row).x())
                vm.SetElement(row, 1, qm.row(row).y())
                vm.SetElement(row, 2, qm.row(row).z())
                vm.SetElement(row, 3, qm.row(row).w())
            self.sensorTransform.SetMatrix(vm)


    dialog = vvCalibrationDialog(getMainWindow())
    if not dialog.exec_():
        return None

    return Calibration(dialog)


def restoreLaserSelectionDialog():

    reopenLaserSelectionDialog = False
    isDisplayMoreSelectionsChecked = False

    if app.laserSelectionDialog != None:
        reopenLaserSelectionDialog = app.laserSelectionDialog.isVisible()
        isDisplayMoreSelectionsChecked = app.laserSelectionDialog.isDisplayMoreSelectionsChecked()
        app.laserSelectionDialog.close()
        app.laserSelectionDialog = None

    onLaserSelection(reopenLaserSelectionDialog)
    app.laserSelectionDialog.setDisplayMoreSelectionsChecked(isDisplayMoreSelectionsChecked)


def openSensor():

    calibration = chooseCalibration()
    if not calibration:
        return

    calibrationFile = calibration.calibrationFile
    sensorTransform = calibration.sensorTransform
    LIDARPort = calibration.lidarPort
    GPSPort = calibration.gpsPort
    LIDARForwardingPort = calibration.lidarForwardingPort
    GPSForwardingPort = calibration.gpsForwardingPort
    isForwarding = calibration.isForwarding
    ipAddressForwarding = calibration.ipAddressForwarding

    close()
    app.grid = createGrid()

    initializeRPMText()

    sensor = smp.VelodyneHDLSource(guiName='Data', CalibrationFile=calibrationFile, CacheSize=100)
    sensor.GetClientSideObject().SetLIDARPort(LIDARPort)
    sensor.GetClientSideObject().SetGPSPort(GPSPort)
    sensor.GetClientSideObject().SetForwardedGPSPort(GPSForwardingPort)
    sensor.GetClientSideObject().SetForwardedLIDARPort(LIDARForwardingPort)
    sensor.GetClientSideObject().SetisForwarding(isForwarding)
    sensor.GetClientSideObject().SetisCrashAnalysing(app.EnableCrashAnalysis)
    sensor.GetClientSideObject().SetForwardedIpAddress(ipAddressForwarding)
    sensor.GetClientSideObject().SetSensorTransform(sensorTransform)
    sensor.UpdatePipeline()
    sensor.Start()

    if SAMPLE_PROCESSING_MODE:
        processor = smp.ProcessingSample(sensor)

    smp.GetActiveView().ViewTime = 0.0

    app.sensor = sensor
    app.colorByInitialized = False
    app.filenameLabel.setText('Live sensor stream.')
    enablePlaybackActions()
    enableSaveActions()

    onCropReturns(False) # Dont show the dialog just restore settings
    onLaserSelection(False)

    rep = smp.Show(sensor)
    rep.InterpolateScalarsBeforeMapping = 0

    if SAMPLE_PROCESSING_MODE:
        prep = smp.Show(processor)
    smp.Render()

    showSourceInSpreadSheet(sensor)

    app.actions['actionShowRPM'].enabled = True
    app.actions['actionCorrectIntensityValues'].enabled = True

    #Auto adjustment of the grid size with the distance resolution
    app.distanceResolutionM = sensor.GetClientSideObject().GetDistanceResolutionM()
    app.actions['actionMeasurement_Grid'].setChecked(True)
    showMeasurementGrid()

    play()

    # Always enable dual return mode selection. A warning will be raised if
    # there's no dual return on the current frame later on
    app.actions['actionDualReturnModeDual'].enabled = True
    app.actions['actionDualReturnDistanceNear'].enabled = True
    app.actions['actionDualReturnDistanceFar'].enabled = True
    app.actions['actionDualReturnIntensityHigh'].enabled = True
    app.actions['actionDualReturnIntensityLow'].enabled = True

    sensor.GetClientSideObject().SetIgnoreZeroDistances(app.actions['actionIgnoreZeroDistances'].isChecked())
    sensor.GetClientSideObject().SetIntraFiringAdjust(app.actions['actionIntraFiringAdjust'].isChecked())
    sensor.GetClientSideObject().SetIgnoreEmptyFrames(app.actions['actionIgnoreEmptyFrames'].isChecked())


def openPCAP(filename, positionFilename=None):

    calibration = chooseCalibration()
    if not calibration:
        return

    calibrationFile = calibration.calibrationFile
    sensorTransform = calibration.sensorTransform

    close()

    def onProgressEvent(o, e):
        PythonQt.QtGui.QApplication.instance().processEvents()

    progressDialog = QtGui.QProgressDialog('Reading packet file...', '', 0, 0, getMainWindow())
    progressDialog.setCancelButton(None)
    progressDialog.setModal(True)
    progressDialog.show()

    handler = servermanager.ActiveConnection.Session.GetProgressHandler()
    handler.PrepareProgress()
    freq = handler.GetProgressFrequency()
    handler.SetProgressFrequency(0.05)
    tag = handler.AddObserver('ProgressEvent', onProgressEvent)

    # construct the reader, this calls UpdateInformation on the
    # reader which scans the pcap file and emits progress events
    reader = smp.VelodyneHDLReader(guiName='Data',
                                   FileName=filename,
                                   CalibrationFile=calibrationFile,
                                   ApplyTransform=(app.transformMode > 0),
                                   NumberOfTrailingFrames=app.trailingFramesSpinBox.value,
                                   PointsSkip=app.trailingFramesSpinBox.value)

    app.reader = reader
    app.filenameLabel.setText('File: %s' % os.path.basename(filename))
    onCropReturns(False) # Dont show the dialog just restore settings

    # Resetting laser selection dialog according to the opened PCAP file
    # and restoring the dialog visibility afterward

    restoreLaserSelectionDialog()

    reader.GetClientSideObject().SetSensorTransform(sensorTransform)

    if SAMPLE_PROCESSING_MODE:
        processor = smp.ProcessingSample(reader)

    handler.RemoveObserver(tag)
    handler.SetProgressFrequency(freq)
    progressDialog.close()

    smp.GetActiveView().ViewTime = 0.0

    rep = smp.Show(reader)
    if SAMPLE_PROCESSING_MODE:
        prep = smp.Show(processor)
    app.scene.UpdateAnimationUsingDataTimeSteps()

    # update overhead view
    smp.SetActiveView(app.overheadView)

    if positionFilename is None:
        posreader = smp.VelodyneHDLPositionReader(guiName="Position",
                                                  FileName=filename)
    else:
        posreader = smp.ApplanixPositionReader(guiName="Position",
                                               FileName=positionFilename)
        posreader.BaseYaw = calibration.gpsYaw
        posreader.BaseRoll = calibration.gpsRoll
        posreader.BasePitch = calibration.gpsPitch

    posreader.GetClientSideObject().SetShouldWarnOnWeirdGPSData(app.geolocationToolBar.visible)
    smp.Show(posreader)

    # Create a sphere glyph
    g = smp.Sphere()
    g.Radius = 5.0
    smp.Show(g)

    if posreader.GetClientSideObject().GetOutput().GetNumberOfPoints():
        reader.GetClientSideObject().SetInterpolator(
            posreader.GetClientSideObject().GetInterpolator())

        smp.Render(app.overheadView)
        app.overheadView.ResetCamera()

        trange = posreader.GetPointDataInformation().GetArray('time').GetRange()

        # By construction time zero is at position 0,0,0

        # Setup scalar bar
        rep = smp.GetDisplayProperties(posreader)
        rep.ColorArrayName = 'time'
        rgbPoints = [trange[0], 0.0, 0.0, 1.0,
                     trange[1], 1.0, 0.0, 0.0]
        rep.LookupTable = smp.GetLookupTableForArray('time', 1,
                                                     RGBPoints=rgbPoints,
                                                     ScalarRangeInitialized=1.0)
        sb = smp.CreateScalarBar(LookupTable=rep.LookupTable, Title='Time')
        sb.Orientation = 'Horizontal'
        sb.Position, sb.Position2 = [.1, .05], [.8, .02]
        app.overheadView.Representations.append(sb)

        app.position = (posreader, None, g)
        smp.Render(app.overheadView)
    else:
        if positionFilename is not None:
            QtGui.QMessageBox.warning(getMainWindow(), 'Georeferncing data invalid',
                                      'File %s is not supported' % positionFilename)

        smp.Delete(posreader)

    smp.SetActiveView(app.mainView)

    rep.InterpolateScalarsBeforeMapping = 0
    setDefaultLookupTables(reader)
    colorByIntensity(reader)

    initializeRPMText()

    showSourceInSpreadSheet(reader)

    updateSliderTimeRange()
    enablePlaybackActions()
    enableSaveActions()
    addRecentFile(filename)
    app.actions['actionRecord'].setEnabled(False)

    # Always enable dual return mode selection. A warning will be raised if
    # there's no dual return on the current frame later on
    app.actions['actionSelectDualReturn'].enabled = True
    app.actions['actionSelectDualReturn2'].enabled = True
    app.actions['actionDualReturnModeDual'].enabled = True
    app.actions['actionDualReturnDistanceNear'].enabled = True
    app.actions['actionDualReturnDistanceFar'].enabled = True
    app.actions['actionDualReturnIntensityHigh'].enabled = True
    app.actions['actionDualReturnIntensityLow'].enabled = True

    app.actions['actionShowRPM'].enabled = True
    app.actions['actionCorrectIntensityValues'].enabled = True

    reader.GetClientSideObject().SetIgnoreZeroDistances(app.actions['actionIgnoreZeroDistances'].isChecked())
    reader.GetClientSideObject().SetIntraFiringAdjust(app.actions['actionIntraFiringAdjust'].isChecked())
    reader.GetClientSideObject().SetIgnoreEmptyFrames(app.actions['actionIgnoreEmptyFrames'].isChecked())

    #Auto adjustment of the grid size with the distance resolution
    app.distanceResolutionM = reader.GetClientSideObject().GetDistanceResolutionM()
    app.grid = createGrid()
    app.actions['actionMeasurement_Grid'].setChecked(True)
    showMeasurementGrid()

    smp.SetActiveSource(reader)
    updatePosition()
    resetCamera()
    
    #Remove the Rotation per minute from color label comboBox
    ComboBox = getMainWindow().findChild('vvColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[0]
    n = ComboBox.findText('RotationPerMinute')
    ComboBox.removeItem(n)



def hideMeasurementGrid():
    rep = smp.GetDisplayProperties(app.grid)
    rep.Visibility = 0
    smp.Render()


def showMeasurementGrid():
    rep = smp.GetDisplayProperties(app.grid)
    rep.Visibility = 1
    smp.Render()


# Start Functions related to ruler


def createRuler():
    pxm = servermanager.ProxyManager()
    distancerep = pxm.NewProxy('representations', 'DistanceWidgetRepresentation')
    distancerepeasy = servermanager._getPyProxy(distancerep)
    smp.GetActiveView().Representations.append(distancerepeasy)
    distancerepeasy.Visibility = False
    smp.Render()

    return distancerepeasy


def hideRuler():
    app.ruler.Visibility = False
    smp.Render()


def showRuler():
    app.ruler.Visibility = True
    smp.Render()

def getPointFromCoordinate(coord, midPlaneDistance = 0.5):
    assert len(coord) == 2

    windowHeight = smp.GetActiveView().ViewSize[1]

    displayPoint = [coord[0], windowHeight - coord[1], midPlaneDistance]
    renderer = smp.GetActiveView().GetRenderer()
    renderer.SetDisplayPoint(displayPoint)
    renderer.DisplayToWorld()
    world1 = renderer.GetWorldPoint()

    return world1[:3]

def toggleRulerContext():

    measurmentState = app.actions['actionMeasure'].isChecked()

    mW = getMainWindow()
    vtkW = mW.findChild('pqQVTKWidget')

    if measurmentState == True:
        vtkW.connect('mouseEvent(QMouseEvent*)', setRulerCoordinates)

    elif measurmentState == False:
        vtkW.disconnect('mouseEvent(QMouseEvent*)', setRulerCoordinates)

        app.mousePressed = False
        hideRuler()


def setRulerCoordinates(mouseEvent):

    pqView = smp.GetActiveView()
    rW = pqView.GetRenderWindow()
    windowInteractor = rW.GetInteractor()
    currentMouseState = mouseEvent.buttons()
    currentKeyboardState = mouseEvent.modifiers()

    if currentMouseState == 1:  #Left button pressed

        if app.mousePressed == False: #For the first time

            if currentKeyboardState == 67108864: #Control key pressed

                app.mousePressed = True
                app.ruler.Point1WorldPosition = getPointFromCoordinate([mouseEvent.x(),mouseEvent.y()])

                windowInteractor.Disable()

        elif app.mousePressed == True: #Not for the first time

            app.ruler.Point2WorldPosition = getPointFromCoordinate([mouseEvent.x(),mouseEvent.y()])
            showRuler()
            smp.Render()

    elif currentMouseState == 0: #Left button released

        windowInteractor.Enable()

        if  app.mousePressed == True: #For the first time

            app.mousePressed = False
            app.ruler.Point2WorldPosition = getPointFromCoordinate([mouseEvent.x(),mouseEvent.y()])
            showRuler()
            smp.Render()


# End Functions related to ruler


def rotateCSVFile(filename):

    # read the csv file, move the last 3 columns to the
    # front, and then overwrite the file with the result
    csvFile = open(filename, 'rb')
    reader = csv.reader(csvFile, quoting=csv.QUOTE_NONNUMERIC)
    rows = [row[-3:] + row[:-3] for row in reader]
    csvFile.close()

    writer = csv.writer(open(filename, 'wb'), quoting=csv.QUOTE_NONNUMERIC, delimiter=',')
    writer.writerows(rows)


def savePositionCSV(filename):
    w = smp.CreateWriter(filename, getPosition())
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)

def saveCSVCurrentFrame(filename):
    w = smp.CreateWriter(filename, smp.GetActiveSource())
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)
    rotateCSVFile(filename)

def saveCSVCurrentFrameSelection(filename):
    source = smp.GetActiveSource()
    selection = source.GetSelectionOutput(0)
    extractSelection = smp.ExtractSelection(Input = source, Selection = selection.Selection)
    w = smp.CreateWriter(filename, extractSelection)
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)
    rotateCSVFile(filename)

def saveLASFrames(filename, first, last, transform):
    reader = getReader().GetClientSideObject()
    position = getPosition().GetClientSideObject().GetOutput()

    PythonQt.paraview.pqVelodyneManager.saveFramesToLAS(
        reader, position, first, last, filename, transform)


def saveLASCurrentFrame(filename, transform):
    t = app.scene.AnimationTime
    saveLASFrames(filename, t, t, transform)


def saveAllFrames(filename, saveFunction):
    saveFunction(filename, getCurrentTimesteps())


def saveFrameRange(filename, frameStart, frameStop, saveFunction):
    timesteps = range(frameStart, frameStop+1)
    saveFunction(filename, timesteps)


def saveCSV(filename, timesteps):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    basenameWithoutExtension = os.path.splitext(os.path.basename(filename))[0]
    outDir = os.path.join(tempDir, basenameWithoutExtension)
    filenameTemplate = os.path.join(outDir, basenameWithoutExtension + ' (Frame %04d).csv')
    os.makedirs(outDir)

    writer = smp.CreateWriter('tmp.csv', getSensor() or getReader())
    writer.FieldAssociation = 'Points'
    writer.Precision = 16

    for t in timesteps:
        app.scene.AnimationTime = t
        writer.FileName = filenameTemplate % t
        writer.UpdatePipeline()
        rotateCSVFile(writer.FileName)

    smp.Delete(writer)

    kiwiviewerExporter.zipDir(outDir, filename)
    kiwiviewerExporter.shutil.rmtree(tempDir)


def saveLAS(filename, timesteps, transform):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    basenameWithoutExtension = os.path.splitext(os.path.basename(filename))[0]
    outDir = os.path.join(tempDir, basenameWithoutExtension)
    filenameTemplate = os.path.join(outDir, basenameWithoutExtension + ' (Frame %04d).csv')
    os.makedirs(outDir)

    for t in sorted(timesteps):
        saveLASFrames(filenameTemplate % t, t, t, transform)

    kiwiviewerExporter.zipDir(outDir, filename)
    kiwiviewerExporter.shutil.rmtree(tempDir)


def getTimeStamp():
    format = '%Y-%m-%d-%H-%M-%S'
    return datetime.datetime.now().strftime(format)


def getSaveFileName(title, extension, defaultFileName=None):

    settings = getPVSettings()
    defaultDir = settings.value('VelodyneHDLPlugin/OpenData/DefaultDir', QtCore.QDir.homePath())
    defaultFileName = defaultDir if not defaultFileName else os.path.join(defaultDir, defaultFileName)

    nativeDialog = 0 if app.actions['actionNative_File_Dialogs'].isChecked() else QtGui.QFileDialog.DontUseNativeDialog

    filters = '%s (*.%s)' % (extension, extension)
    selectedFilter = '*.%s' % extension
    fileName = QtGui.QFileDialog.getSaveFileName(getMainWindow(), title,
                        defaultFileName, filters, selectedFilter, nativeDialog)

    if fileName:
        settings.setValue('VelodyneHDLPlugin/OpenData/DefaultDir', QtCore.QFileInfo(fileName).absoluteDir().absolutePath())
        return fileName


def restoreNativeFileDialogsAction():
    settings = getPVSettings()
    app.actions['actionNative_File_Dialogs'].setChecked(int(settings.value('VelodyneHDLPlugin/NativeFileDialogs', 1)))


def onNativeFileDialogsAction():
    settings = getPVSettings()
    defaultDir = settings.setValue('VelodyneHDLPlugin/NativeFileDialogs', int(app.actions['actionNative_File_Dialogs'].isChecked()))


def getFrameSelectionFromUser(frameStrideVisibility=False, framePackVisibility=False, frameTransformVisibility=False):
    class FrameOptions(object):
        pass

    dialog = PythonQt.paraview.vvSelectFramesDialog(getMainWindow())
    dialog.frameMinimum = app.scene.StartTime
    dialog.frameMaximum = app.scene.EndTime
    dialog.frameStrideVisibility = frameStrideVisibility
    dialog.framePackVisibility = framePackVisibility
    dialog.frameTransformVisibility = frameTransformVisibility
    dialog.restoreState()

    if not dialog.exec_():
        return None

    frameOptions = FrameOptions()
    frameOptions.mode = dialog.frameMode
    frameOptions.start = dialog.frameStart
    frameOptions.stop = dialog.frameStop
    frameOptions.stride = dialog.frameStride
    frameOptions.pack = dialog.framePack
    frameOptions.transform = dialog.frameTransform

    dialog.setParent(None)

    return frameOptions


def onSaveCSV():

    frameOptions = getFrameSelectionFromUser()
    if frameOptions is None:
        return


    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', appendFrameNumber=True))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            saveCSVCurrentFrame(fileName)

            setTransformMode(oldTransform)

    else:
        fileName = getSaveFileName('Save CSV (to zip file)', 'zip', getDefaultSaveFileName('zip'))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            if frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
                saveAllFrames(fileName, saveCSV)
            else:
                start = frameOptions.start
                stop = frameOptions.stop
                saveFrameRange(fileName, start, stop, saveCSV)

            setTransformMode(oldTransform)


def onSavePosition():
    fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', '-position'))
    if fileName:
        savePositionCSV(fileName)


def onSaveLAS():

    frameOptions = getFrameSelectionFromUser(framePackVisibility=True, frameTransformVisibility=False)
    if frameOptions is None:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        frameOptions.start = frameOptions.stop = app.scene.AnimationTime
    elif frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
        frameOptions.start = int(app.scene.StartTime)
        frameOptions.stop = int(app.scene.EndTime)

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        fileName = getSaveFileName('Save LAS', 'las', getDefaultSaveFileName('las', appendFrameNumber=True))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            saveLASCurrentFrame(fileName, frameOptions.transform)

            setTransformMode(oldTransform)

    elif frameOptions.pack == vvSelectFramesDialog.FILE_PER_FRAME:
        fileName = getSaveFileName('Save LAS (to zip file)', 'zip',
                                   getDefaultSaveFileName('zip'))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            def saveTransformedLAS(filename, timesteps):
                saveLAS(filename, timesteps, frameOptions.transform)

            if frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
                saveAllFrames(fileName, saveTransformedLAS)
            else:
                start = frameOptions.start
                stop = frameOptions.stop
                saveFrameRange(fileName, start, stop, saveTransformedLAS)

            setTransformMode(oldTransform)

    else:
        suffix = ' (Frame %d to %d)' % (frameOptions.start, frameOptions.stop)
        defaultFileName = getDefaultSaveFileName('las', suffix=suffix)
        fileName = getSaveFileName('Save LAS', 'las', defaultFileName)
        if not fileName:
            return

        oldTransform = transformMode()
        setTransformMode(1 if frameOptions.transform else 0)

        saveLASFrames(fileName, frameOptions.start, frameOptions.stop,
                      frameOptions.transform)

        setTransformMode(oldTransform)


def onSavePCAP():

    frameOptions = getFrameSelectionFromUser(frameTransformVisibility=False)
    if frameOptions is None:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        frameOptions.start = frameOptions.stop = app.scene.AnimationTime
    elif frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
        frameOptions.start = int(app.scene.StartTime)
        frameOptions.stop = int(app.scene.EndTime)

    defaultFileName = getDefaultSaveFileName('pcap', suffix=' (Frame %d to %d)' % (frameOptions.start, frameOptions.stop))
    fileName = getSaveFileName('Save PCAP', 'pcap', defaultFileName)
    if not fileName:
        return

    PythonQt.paraview.pqVelodyneManager.saveFramesToPCAP(getReader().SMProxy, frameOptions.start, frameOptions.stop, fileName)


def onSaveScreenshot():

    fileName = getSaveFileName('Save Screenshot', 'png', getDefaultSaveFileName('png', appendFrameNumber=True))
    if fileName:
        saveScreenshot(fileName)


def onKiwiViewerExport():

    frameOptions = getFrameSelectionFromUser(frameStrideVisibility=True,
                                             frameTransformVisibility=False)
    if frameOptions is None:
        return

    defaultFileName = getDefaultSaveFileName('zip', suffix=' (KiwiViewer)')
    fileName = getSaveFileName('Export To KiwiViewer', 'zip', defaultFileName)
    if not fileName:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        timesteps = [app.scene.AnimationTime]
    elif frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
        timesteps = range(int(app.scene.StartTime), int(app.scene.EndTime) + 1, frameOptions.stride)
    else:
        timesteps = range(frameOptions.start, frameOptions.stop+1, frameOptions.stride)

    saveToKiwiViewer(fileName, timesteps)


def saveToKiwiViewer(filename, timesteps):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    outDir = os.path.join(tempDir, os.path.splitext(os.path.basename(filename))[0])

    os.makedirs(outDir)

    filenames = exportToDirectory(outDir, timesteps)

    kiwiviewerExporter.writeJsonData(outDir, smp.GetActiveView(), smp.GetDisplayProperties(), filenames)

    kiwiviewerExporter.zipDir(outDir, filename)
    kiwiviewerExporter.shutil.rmtree(tempDir)


def exportToDirectory(outDir, timesteps):

    filenames = []

    alg = smp.GetActiveSource().GetClientSideObject()

    writer = vtkXMLPolyDataWriter()
    writer.SetDataModeToAppended()
    writer.EncodeAppendedDataOff()
    writer.SetCompressorTypeToZLib()

    for t in timesteps:

        filename = 'frame_%04d.vtp' % t
        filenames.append(filename)

        app.scene.AnimationTime = t
        polyData = vtk.vtkPolyData()
        polyData.ShallowCopy(alg.GetOutput())

        writer.SetInputData(polyData)
        writer.SetFileName(os.path.join(outDir, filename))
        writer.Update()

    return filenames


def getVersionString():
  return " ".join(getMainWindow().windowTitle.split(" ")[1:])


def onDeveloperGuide():
    basePath = PythonQt.QtGui.QApplication.instance().applicationDirPath()

    paths = ['../Resources/VeloView_Developer_Guide.pdf']

    for path in paths:
        filename = os.path.join(basePath, path)
        if os.path.isfile(filename):
            QtGui.QDesktopServices.openUrl(QtCore.QUrl('file:///%s' % filename, QtCore.QUrl.TolerantMode))

def onUserGuide():
    basePath = PythonQt.QtGui.QApplication.instance().applicationDirPath()

    paths = ['../Resources/VeloView_User_Guide.pdf']

    for path in paths:
        filename = os.path.join(basePath, path)
        if os.path.isfile(filename):
            QtGui.QDesktopServices.openUrl(QtCore.QUrl('file:///%s' % filename, QtCore.QUrl.TolerantMode))

def onAbout():
    aboutDialog.showDialog(getMainWindow())


def close():
    # Save grid properties for this session
    app.gridProperties.Normal = app.grid.Normal
    app.gridProperties.Origin = app.grid.Origin
    app.gridProperties.Scale = app.grid.Scale
    app.gridProperties.GridNbTicks = app.grid.GridNbTicks
    app.gridProperties.LineWidth = app.grid.LineWidth
    app.gridProperties.Color = app.grid.Color

    stop()
    hideRuler()
    unloadData()
    smp.Render(app.overheadView)
    app.scene.AnimationTime = 0
    app.reader = None
    app.sensor = None
    if app.text:
        smp.Delete(app.text)
    smp.Delete(app.grid)

    smp.HideUnusedScalarBars()

    resetCameraToForwardView()
    app.filenameLabel.setText('')
    app.statusLabel.setText('')
    app.timeLabel.setText('')
    updateSliderTimeRange()
    disablePlaybackActions()
    disableSaveActions()
    app.actions['actionRecord'].setChecked(False)
    app.actions['actionDualReturnModeDual'].setChecked(True)

    app.actions['actionSelectDualReturn'].enabled = False
    app.actions['actionSelectDualReturn2'].enabled = False
    app.actions['actionDualReturnModeDual'].enabled = False
    app.actions['actionDualReturnDistanceNear'].enabled = False
    app.actions['actionDualReturnDistanceFar'].enabled = False
    app.actions['actionDualReturnIntensityHigh'].enabled = False
    app.actions['actionDualReturnIntensityLow'].enabled = False
    app.actions['actionCorrectIntensityValues'].enabled = False


def seekForward():

    if app.playing:
        if app.playDirection < 0 or app.playDirection == 5:
            app.playDirection = 0
        app.playDirection += 1
        updateSeekButtons()

    else:
        gotoNext()


def seekBackward():

    if app.playing:
        if app.playDirection > 0 or app.playDirection == -5:
            app.playDirection = 0
        app.playDirection -= 1
        updateSeekButtons()

    else:
        gotoPrevious()


def seekPressTimeout():
    app.seekPlay = True
    onPlayTimer()


def seekForwardPressed():
    app.seekPlayDirection = 1
    if not app.playing:
        app.seekTimer.start(500)


def seekForwardReleased():
    app.seekTimer.stop()
    app.seekPlay = False


def seekBackwardPressed():
    app.seekPlayDirection = -1
    if not app.playing:
        app.seekTimer.start(500)


def seekBackwardReleased():
    seekForwardReleased()


def updateSeekButtons():

    icons = {
              -5 : IconPaths.seekBackwardQuarterx,
              -4 : IconPaths.seekBackwardHalfx,
              -3 : IconPaths.seekBackward3x,
              -2 : IconPaths.seekBackward2x,
              -1 : IconPaths.seekBackward,
               1 : IconPaths.seekForward,
               2 : IconPaths.seekForward2x,
               3 : IconPaths.seekForward3x,
               4 : IconPaths.seekForwardHalfx,
               5 : IconPaths.seekForwardQuarterx,
            }

    setActionIcon('actionSeek_Backward', icons[app.playDirection] if app.playDirection < 0 else IconPaths.seekBackward)
    setActionIcon('actionSeek_Forward', icons[app.playDirection] if app.playDirection > 0 else IconPaths.seekForward)

    fpsMap = {-5:5, 5:5, -4:11, 4:11}
    fpsDefault = 30
    app.targetFps = fpsMap.get(app.playDirection, fpsDefault)


def setPlaybackActionsEnabled(enabled):
    for action in ('Play', 'Record', 'Seek_Forward', 'Seek_Backward', 'Go_To_Start', 'Go_To_End'):
        app.actions['action'+action].setEnabled(enabled)


def enablePlaybackActions():
    setPlaybackActionsEnabled(True)


def disablePlaybackActions():
    setPlaybackActionsEnabled(False)


def _setSaveActionsEnabled(enabled):
    for action in ('SaveCSV', 'SavePCAP', 'SaveLAS', 'Export_To_KiwiViewer',
                   'Close', 'Choose_Calibration_File', 'CropReturns'):
        app.actions['action'+action].setEnabled(enabled)
    getMainWindow().findChild('QMenu', 'menuSaveAs').enabled = enabled


def enableSaveActions():
    _setSaveActionsEnabled(True)
    if getPosition():
        app.actions['actionSavePositionCSV'].setEnabled(True)


def disableSaveActions():
    _setSaveActionsEnabled(False)
    app.actions['actionSavePositionCSV'].setEnabled(False)


def recordFile(filename):

    sensor = getSensor()
    if sensor:
        stopStream()
        sensor.OutputFile = filename
        app.statusLabel.setText('  Recording file: %s.' % os.path.basename(filename))
        if app.playing:
            startStream()


def onRecord():

    recordAction = app.actions['actionRecord']

    if not recordAction.isChecked():
        stopRecording()

    else:

        fileName = getSaveFileName('Choose Output File', 'pcap', getDefaultSaveFileName('pcap'))
        if not fileName:
            recordAction.setChecked(False)
            return

        recordFile(fileName)


def stopRecording():

    app.statusLabel.setText('')
    sensor = getSensor()
    if sensor:
        stopStream()
        sensor.OutputFile = ''
        if app.playing:
            startStream()


def startStream():

    sensor = getSensor()
    if sensor:
        sensor.Start()


def stopStream():
    sensor = getSensor()
    if sensor:
        sensor.Stop()


def pollSource():

    source = getSensor()
    reader = getReader()
    if source is not None:
        source.Poll()
        source.UpdatePipelineInformation()
    return source or reader


def getPointCloudData(attribute=None):

    if attribute is not None:
        data = getPointCloudData()
        if data:
            if attribute == 'points':
                return data.GetPoints().GetData()
            else:
                return data.GetPointData().GetArray(attribute)
    else:
        source = getSensor() or getReader()
        if source:
            return source.GetClientSideObject().GetOutput()


def getCurrentTimesteps():
    source = pollSource()
    return list(source.TimestepValues) if source is not None else []


def getNumberOfTimesteps():
    return getTimeKeeper().getNumberOfTimeStepValues()


def togglePlay():
    setPlayMode(not app.playing)

def play():
    setPlayMode(True)


def stop():
    setPlayMode(False)


def onPlayTimer():

    if app.playing or app.seekPlay:

        startTime = vtk.vtkTimerLog.GetUniversalTime()

        playbackTick()

        rpmArray = None
        targetPlaybackFps = app.targetFps
        defaultRPM = 600
        defaultFps = 200
        currentFrameOriginalFps = defaultRPM / 60
        # if the rpm is below this value, we consider
        # that the computation has failed
        minimalRPM = 10

        # if we are in playback mode, compute the targetFps
        # using the rpm of the current frame
        if getReader():
            # Get the computed rpm of the current frame
            rpmArray = getReader().GetClientSideObject().GetOutput().GetFieldData().GetArray('RotationPerMinute')

            # If the rpm is available
            if rpmArray:
                rpm = rpmArray.GetTuple1(0)
                if rpm > minimalRPM:
                    currentFrameOriginalFps = rpm / 60

            # Get the playback speed multiplier
            playbackSpeed = str(app.PlaybackSpeed.currentText)

            canConvertToFloat = True
            try:
                float(playbackSpeed)
            except ValueError:
                canConvertToFloat = False

            # if the speed multiplier is set to default, we
            # use the default target fps. Otherwise, we apply
            # the multiplier to the current frame real fps
            if (playbackSpeed == 'default') or (not canConvertToFloat):
                targetPlaybackFps = defaultFps
            else:
                speedMultiplier = float(playbackSpeed)
                targetPlaybackFps = speedMultiplier * currentFrameOriginalFps

        # if we are in live mode, we want to read the frames using
        # the standard fps
        if getSensor():
            targetPlaybackFps = defaultFps

        fpsDelayMilliseconds = int(1000.0 / targetPlaybackFps)
        elapsedMilliseconds = int((vtk.vtkTimerLog.GetUniversalTime() - startTime)*1000.0)

        if elapsedMilliseconds > 0:
            fps = 1000.0/elapsedMilliseconds
            app.fps[0] += fps
            app.fps[1] += 1

        waitMilliseconds = fpsDelayMilliseconds - elapsedMilliseconds
        app.playTimer.start(max(waitMilliseconds,0))


def setPlayMode(mode):

    if not getReader() and not getSensor():
        return

    app.playing = mode

    if mode:
        startStream()
        setActionIcon('actionPlay', IconPaths.pause)
        app.playTimer.start(33)
        if app.scene.AnimationTime == app.scene.EndTime:
            app.scene.AnimationTime = app.scene.StartTime

    else:
        stopStream()
        setActionIcon('actionPlay', IconPaths.play)
        app.playDirection = 1
        updateSeekButtons()


def gotoStart():
    pollSource()
    app.scene.GoToFirst()
    updatePosition()


def gotoEnd():
    pollSource()
    app.scene.GoToLast()
    updatePosition()


def gotoNext():
    pollSource()
    app.scene.GoToNext()
    updatePosition()


def gotoPrevious():
    pollSource()
    app.scene.GoToPrevious()
    updatePosition()


def updatePosition():
    reader = getReader()
    pos = getPosition()

    if reader and pos:
        pointcloud = reader.GetClientSideObject().GetOutput()

        if pointcloud.GetNumberOfPoints():
            # Update the overhead view
            # TODO: Approximate time, just grabbing the last
            t = pointcloud.GetPointData().GetScalars('adjustedtime')
            #currentTime = t.GetTuple1(t.GetNumberOfTuples() - 1)
            currentTime = t.GetTuple1(0) * 1e-6

            interp = getPosition().GetClientSideObject().GetInterpolator()
            trange = [interp.GetMinimumT(), interp.GetMaximumT()]

            # Clamp
            currentTime = min(max(currentTime, trange[0]+1.0e-1), trange[1]-1.0e-1)

            position = [0.0] * 3
            transform = vtk.vtkTransform()
            interp.InterpolateTransform(currentTime, transform)
            transform.TransformPoint(position, position)

            rep = cachedGetRepresentation(reader, view=app.mainView)
            if app.relativeTransform:
                rep.Position = transform.GetInverse().GetPosition()
                rep.Orientation = transform.GetInverse().GetOrientation()
            else:
                rep.Position = [0.0, 0.0, 0.0]
                rep.Orientation = [0.0, 0.0, 0.0]

            g = getGlyph()
            rep = cachedGetRepresentation(g, view=app.overheadView)
            rep.Position = position[:3]

    showRPM()


def playbackTick():

    sensor = getSensor()
    reader = getReader()
    view = smp.GetActiveView()

    if sensor is not None:
        timesteps = getCurrentTimesteps()
        if not timesteps:
            return

        if view.ViewTime == timesteps[-1]:
            return

        if not app.colorByInitialized:
            sensor.UpdatePipeline()
            if colorByIntensity(sensor):
                app.colorByInitialized = True
                resetCamera()

        app.scene.GoToLast()

    elif reader is not None:

        numberOfTimesteps = getNumberOfTimesteps()
        if not numberOfTimesteps:
            return

        step = app.seekPlayDirection if app.seekPlay else app.playDirection
        stepMap = {4:1, 5:1, -4:-1, -5:-1}
        step = stepMap.get(step, step)
        newTime = app.scene.AnimationTime + step

        if app.actions['actionLoop'].isChecked():
            newTime = newTime % numberOfTimesteps
        else:
            newTime = max(app.scene.StartTime, min(newTime, app.scene.EndTime))

            # stop playback when it reaches the first or last timestep
            if newTime in (app.scene.StartTime, app.scene.EndTime):
                stop()

        app.scene.AnimationTime = newTime
        # TODO: For sensor as well?
        updatePosition()


def unloadData():
    _repCache.clear()

    for k, src in smp.GetSources().iteritems():
        if src != app.grid:
            smp.Delete(src)

    toremove = [x for x in app.overheadView.Representations if type(x) == servermanager.rendering.ScalarBarWidgetRepresentation]
    for t in toremove:
        app.overheadView.Representations.remove(t)

    app.reader = None
    app.position = (None, None, None)
    app.sensor = None

    clearSpreadSheetView()

def getReader():
    return getattr(app, 'reader', None)


def getSensor():
    return getattr(app, 'sensor', None)

def getPosition():
    return getattr(app, 'position', (None, None, None))[0]

def getGlyph():
    return getattr(app, 'position', (None, None, None))[2]

def getLaserSelectionDialog():
    return getattr(app, 'laserSelectionDialog', None)

def onChooseCalibrationFile():

    calibration = chooseCalibration()
    if not calibration:
        return

    calibrationFile = calibration.calibrationFile
    sensorTransform = calibration.sensorTransform

    reader = getReader()
    sensor = getSensor()

    if reader is not None:
        reader.GetClientSideObject().SetSensorTransform(sensorTransform)
        reader.CalibrationFile = calibrationFile
        reloadCurrentFrame()

    elif sensor is not None:
        sensor.GetClientSideObject().SetSensorTransform(sensorTransform)
        sensor.CalibrationFile = calibrationFile
        # no need to render now, calibration file will be used on the next frame

    restoreLaserSelectionDialog()


def onCropReturns(show = True):
    dialog = vvCropReturnsDialog(getMainWindow())

    cropEnabled = False
    cropOutside = False
    firstCorner = QtGui.QVector3D()
    secondCorner = QtGui.QVector3D()

    reader = getReader()
    sensor = getSensor()

    if reader is not None:
        cropEnabled = reader.CropReturns
        cropOutside = reader.CropOutside
        firstCorner = QtGui.QVector3D(reader.CropRegion[0], reader.CropRegion[2], reader.CropRegion[4])
        secondCorner = QtGui.QVector3D(reader.CropRegion[1], reader.CropRegion[3], reader.CropRegion[5])

    if sensor is not None:
        cropEnabled = sensor.CropReturns
        cropOutside = sensor.CropOutside
        firstCorner = QtGui.QVector3D(sensor.CropRegion[0], sensor.CropRegion[2], sensor.CropRegion[4])
        secondCorner = QtGui.QVector3D(sensor.CropRegion[1], sensor.CropRegion[3], sensor.CropRegion[5])

    #show the dialog box
    if show:
        dialog.cropOutside = cropOutside
        dialog.firstCorner = firstCorner
        dialog.secondCorner = secondCorner
        dialog.croppingEnabled = cropEnabled
        # Enforce the call to dialog.croppingEnabled."onChanged" even if dialog.croppingEnabled == cropEnabled
        dialog.croppingEnabled = not dialog.croppingEnabled
        dialog.croppingEnabled = not dialog.croppingEnabled

        # update the dialog configuration
        dialog.UpdateDialogWithCurrentSetting()

        if not dialog.exec_():
            return

    if reader is not None:
        reader.CropReturns = dialog.croppingEnabled
        reader.CropOutside = dialog.cropOutside
        reader.GetClientSideObject().SetCropMode(dialog.GetCropMode())
        p1 = dialog.firstCorner
        p2 = dialog.secondCorner
        reader.CropRegion = [p1.x(), p2.x(), p1.y(), p2.y(), p1.z(), p2.z()]
        if show:
            smp.Render()

    if sensor is not None:
        sensor.CropReturns = dialog.croppingEnabled
        sensor.CropOutside = dialog.cropOutside
        p1 = dialog.firstCorner
        p2 = dialog.secondCorner
        sensor.CropRegion = [p1.x(), p2.x(), p1.y(), p2.y(), p1.z(), p2.z()]
        if show:
            smp.Render()

def resetCamera():


    def subtract(a, b):
        result = range(3)
        vtk.vtkMath.Subtract(a, b, result)
        return result

    def cross(a, b):
        result = range(3)
        vtk.vtkMath.Cross(a, b, result)
        return result

    view = smp.GetActiveView()

    foc = list(view.CenterOfRotation)
    pos = list(view.CameraPosition)

    viewDirection = subtract(foc, pos)

    view.CameraPosition = subtract([0, 0, 0], viewDirection)
    view.CameraFocalPoint = [0, 0, 0]
    view.CenterOfRotation = [0, 0, 0]

    vtk.vtkMath.Normalize(viewDirection)

    perp = cross(viewDirection, [0, 0, 1])
    viewUp = cross(perp, viewDirection)
    view.CameraViewUp = viewUp

    view.StillRender()


def resetCameraToBirdsEyeView(view=None):

    view = view or smp.GetActiveView()
    view.CameraFocalPoint = [0, 0, 0]
    view.CameraViewUp = [0, 1, 0]
    view.CameraPosition = [0, 0, 40]
    view.CenterOfRotation = [0, 0, 0]
    smp.Render(view)


def resetCameraToForwardView(view=None):

    view = view or smp.GetActiveView()
    view.CameraFocalPoint = [0,0,0]
    view.CameraViewUp = [0, 0.27, 0.96]
    view.CameraPosition = [0, -72, 18.0]
    view.CenterOfRotation = [0, 0, 0]
    smp.Render(view)


def saveScreenshot(filename):
    smp.WriteImage(filename)

    # reload the saved screenshot as a pixmap
    screenshot = QtGui.QPixmap()
    screenshot.load(filename)

    # create a new pixmap with the status bar widget painted at the bottom
    statusBar = QtGui.QPixmap.grabWidget(getMainWindow().statusBar())
    composite = QtGui.QPixmap(screenshot.width(), screenshot.height() + statusBar.height())
    painter = QtGui.QPainter()
    painter.begin(composite)
    painter.drawPixmap(screenshot.rect(), screenshot, screenshot.rect())
    painter.drawPixmap(statusBar.rect().translated(0, screenshot.height()), statusBar, statusBar.rect())
    painter.end()

    # save final screenshot
    composite.save(filename)


def getSpreadSheetViewProxy():
    for p in smp.servermanager.ProxyManager():
        if p.GetXMLName() == 'SpreadSheetView':
            return p


def clearSpreadSheetView():
    view = getSpreadSheetViewProxy()
    view.Representations = []


def showSourceInSpreadSheet(source):

    spreadSheetView = getSpreadSheetViewProxy()
    smp.Show(source, spreadSheetView)

    # Work around a bug where the 'Showing' combobox doesn't update.
    # Calling hide and show again will trigger the refresh.
    smp.Hide(source, spreadSheetView)
    smp.Show(source, spreadSheetView)


def createGrid(view=None):

    view = view or smp.GetActiveView()
    grid = smp.VelodyneHDLGridSource(guiName='Measurement Grid')

    if app.gridProperties.Persist == False:
        grid.GridNbTicks = (int(math.ceil(50000 * app.distanceResolutionM/ grid.GetClientSideObject().GetScale()) ))
    else:
        # Restore grid properties
        grid.Normal = app.gridProperties.Normal
        grid.Origin = app.gridProperties.Origin
        grid.Scale = app.gridProperties.Scale
        grid.GridNbTicks = app.gridProperties.GridNbTicks
        grid.LineWidth = app.gridProperties.LineWidth
        grid.Color = app.gridProperties.Color

    rep = smp.Show(grid, view)
    rep.LineWidth = grid.LineWidth
    rep.DiffuseColor = grid.Color
    rep.Pickable = 0
    rep.Visibility = 0
    smp.SetActiveSource(None)
    return grid


def hideGrid():
    smp.GetDisplayProperties(app.grid).Hide()


def showGrid():
    smp.GetDisplayProperties(app.grid).Show()


def getAnimationScene():
    '''This function is a workaround because paraview.simple.GetAnimationScene()
    has an issue where the returned proxy might not have its Cues property initialized'''
    for proxy in servermanager.ProxyManager().GetProxiesInGroup("animation").values():
        if proxy.GetXMLName() == 'AnimationScene' and len(proxy.Cues):
            return proxy


def start():

    global app
    app = AppLogic()
    app.scene = getAnimationScene()
    app.gridProperties = GridProperties()

    view = smp.GetActiveView()
    view.Background = [0.0, 0.0, 0.0]
    view.Background2 = [0.0, 0.0, 0.2]
    view.UseGradientBackground = True
    smp._DisableFirstRenderCameraReset()
    smp.GetActiveView().LODThreshold = 1e100
    app.distanceResolutionM = 0.002
    app.grid = createGrid()
    app.ruler = createRuler()

    resetCameraToForwardView()

    setupActions()
    disablePlaybackActions()
    disableSaveActions()
    app.actions['actionSelectDualReturn'].setEnabled(False)
    app.actions['actionMeasure'].setEnabled(view.CameraParallelProjection)
    setupStatusBar()
    setupTimeSliderWidget()
    hideColorByComponent()
    restoreNativeFileDialogsAction()
    updateRecentFiles()
    getTimeKeeper().connect('timeChanged()', onTimeChanged)

    initializeRPMText()


def findQObjectByName(widgets, name):
    for w in widgets:
        if w.objectName == name:
            return w


def getMainWindow():
    return findQObjectByName(QtGui.QApplication.topLevelWidgets(), 'vvMainWindow')


def getPVApplicationCore():
    return PythonQt.paraview.pqPVApplicationCore.instance()


def getPVSettings():
    return getPVApplicationCore().settings()


def getTimeKeeper():
    return getPVApplicationCore().getActiveServer().getTimeKeeper()


def getPlaybackToolBar():
    return findQObjectByName(getMainWindow().children(), 'playbackToolbar')


def quit():
    PythonQt.QtGui.QApplication.instance().quit()
exit = quit


def addShortcuts(keySequenceStr, function):
    shortcut = PythonQt.QtGui.QShortcut(PythonQt.QtGui.QKeySequence(keySequenceStr), getMainWindow())
    shortcut.connect("activated()", function)


def onTrailingFramesChanged(numFrames):
    try:
        app.reader.NumberOfTrailingFrames = numFrames
        smp.Render()
        smp.Render(getSpreadSheetViewProxy())
    except AttributeError:
        pass

def onPointsSkipChanged(pr):
    try:
        app.reader.PointsSkip = pr
        smp.Render()
        smp.Render(getSpreadSheetViewProxy())
    except AttributeError:
        pass

def setupTimeSliderWidget():

    frame = QtGui.QWidget()
    layout = QtGui.QHBoxLayout(frame)
    spinBox = QtGui.QSpinBox()
    spinBox.setMinimum(0)
    spinBox.setMaximum(100)
    spinBox.setValue(0)
    slider = QtGui.QSlider(QtCore.Qt.Horizontal)
    slider.setMaximumWidth(160)
    slider.connect('valueChanged(int)', onTimeSliderChanged)
    spinBox.connect('valueChanged(int)', onTimeSliderChanged)
    slider.setEnabled(False)
    spinBox.setEnabled(False)
    layout.addWidget(slider)
    layout.addWidget(spinBox)
    layout.addStretch()

    toolbar = getPlaybackToolBar()
    toolbar.addWidget(frame)
    app.timeSlider = slider
    app.timeSpinBox = spinBox


def updateSliderTimeRange():
    frame = int(app.scene.AnimationTime)
    lastFrame = int(app.scene.EndTime)

    for widget in (app.timeSlider, app.timeSpinBox):
        widget.setMinimum(0)
        widget.setMaximum(lastFrame)
        widget.setSingleStep(1)
        if hasattr(widget, 'setPageStep'):
            widget.setPageStep(10)
        widget.setValue(frame)
        widget.setEnabled(getNumberOfTimesteps())


def scheduleRender():
    if not app.renderIsPending:
        app.renderIsPending = True
        app.renderTimer.start(33)


def forceRender():
    smp.Render()
    app.renderIsPending = False


def onTimeSliderChanged(frame):
    app.scene.AnimationTime = frame
    updatePosition()


def setupStatusBar():

    statusBar = getMainWindow().statusBar()
    statusBar.addPermanentWidget(app.logoLabel)
    statusBar.addWidget(app.filenameLabel)
    statusBar.addWidget(app.statusLabel)
    statusBar.addWidget(app.timeLabel)


def setActionIcon(actionName, iconPath):
    app.actions[actionName].setIcon(QtGui.QIcon(QtGui.QPixmap(iconPath)))


def onTimeChanged():

    frame = int(getTimeKeeper().getTime())
    app.timeLabel.setText('  Frame: %s' % frame)

    for widget in (app.timeSlider, app.timeSpinBox):
        widget.blockSignals(True)
        widget.setValue(frame)
        widget.blockSignals(False)

    #Remove the Rotation per minute from color label comboBox
    ComboBox = getMainWindow().findChild('vvColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[0]
    n = ComboBox.findText('RotationPerMinute')
    ComboBox.removeItem(n)

    if getSensor():
        showRPM()
    

def onGridProperties():
    if gridAdjustmentDialog.showDialog(getMainWindow(), app.grid, app.gridProperties):
        rep = smp.Show(app.grid, None)
        rep.LineWidth = app.grid.LineWidth
        rep.DiffuseColor = app.grid.Color
        app.actions['actionMeasurement_Grid'].setChecked(True)
        smp.Render()


def onLaserSelection(show = True):
    nchannels = 64
    oldmask = [1] * nchannels
    reader = getReader()
    sensor = getSensor()
    verticalCorrection = [0] * nchannels
    rotationalCorrection = [0] * nchannels
    distanceCorrection = [0] * nchannels
    distanceCorrectionX = [0] * nchannels
    distanceCorrectionY = [0] * nchannels
    verticalOffsetCorrection = [0] * nchannels
    horizontalOffsetCorrection = [0] * nchannels
    focalDistance = [0] * nchannels
    focalSlope = [0] * nchannels
    minIntensity = [0] * nchannels
    maxIntensity = [0] * nchannels

    if reader:
        reader.GetClientSideObject().GetLaserSelection(oldmask)
        reader.GetClientSideObject().GetLaserCorrections(verticalCorrection,
            rotationalCorrection,
            distanceCorrection,
            distanceCorrectionX,
            distanceCorrectionY,
            verticalOffsetCorrection,
            horizontalOffsetCorrection,
            focalDistance,
            focalSlope,
            minIntensity,
            maxIntensity)
        nchannels = reader.GetPropertyValue('NumberOfChannels')

    elif sensor:
        sensor.GetClientSideObject().GetLaserSelection(oldmask)
        sensor.GetClientSideObject().GetLaserCorrections(verticalCorrection,
            rotationalCorrection,
            distanceCorrection,
            distanceCorrectionX,
            distanceCorrectionY,
            verticalOffsetCorrection,
            horizontalOffsetCorrection,
            focalDistance,
            focalSlope,
            minIntensity,
            maxIntensity)

        nchannels = sensor.GetPropertyValue('NumberOfChannels')

    # Initializing the laser selection dialog
    if app.laserSelectionDialog == None:
        app.laserSelectionDialog = PythonQt.paraview.vvLaserSelectionDialog(getMainWindow())
        app.laserSelectionDialog.connect('accepted()', onLaserSelectionChanged)
        app.laserSelectionDialog.connect('laserSelectionChanged()', onLaserSelectionChanged)

    # Need a way to initialize the mask
    app.laserSelectionDialog.setLaserSelectionSelector(oldmask)

    app.laserSelectionDialog.setLasersCorrections(verticalCorrection,
        rotationalCorrection,
        distanceCorrection,
        distanceCorrectionX,
        distanceCorrectionY,
        verticalOffsetCorrection,
        horizontalOffsetCorrection,
        focalDistance,
        focalSlope,
        minIntensity,
        maxIntensity,
        nchannels)

    app.laserSelectionDialog.onDisplayMoreCorrectionsChanged()
    if show:
        app.laserSelectionDialog.show()


def onLaserSelectionChanged():
    dialog = getLaserSelectionDialog();
    reader = getReader()
    sensor = getSensor()

    mask = dialog.getLaserSelectionSelector()
    if reader:
        reader.GetClientSideObject().SetLaserSelection(mask)
        reloadCurrentFrame()

    if sensor:
        sensor.GetClientSideObject().SetLaserSelection(mask)
        reloadCurrentFrame()


def hideColorByComponent():
    getMainWindow().findChild('vvColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[1].hide()


def adjustScalarBarRangeLabelFormat():
    if not app.actions['actionScalarBarVisibility'].isChecked():
        return

    arrayName = getMainWindow().findChild('vvColorToolbar').findChild('pqDisplayColorWidget').findChild('QComboBox').currentText
    if arrayName != '' and hasArrayName(app.reader, arrayName):
        sb = smp.GetScalarBar(smp.GetLookupTableForArray(arrayName, []))
        sb.RangeLabelFormat = '%g'
        smp.Render()


def addRecentFile(filename):

    recentFiles = getRecentFiles()

    try:
        recentFiles.remove(filename)
    except ValueError:
        pass

    recentFiles = recentFiles[:4]
    recentFiles.insert(0, filename)
    getPVSettings().setValue('VelodyneHDLPlugin/RecentFiles', recentFiles)
    updateRecentFiles()


def openRecentFile(filename):
    if not os.path.isfile(filename):
        QtGui.QMessageBox.warning(getMainWindow(), 'File not found', 'File not found: %s' % filename)
        return

    if os.path.splitext(filename)[1].lower() == '.pcap':
        openPCAP(filename)
    else:
        openData(filename)


def getRecentFiles():
    return list(getPVSettings().value('VelodyneHDLPlugin/RecentFiles', []) or [])


def updateRecentFiles():
    settings = getPVSettings()
    recentFiles = getRecentFiles()
    recentFilesMenu = findQObjectByName(findQObjectByName(getMainWindow().menuBar().children(), 'menu_File').children(), 'menuRecent_Files')

    clearMenuAction = app.actions['actionClear_Menu']
    for action in recentFilesMenu.actions()[:-2]:
        recentFilesMenu.removeAction(action)

    def createActionFunction(filename):
        def f():
            openRecentFile(filename)
        return f

    actions = []
    for filename in recentFiles:
        actions.append(QtGui.QAction(os.path.basename(filename), recentFilesMenu))
        actions[-1].connect('triggered()', createActionFunction(filename))
    recentFilesMenu.insertActions(recentFilesMenu.actions()[0], actions)


def onClearMenu():
    settings = getPVSettings()
    settings.setValue('VelodyneHDLPlugin/RecentFiles', [])
    updateRecentFiles()

def toggleProjectionType():

    view = smp.GetActiveView()

    view.CameraParallelProjection = not view.CameraParallelProjection
    if app.actions['actionMeasure'].isChecked():
        app.actions['actionMeasure'].trigger()
        app.actions['actionMeasure'].toggle()

    app.actions['actionMeasure'].setEnabled(view.CameraParallelProjection)
    if not view.CameraParallelProjection:
        app.actions['actionMeasure'].setChecked(False)

    smp.Render()


def toggleRPM():

    r = smp.GetRepresentation(app.text)
    r.Visibility = app.actions['actionShowRPM'].isChecked()

    smp.Render()


def toggleSelectDualReturn():
    # test if we are on osx os
    osName = str(sys.platform)
    if osName == 'darwin':
        QtGui.QMessageBox.warning(getMainWindow(), 'Information', 'This functionality is not yet available on %s' % osName)
        return

    #Get the active source
    source = smp.GetActiveSource()

    #If no data are available
    if not source :
        return

    if not source.GetClientSideObject().GetHasDualReturn() :
        QtGui.QMessageBox.warning(getMainWindow(), 'Dual returns not found',
        "The functionality only works with dual returns, and the current"
        "frame has no dual returns.")
        return

    #Get the polyData which contains all points
    allFrame = source.GetClientSideObject().GetOutput()
    nPoints = allFrame.GetNumberOfPoints()

    #Get the selected Points
    selectedPoints = source.GetSelectionOutput(0)
    polyData = selectedPoints.GetClientSideObject().GetOutput()
    idArray = polyData.GetPointData().GetArray('dual_return_matching')
    nSelectedpoints = polyData.GetNumberOfPoints()

    #Select the dual return of each selected points which have a dual return
    if nSelectedpoints >0 :
        #create a temporary array to make a query selection
        array = range(0,nPoints)
        
        #fill the temporary array
        for i in range (nPoints):
            array[i] = -1
        
        #Add the dualId in the temporary array
        for i in range(nSelectedpoints):
            dualId = idArray.GetValue(i)
            if dualId >=0:
                array[dualId] = 1
        
        #Add the temporary array to the source
        source.GetClientSideObject().SetSelectedPointsWithDualReturn(array,nPoints)
        source.GetClientSideObject().SetShouldAddDualReturnArray(True)
        reloadCurrentFrame()
        
        query = 'dualReturn_of_selectedPoints>0'
        smp.SelectPoints(query,source)
        smp.Render()
        #Tell the source the selection is made
        #source.GetClientSideObject().SetShouldAddDualReturnArray(False)
        #Remove the query
        query = ''
    #Select all the points which have a dual return if no points are selected
    else :
        query = 'dual_return_matching>-1'
        smp.SelectPoints(query)
        smp.Render()


def toggleCrashAnalysis():

    app.EnableCrashAnalysis = app.actions['actionEnableCrashAnalysis'].isChecked()


def setViewTo(axis,sign):
    view = smp.GetActiveView()
    viewUp = view.CameraViewUp
    position = view.CameraPosition

    norm = math.sqrt(math.pow(position[0],2) + math.pow(position[1],2) + math.pow(position[2],2))

    if axis == 'X':
        view.CameraViewUp = [0,0,1]
        view.CameraPosition = [-1*sign*norm,0,0]
    elif axis == 'Y':
        view.CameraViewUp = [0,0,1]
        view.CameraPosition = [0,-1*sign*norm,0]
    elif axis == 'Z':
        view.CameraViewUp = [0,1,0]
        view.CameraPosition = [0,0,-1*sign*norm]

    view.CameraFocalPoint = [0,0,0]
    view.CenterOfRotation = [0,0,0]

    view.ResetCamera()
    smp.Render()


def setViewToXPlus():
    setViewTo('X',1)


def setViewToXMinus():
    setViewTo('X',-1)


def setViewToYPlus():
    setViewTo('Y',1)


def setViewToYMinus():
    setViewTo('Y',-1)


def setViewToZPlus():
    setViewTo('Z',1)


def setViewToZMinus():
    setViewTo('Z',-1)

def setFilterToDual():
    setFilterTo(0)

def setFilterToDistanceNear():
    setFilterTo(vtkVelodyneHDLReader.DUAL_DISTANCE_NEAR)

def setFilterToDistanceFar():
    setFilterTo(vtkVelodyneHDLReader.DUAL_DISTANCE_FAR)

def setFilterToIntensityHigh():
    setFilterTo(vtkVelodyneHDLReader.DUAL_INTENSITY_HIGH)

def setFilterToIntensityLow():
    setFilterTo(vtkVelodyneHDLReader.DUAL_INTENSITY_LOW)

def setFilterTo(mask):

    reader = getReader()
    if reader:
        if reader.GetClientSideObject().GetHasDualReturn():
            reader.DualReturnFilter = mask
            smp.Render()
            smp.Render(getSpreadSheetViewProxy())
        else:
            app.actions['actionDualReturnModeDual'].setChecked(True)
            QtGui.QMessageBox.warning(getMainWindow(), 'Dual returns not found',
            "The functionality only works with dual returns, and the current"
            "frame has no dual returns.")

    sensor = getSensor()
    if sensor:
        if sensor.GetClientSideObject().GetHasDualReturn():
            sensor.DualReturnFilter = mask
            smp.Render()
            smp.Render(getSpreadSheetViewProxy())
        else:
            app.actions['actionDualReturnModeDual'].setChecked(True)
            QtGui.QMessageBox.warning(getMainWindow(), 'Dual returns not found',
            "The functionality only works with dual returns, and the current"
            "frame has no dual returns.")


def transformMode():
    reader = getReader()
    if not reader:
        return None

    if reader.ApplyTransform:
        if app.relativeTransform:
            return 2 # relative
        else:
            return 1 # absolute
    return 0 # raw

def setTransformMode(mode):
    # 0 - raw
    # 1 - absolute
    # 2 - relative
    reader = getReader()

    if reader:
        reader.ApplyTransform = (mode > 0)
    app.transformMode = mode
    app.relativeTransform = (mode == 2)

def geolocationChanged(setting):
    setTransformMode(setting)

    updatePosition()
    smp.Render(view=app.mainView)

def intensitiesCorrectedChanged():
    reader = getReader()
    sensor = getSensor()

    if sensor is not None:
        sensor.GetClientSideObject().SetIntensitiesCorrected(app.actions['actionCorrectIntensityValues'].isChecked())
    if reader is not None:
        reader.GetClientSideObject().SetIntensitiesCorrected(app.actions['actionCorrectIntensityValues'].isChecked())

    # Workaround to force the refresh for all the views
    seekForward()
    seekBackward()


def setupActions():

    mW = getMainWindow()
    actions = mW.findChildren('QAction')

    app.actions = {}

    for a in actions:
        app.actions[a.objectName] = a

    app.actions['actionIgnoreZeroDistances'].connect('triggered()', onIgnoreZeroDistances)
    app.actions['actionIntraFiringAdjust'].connect('triggered()', onIntraFiringAdjust)
    app.actions['actionIgnoreEmptyFrames'].connect('triggered()', onIgnoreEmptyFrames)

    app.actions['actionPlaneFit'].connect('triggered()', planeFit)

    app.actions['actionClose'].connect('triggered()', close)
    app.actions['actionPlay'].connect('triggered()', togglePlay)
    app.actions['actionRecord'].connect('triggered()', onRecord)
    app.actions['actionSaveCSV'].connect('triggered()', onSaveCSV)
    app.actions['actionSavePositionCSV'].connect('triggered()', onSavePosition)
    app.actions['actionSaveLAS'].connect('triggered()', onSaveLAS)
    app.actions['actionSavePCAP'].connect('triggered()', onSavePCAP)
    app.actions['actionSaveScreenshot'].connect('triggered()', onSaveScreenshot)
    app.actions['actionExport_To_KiwiViewer'].connect('triggered()', onKiwiViewerExport)
    app.actions['actionReset_Camera'].connect('triggered()', resetCamera)
    app.actions['actionGrid_Properties'].connect('triggered()', onGridProperties)
    app.actions['actionLaserSelection'].connect('triggered()', onLaserSelection)
    app.actions['actionChoose_Calibration_File'].connect('triggered()', onChooseCalibrationFile)
    app.actions['actionCropReturns'].connect('triggered()', onCropReturns)
    app.actions['actionSeek_Forward'].connect('triggered()', seekForward)
    app.actions['actionSeek_Backward'].connect('triggered()', seekBackward)
    app.actions['actionGo_To_End'].connect('triggered()', gotoEnd)
    app.actions['actionGo_To_Start'].connect('triggered()', gotoStart)
    app.actions['actionNative_File_Dialogs'].connect('triggered()', onNativeFileDialogsAction)
    app.actions['actionAbout_VeloView'].connect('triggered()', onAbout)
    app.actions['actionVeloViewDeveloperGuide'].connect('triggered()', onDeveloperGuide)
    app.actions['actionClear_Menu'].connect('triggered()', onClearMenu)

    app.actions['actionToggleProjection'].connect('triggered()', toggleProjectionType)
    app.actions['actionMeasure'].connect('triggered()', toggleRulerContext)

    app.actions['actionSetViewXPlus'].connect('triggered()', setViewToXPlus)
    app.actions['actionSetViewXMinus'].connect('triggered()', setViewToXMinus)
    app.actions['actionSetViewYPlus'].connect('triggered()', setViewToYPlus)
    app.actions['actionSetViewYMinus'].connect('triggered()', setViewToYMinus)
    app.actions['actionSetViewZPlus'].connect('triggered()', setViewToZPlus)
    app.actions['actionSetViewZMinus'].connect('triggered()', setViewToZMinus)

    app.actions['actionDualReturnModeDual'].connect('triggered()', setFilterToDual)
    app.actions['actionDualReturnDistanceNear'].connect('triggered()', setFilterToDistanceNear)
    app.actions['actionDualReturnDistanceFar'].connect('triggered()', setFilterToDistanceFar)
    app.actions['actionDualReturnIntensityHigh'].connect('triggered()', setFilterToIntensityHigh)
    app.actions['actionDualReturnIntensityLow'].connect('triggered()', setFilterToIntensityLow)
    app.actions['actionShowRPM'].connect('triggered()', toggleRPM)
    app.actions['actionEnableCrashAnalysis'].connect('triggered()',toggleCrashAnalysis)
    app.actions['actionCorrectIntensityValues'].connect('triggered()',intensitiesCorrectedChanged)
    app.actions['actionSelectDualReturn'].connect('triggered()',toggleSelectDualReturn)
    app.actions['actionSelectDualReturn2'].connect('triggered()',toggleSelectDualReturn)
    app.EnableCrashAnalysis = app.actions['actionEnableCrashAnalysis'].isChecked()

    # Restore action states from settings
    settings = getPVSettings()
    app.actions['actionIgnoreZeroDistances'].setChecked(int(settings.value('VelodyneHDLPlugin/IgnoreZeroDistances', 1)))
    app.actions['actionIntraFiringAdjust'].setChecked(int(settings.value('VelodyneHDLPlugin/IntraFiringAdjust', 1)))
    app.actions['actionIgnoreEmptyFrames'].setChecked(int(settings.value('VelodyneHDLPlugin/IgnoreEmptyFrames', 1)))

    # Setup and add the geolocation toolbar
    geolocationToolBar = mW.findChild('QToolBar', 'geolocationToolbar')

    # Creating and adding the geolocation label to the geolocation toolbar
    geolocationLabel = QtGui.QLabel('Frame Mapping: ')
    geolocationToolBar.addWidget(geolocationLabel)

    # Creating the geolocation combobox
    geolocationComboBox = QtGui.QComboBox()

    #Adding the different entries
    geolocationComboBox.addItem('None (RAW Data)')
    geolocationComboBox.setItemData(0, "No mapping: Each frame is at the origin", 3)

    geolocationComboBox.addItem('Absolute Geolocation')
    geolocationComboBox.setItemData(1, "Use GPS geolocation to get each frame absolute location, the first frame is shown at origin", 3)

    geolocationComboBox.addItem('Relative Geolocation')
    geolocationComboBox.setItemData(2, "Use GPS geolocation to get each frame absolute location, the current frame is shown at origin", 3)

    geolocationComboBox.connect('currentIndexChanged(int)', geolocationChanged)
    geolocationToolBar.addWidget(geolocationComboBox)

    # Set default toolbar visibility
    geolocationToolBar.visible = False
    app.geolocationToolBar = geolocationToolBar
    
    # Setup and add the playback speed control toolbar
    timeToolBar = mW.findChild('QToolBar','playbackToolbar')

    PlaybackSpeedLabel = QtGui.QLabel('Speed: x')
    PlaybackSpeedLabel.setObjectName('PlaybackSpeedLabel')
    timeToolBar.addWidget(PlaybackSpeedLabel)
    
    PlaybackSpeedComboBox = QtGui.QComboBox()
    PlaybackSpeedComboBox.setObjectName('PlaybackSpeedCombobox')
    PlaybackSpeedComboBox.toolTip = "Playback speed multiplier"
    PlaybackSpeedComboBox.addItem("0.1")
    PlaybackSpeedComboBox.addItem("0.5")
    PlaybackSpeedComboBox.addItem("1")
    PlaybackSpeedComboBox.addItem("1.5")
    PlaybackSpeedComboBox.addItem("2")
    PlaybackSpeedComboBox.addItem("4")
    PlaybackSpeedComboBox.addItem("8")
    PlaybackSpeedComboBox.addItem("16")
    PlaybackSpeedComboBox.addItem("default")
    PlaybackSpeedComboBox.setCurrentIndex(PlaybackSpeedComboBox.count - 1)
    timeToolBar.addWidget(PlaybackSpeedComboBox)
    app.PlaybackSpeed = PlaybackSpeedComboBox
    
    spinBoxLabel = QtGui.QLabel('TF:')
    spinBoxLabel.toolTip = "Number of trailing frames"
    timeToolBar.addWidget(spinBoxLabel)

    spinBox = QtGui.QSpinBox()
    spinBox.toolTip = "Number of trailing frames"
    spinBox.setMinimum(0)
    spinBox.setMaximum(100)
    spinBox.connect('valueChanged(int)', onTrailingFramesChanged)
    app.trailingFramesSpinBox = spinBox

    app.actions['actionTrailingFramesSelector'] = timeToolBar.addWidget(spinBox)
    app.actions['actionTrailingFramesSelector'].setVisible(True)

    pointsSkipLabel = QtGui.QLabel('Skip:')
    pointsSkipLabel.toolTip = "Number of Points to Skip"
    timeToolBar.addWidget(pointsSkipLabel)

    pointsSkipBox = QtGui.QSpinBox()
    pointsSkipBox.toolTip = "Number of Points to Skip"
    pointsSkipBox.setMinimum(0)
    pointsSkipBox.setMaximum(100)
    pointsSkipBox.connect('valueChanged(int)', onPointsSkipChanged)
    app.pointsSkipSpinBox = pointsSkipBox

    app.actions['actionPointsSkipSelector'] = timeToolBar.addWidget(pointsSkipBox)
    app.actions['actionPointsSkipSelector'].setVisible(True)

    buttons = {}
    for button in getPlaybackToolBar().findChildren('QToolButton'):
        buttons[button.text] = button

    buttons['Seek Forward'].connect('pressed()', seekForwardPressed)
    buttons['Seek Forward'].connect('released()', seekForwardReleased)

    buttons['Seek Backward'].connect('pressed()', seekBackwardPressed)
    buttons['Seek Backward'].connect('released()', seekBackwardReleased)

    displayWidget = getMainWindow().findChild('vvColorToolbar').findChild('pqDisplayColorWidget')
    displayWidget.connect('arraySelectionChanged ()',adjustScalarBarRangeLabelFormat)
    app.actions['actionScalarBarVisibility'].connect('triggered()',adjustScalarBarRangeLabelFormat)


def showRPM():

    rpmArray = None

    if getReader():
        rpmArray = getReader().GetClientSideObject().GetOutput().GetFieldData().GetArray('RotationPerMinute')
    elif getSensor():
        rpmArray = getSensor().GetClientSideObject().GetOutput().GetFieldData().GetArray('RotationPerMinute')

    if rpmArray:
        rpm = rpmArray.GetTuple1(0)
        # try to convert the RPM into a str
        # If the RPM is NaN, Infinity, ... catch
        # it and display ??? RPM
        try:
            app.text.Text = str(int(rpm)) + " RPM"
        except :
            app.text.Text = "??? RPM"
    else:
        app.text.Text = "No RPM"

    # Set text style

    textRepresentation = smp.GetRepresentation(app.text)
    textRepresentation.Visibility = app.actions['actionShowRPM'].isChecked()

    smp.Render()


def initializeRPMText():
    app.text = smp.Text()
    app.text.Text = "No RPM"
    textRepresentation = smp.GetRepresentation(app.text)
    textRepresentation.Visibility = app.actions['actionShowRPM'].isChecked()
    textRepresentation.FontSize = 8
    textRepresentation.Color = [1,1,0]


def onIgnoreZeroDistances():
    # Get the check box value as an int to save it into the PV settings (there's incompatibility with python booleans)
    IgnoreZeroDistances = int(app.actions['actionIgnoreZeroDistances'].isChecked())

    # Save the setting for future session
    getPVSettings().setValue('VelodyneHDLPlugin/IgnoreZeroDistances', IgnoreZeroDistances)

    # Apply it to the current source if any
    source = getReader() or getSensor()

    if source:
        source.GetClientSideObject().SetIgnoreZeroDistances(IgnoreZeroDistances)
        reloadCurrentFrame()


def onIntraFiringAdjust():
    # Get the check box value as an int to save it into the PV settings (there's incompatibility with python booleans)
    intraFiringAdjust = int(app.actions['actionIntraFiringAdjust'].isChecked())

    # Save the setting for future session
    getPVSettings().setValue('VelodyneHDLPlugin/IntraFiringAdjust', intraFiringAdjust)

    # Apply it to the current source if any
    source = getReader() or getSensor()

    if source:
        source.GetClientSideObject().SetIntraFiringAdjust(intraFiringAdjust)
        reloadCurrentFrame()


def onIgnoreEmptyFrames():
    # Get the check box value as an int to save it into the PV settings (there's incompatibility with python booleans)
    ignoreEmptyFrames = int(app.actions['actionIgnoreEmptyFrames'].isChecked())

    # Save the setting for future session
    getPVSettings().setValue('VelodyneHDLPlugin/IgnoreEmptyFrames', ignoreEmptyFrames)

    # Apply it to the current source if any
    source = getReader() or getSensor()

    if source:
        source.GetClientSideObject().SetIgnoreEmptyFrames(ignoreEmptyFrames)
        reloadCurrentFrame()


def reloadCurrentFrame():
    source = getReader() or getSensor()
    if source:
        source.DummyProperty = not source.DummyProperty
        smp.Render()
        smp.Render(getSpreadSheetViewProxy())