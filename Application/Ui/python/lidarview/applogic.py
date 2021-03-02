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
import bisect
import sys
import paraview.simple as smp
from paraview import servermanager
from paraview import vtk

import PythonQt
from PythonQt import QtCore, QtGui

from vtk import vtkXMLPolyDataWriter
import lidarviewcore.kiwiviewerExporter
import gridAdjustmentDialog
import aboutDialog
import bisect

from PythonQt.paraview import vvCalibrationDialog, vvCropReturnsDialog, vvSelectFramesDialog

# import the vtk wrapping of the Lidar Plugin
# this enable to get the specific vtkObject behind a proxy via GetClientSideObject()
# without this plugin, GetClientSideObject(), would return the first mother class known by paraview
import LidarPluginPython

import planefit


_repCache = {}

SAMPLE_PROCESSING_MODE = False

def onSpreadSheetEnabled(status):
    smp.SetActiveView(app.mainView)

def vtkGetFileNameFromPluginName(pluginName):
  import os
  if os.name == "nt":
    return pluginName + ".dll";
  elif sys.platform == "darwin":
    return "lib" + pluginName + ".dylib";
  else:
    return "lib" + pluginName + ".so";

def cachedGetRepresentation(src, view):
    try:
        return _repCache[(src, view)]
    except KeyError:
        rep = smp.GetRepresentation(src, view)
        _repCache[(src, view)] = rep
        return rep

class AppLogic(object):

    def __init__(self):
        self.createStatusBarWidgets()

        self.mousePressed = False

        mainView = smp.GetActiveView()
        self.mainView = mainView

        self.transformMode = 0
        self.relativeTransform = False

        self.reader = None
        self.trailingFrame = None
        self.position = None
        self.sensor = None

        self.laserSelectionSession = {}

        self.gridProperties = None

        #smp.LoadPlugin(vtkGetFileNameFromPluginName('PointCloudPlugin'))
        smp.LoadPlugin(vtkGetFileNameFromPluginName('EyeDomeLightingView'))


    def createStatusBarWidgets(self):

        self.logoLabel = QtGui.QLabel()
        self.logoLabel.setPixmap(QtGui.QPixmap(":/vvResources/SoftwareInformation/bottom_logo.png"))
        self.logoLabel.setScaledContents(True)

        self.filenameLabel = QtGui.QLabel()
        self.statusLabel = QtGui.QLabel()
        self.sensorInformationLabel = QtGui.QLabel()
        self.positionPacketInfoLabel = QtGui.QLabel()


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

    array = sourceProxy.PointData.GetArray(arrayName)
    if array:
        return True

    return False


def openData(filename):

    close()

    reader = smp.OpenDataFile(filename, guiName='Data')

    if not reader:
        return

    rep = smp.Show(reader)
    rep.InterpolateScalarsBeforeMapping = 0
    colorByArrayName(reader, "intensity")

    showSourceInSpreadSheet(reader)

    app.reader = reader
    app.filenameLabel.setText('File: %s' % os.path.basename(filename))

    enableSaveActions()
    addRecentFile(filename)
    app.actions['actionSavePCAP'].setEnabled(False)
    app.actions['actionChoose_Calibration_File'].setEnabled(False)
    app.actions['actionCropReturns'].setEnabled(False)
    app.actions['actionDualReturnModeDual'].enabled = True
    app.actions['actionDualReturnDistanceNear'].enabled = True
    app.actions['actionDualReturnDistanceFar'].enabled = True
    app.actions['actionDualReturnIntensityHigh'].enabled = True
    app.actions['actionDualReturnIntensityLow'].enabled = True
    app.actions['actionShowRPM'].enabled = True


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


def setDefaultLookupTables(sourceProxy, arrayName):
    createDSRColorsPreset()

    presets = servermanager.vtkSMTransferFunctionPresets()

    dsrIndex = findPresetByName("DSR Colors")
    presetDSR = presets.GetPresetAsString(dsrIndex)

    # LUT for arrayName
    smp.GetLookupTableForArray(
      arrayName, 1,
      ScalarRangeInitialized=1.0,
      ColorSpace='HSV',
      RGBPoints=[0.0, 0.0, 0.0, 1.0,
               100.0, 1.0, 1.0, 0.0,
               256.0, 1.0, 0.0, 0.0])

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


def colorByArrayName(sourceProxy, arrayName):
    setDefaultLookupTables(sourceProxy, arrayName)
    rep = smp.GetDisplayProperties(sourceProxy)
    rep.ColorArrayName = arrayName
    rep.LookupTable = smp.GetLookupTableForArray(arrayName, 1)

    return True


def getTimeStamp():
    format = '%Y-%m-%d-%H-%M-%S'
    return datetime.datetime.now().strftime(format)


def getReaderFileName():
    filename = getReader().FileName
    return filename[0] if isinstance(filename, servermanager.FileNameProperty) else filename


def getDefaultSaveFileName(extension, suffix='', frameId=None, baseName="Frame"):

    sensor = getSensor()
    reader = getReader()

    nchannels = None
    if sensor and sensor.Interpreter:
        sensor.Interpreter.UpdatePipelineInformation()
        return '%s.%s' % (sensor.Interpreter.GetProperty("DefaultRecordFileName")[0], extension)

    if reader:
        basename =  os.path.splitext(os.path.basename(getReaderFileName()))[0]
        if frameId is not None:
            suffix = '%s (%s %04d)' % (suffix, baseName, frameId)
        return '%s%s.%s' % (basename, suffix, extension)


def chooseCalibration(calibrationFilename=None):

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
            self.isCrashAnalysing = dialog.isCrashAnalysing()
            self.isEnableInterpretGPSPackets = dialog.isEnableInterpretGPSPackets()

            self.lidarTranslation = [dialog.lidarX(), dialog.lidarY(), dialog.lidarZ()]
            self.lidarRotation = [dialog.lidarRoll(), dialog.lidarPitch(), dialog.lidarYaw()]

            self.gpsTranslation = [dialog.gpsX(), dialog.gpsY(), dialog.gpsZ()]
            self.gpsRotation = [dialog.gpsRoll(), dialog.gpsPitch(), dialog.gpsYaw()]

    dialog = vvCalibrationDialog(getMainWindow())
    if calibrationFilename is None:
        if not dialog.exec_():
            return None
        return Calibration(dialog)
    else:
        result = Calibration(dialog)
        result.calibrationFile = calibrationFilename
        return result

def openSensor():

    calibration = chooseCalibration()
    if not calibration:
        return

    calibrationFile = calibration.calibrationFile
    LidarPort = calibration.lidarPort
    GPSPort = calibration.gpsPort
    LIDARForwardingPort = calibration.lidarForwardingPort
    GPSForwardingPort = calibration.gpsForwardingPort
    isForwarding = calibration.isForwarding
    ipAddressForwarding = calibration.ipAddressForwarding

    close()
    app.grid = createGrid()

    sensor = smp.LidarStream(guiName='Data', CalibrationFile=calibrationFile)

    if "velarray" in calibrationFile.lower():
        sensor.Interpreter = 'Velodyne Special Velarray Interpreter'
    else :
        sensor.Interpreter = 'Velodyne Meta Interpreter'

    sensor.Interpreter.UseIntraFiringAdjustment = app.actions['actionIntraFiringAdjust'].isChecked()

    sensor.ListeningPort = LidarPort
    sensor.ForwardedPort = LIDARForwardingPort
    sensor.IsForwarding = isForwarding
    sensor.ForwardedIpAddress = ipAddressForwarding
    sensor.IsCrashAnalysing = calibration.isCrashAnalysing
    sensor.Interpreter.SensorTransform.Translate = calibration.lidarTranslation
    sensor.Interpreter.SensorTransform.Rotate = calibration.lidarRotation

    sensor.Interpreter.IgnoreZeroDistances = app.actions['actionIgnoreZeroDistances'].isChecked()
    sensor.Interpreter.HideDropPoints = app.actions['actionHideDropPoints'].isChecked()
    sensor.Interpreter.IgnoreEmptyFrames = app.actions['actionIgnoreEmptyFrames'].isChecked()
    sensor.UpdatePipeline()
    sensor.Start()

    if calibration.isEnableInterpretGPSPackets :
        posOrSensor = smp.PositionOrientationStream(guiName='Position Orientation Data')
        posOrSensor.ListeningPort = GPSPort
        posOrSensor.ForwardedPort = GPSForwardingPort
        posOrSensor.IsForwarding = isForwarding
        posOrSensor.ForwardedIpAddress = ipAddressForwarding
        posOrSensor.IsCrashAnalysing = calibration.isCrashAnalysing
        posOrSensor.Interpreter.SensorTransform.Translate = calibration.gpsTranslation
        posOrSensor.Interpreter.SensorTransform.Rotate = calibration.gpsRotation
        app.position = posOrSensor
        posOrSensor.UpdatePipeline()
        posOrSensor.Start()

    if SAMPLE_PROCESSING_MODE:
        processor = smp.ProcessingSample(sensor)

    app.sensor = sensor
    app.trailingFramesSpinBox.enabled = False
    app.colorByInitialized = False
    app.filenameLabel.setText('Live sensor stream (Port:'+str(LidarPort)+')' )
    app.positionPacketInfoLabel.setText('')
    enableSaveActions()

    onCropReturns(False) # Dont show the dialog just restore settings
    restoreLaserSelection()

    rep = smp.Show(sensor)
#    rep.InterpolateScalarsBeforeMapping = 0
#    if app.sensor.GetClientSideObject().GetNumberOfChannels() == 128:
#        rep.Representation = 'Point Cloud'
#        rep.ColorArrayName = 'intensity'

    if SAMPLE_PROCESSING_MODE:
        prep = smp.Show(processor)
    smp.Render()

    showSourceInSpreadSheet(app.trailingFrame)

    app.actions['actionShowRPM'].enabled = True
    app.actions['actionCorrectIntensityValues'].enabled = True
    app.actions['actionFastRenderer'].enabled = True

    #Auto adjustment of the grid size with the distance resolution
    app.DistanceResolutionM = sensor.Interpreter.GetClientSideObject().GetDistanceResolutionM()
    app.actions['actionMeasurement_Grid'].setChecked(True)
    showMeasurementGrid()

    # Always enable dual return mode selection. A warning will be raised if
    # there's no dual return on the current frame later on
    app.actions['actionDualReturnModeDual'].enabled = True
    app.actions['actionDualReturnDistanceNear'].enabled = True
    app.actions['actionDualReturnDistanceFar'].enabled = True
    app.actions['actionDualReturnIntensityHigh'].enabled = True
    app.actions['actionDualReturnIntensityLow'].enabled = True

    # Hide Position Orientation Stream by default and select lidarStream as active source
    smp.SetActiveSource(sensor)

    updateUIwithNewLidar()
    smp.Render()

    # In OpenSensor we don't have access to the futur available arrays
    nChannels = sensor.Interpreter.GetProperty("NumberOfChannelsInformation")[0]
    print(nChannels)
    arrayName = "intensity"
    if nChannels == 128:
        arrayName = "reflectivity"
    colorByArrayName(sensor, arrayName)


def openPCAP(filename, positionFilename=None, calibrationFilename=None, calibrationUIArgs=None):

    calibration = chooseCalibration(calibrationFilename)
    if not calibration:
        return

    if calibrationFilename is not None and calibrationUIArgs is not None and isinstance(calibrationUIArgs, dict):
        for k in calibrationUIArgs.keys():
          if hasattr(calibration, k):
            setattr(calibration, k, calibrationUIArgs[k])

    calibrationFile = calibration.calibrationFile

    close()

    def onProgressEvent(o, e):
        PythonQt.QtGui.QApplication.instance().processEvents()

    progressDialog = QtGui.QProgressDialog('Reading packet file...', '', 0, 0, getMainWindow())
    progressDialog.setCancelButton(None)
    progressDialog.setModal(True)
    progressDialog.show()

    handler = servermanager.ActiveConnection.Session.GetProgressHandler()
    handler.PrepareProgress()
    interval = handler.GetProgressInterval()
    handler.SetProgressInterval(0.05)
    tag = handler.AddObserver('ProgressEvent', onProgressEvent)

    # construct the reader, this calls UpdateInformation on the
    # reader which scans the pcap file and emits progress events
    reader = smp.LidarReader(guiName='Data',
                             CalibrationFile = calibrationFile,
                             FileName = filename
                             )

    if "velarray" in calibrationFile.lower():
        reader.Interpreter = 'Velodyne Special Velarray Interpreter'
    else :
        reader.Interpreter = 'Velodyne Meta Interpreter'

    reader.Interpreter.UseIntraFiringAdjustment = app.actions['actionIntraFiringAdjust'].isChecked()

    reader.UpdatePipelineInformation()
    app.reader = reader
    app.trailingFramesSpinBox.enabled = True
    app.trailingFrame = smp.TrailingFrame(guiName="TrailingFrame", Input=getLidar(), NumberOfTrailingFrames=app.trailingFramesSpinBox.value)

    displayableFilename = os.path.basename(filename)
    # shorten the name to display because the status bar gives a lower bound to main window width
    shortDisplayableFilename = (displayableFilename[:59] + '...' + displayableFilename[-58:]) if len(displayableFilename) > 120 else displayableFilename
    app.filenameLabel.setText('File: %s' % shortDisplayableFilename)
    app.filenameLabel.setToolTip('File: %s' % displayableFilename)

    app.positionPacketInfoLabel.setText('') # will be updated later if possible
    onCropReturns(False) # Dont show the dialog just restore settings

    # Resetting laser selection dialog according to the opened PCAP file
    # and restoring the dialog visibility afterward

    restoreLaserSelection()

    reader.Interpreter.SensorTransform.Translate = calibration.lidarTranslation
    reader.Interpreter.SensorTransform.Rotate = calibration.lidarRotation

    lidarPacketInterpreter = getLidarPacketInterpreter()
    lidarPacketInterpreter.IgnoreZeroDistances = app.actions['actionIgnoreZeroDistances'].isChecked()
    lidarPacketInterpreter.HideDropPoints = app.actions['actionHideDropPoints'].isChecked()
    lidarPacketInterpreter.IgnoreEmptyFrames = app.actions['actionIgnoreEmptyFrames'].isChecked()

    if SAMPLE_PROCESSING_MODE:
        processor = smp.ProcessingSample(reader)

    handler.RemoveObserver(tag)
    handler.LocalCleanupPendingProgress()
    handler.SetProgressInterval(interval)
    progressDialog.close()

    if SAMPLE_PROCESSING_MODE:
        prep = smp.Show(processor)
    app.scene.UpdateAnimationUsingDataTimeSteps()

    if calibration.isEnableInterpretGPSPackets :
        posreader = smp.PositionOrientationReader(guiName='Position Orientation Data', FileName = filename)

        posreader.Interpreter.SensorTransform.Translate = calibration.gpsTranslation
        posreader.Interpreter.SensorTransform.Rotate = calibration.gpsRotation

        smp.Show(posreader)

        if positionFilename is None:
            # only VelodyneHDLReader provides this information
            # this information must be read after an update
            # GetTimeSyncInfo() has the side effect of showing a message in the error
            # console in the cases where the timeshift if computed
            # wrapping not currently working for plugins:
            # app.positionPacketInfoLabel.setText(posreader.GetClientSideObject().GetTimeSyncInfo())
            pass

        output0 = posreader.GetClientSideObject().GetOutput(0)
        if output0.GetNumberOfPoints() != 0:

            output1 = posreader.GetClientSideObject().GetOutputDataObject(1)
            trange = output1.GetColumnByName("time").GetRange()

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
            app.position = posreader
            if not app.actions['actionShowPosition'].isChecked():
                smp.Hide(app.position)
        else:
            if positionFilename is not None:
                QtGui.QMessageBox.warning(getMainWindow(), 'Georeferencing data invalid',
                                          'File %s is empty or not supported' % positionFilename)
            smp.Delete(posreader)

        app.position = posreader

    smp.SetActiveView(app.mainView)

    arrayName = "reflectivity"
    if not hasArrayName(app.trailingFrame, arrayName):
        arrayName = "intensity"
    colorByArrayName(app.trailingFrame, arrayName)

    showSourceInSpreadSheet(app.trailingFrame)

    enableSaveActions()
    addRecentFile(filename)

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
    app.actions['actionFastRenderer'].enabled = True

    #Auto adjustment of the grid size with the distance resolution
    app.DistanceResolutionM = reader.Interpreter.GetClientSideObject().GetDistanceResolutionM()
    app.grid = createGrid()
    app.actions['actionMeasurement_Grid'].setChecked(True)
    showMeasurementGrid()

    smp.SetActiveSource(app.trailingFrame)
    updateUIwithNewLidar()



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
    app.mainView.Representations.append(distancerepeasy)
    distancerepeasy.Visibility = False
    smp.Render(app.mainView)

    return distancerepeasy


def hideRuler():
    app.ruler.Visibility = False
    smp.Render(app.mainView)


def showRuler():
    app.ruler.Visibility = True
    smp.Render(app.mainView)

def getPointFromCoordinate(coord, midPlaneDistance = 0.5):
    assert len(coord) == 2

    windowHeight = app.mainView.ViewSize[1]

    displayPoint = [coord[0], windowHeight - coord[1], midPlaneDistance]
    renderer = app.mainView.GetRenderer()
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

    pqView = app.mainView
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
    if False:
        # this is what you should do if you wanted to save CSV from TrailingFrame:
        extractBlock = smp.ExtractBlock(Input=smp.GetActiveSource())
        extractBlock.BlockIndices = [0]
        mergeBlocks = smp.MergeBlocks(Input=extractBlock)
        extractSurface = smp.ExtractSurface(Input=mergeBlocks)
        extractSurface.UpdatePipeline()
        sourceToSave = extractSurface
    else:
        sourceToSave = getLidar()

    w = smp.CreateWriter(filename, sourceToSave)
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)
    if False:
        smp.Delete(extractSurface)
        smp.Delete(mergeBlocks)
        smp.Delete(extractBlock)
    # rotateCSVFile(filename)

def saveCSVCurrentFrameSelection(filename):
    source = getReader()
    selection = source.GetSelectionOutput(0)
    extractSelection = smp.ExtractSelection(Input = source, Selection = selection.Selection)
    w = smp.CreateWriter(filename, extractSelection)
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)
    # rotateCSVFile(filename)

# transform parameter indicates the coordinates system and
# the referential for the exported points clouds:
# - 0 Sensor: sensor referential, cartesian coordinate system
# - 1: Relative Geoposition: NED base centered at the first position
#      of the sensor, cartesian coordinate system
# - 2: Absolute Geoposition: NED base centered at the corresponding
#      UTM zone, cartesian coordinate system
# - 3: Absolute Geoposition Lat/Lon: Lat / Lon coordinate system
def saveLASFrames(filename, first, last, transform = 0):
    reader = getReader().GetClientSideObject()

    # Check that we have a position provider
    if getPosition() is not None:
        position = getPosition().GetClientSideObject().GetOutput()

        PythonQt.paraview.pqLidarViewManager.saveFramesToLAS(
            reader, position, first, last, filename, transform)

    else:
        PythonQt.paraview.pqLidarViewManager.saveFramesToLAS(
            reader, None, first, last, filename, transform)


# transform parameter indicates the coordinates system and
# the referential for the exported points clouds:
# - 0 Sensor: sensor referential, cartesian coordinate system
# - 1: Relative Geoposition: NED base centered at the first position
#      of the sensor, cartesian coordinate system
# - 2: Absolute Geoposition: NED base centered at the corresponding
#      UTM zone, cartesian coordinate system
# - 3: Absolute Geoposition Lat/Lon: Lat / Lon coordinate system
def saveLASCurrentFrame(filename, transform = 0):
    t = app.scene.AnimationTime
    saveLASFrames(filename, t, t, transform)


def saveFrameRange(filename, frameStart, frameStop, saveFunction):
    timesteps = range(frameStart, frameStop+1)
    saveFunction(filename, timesteps)


def saveCSV(filename, steps):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    basenameWithoutExtension = os.path.splitext(os.path.basename(filename))[0]
    outDir = os.path.join(tempDir, basenameWithoutExtension)
    filenameTemplate = os.path.join(outDir, basenameWithoutExtension + ' (Frame %04d).csv')
    os.makedirs(outDir)

    writer = smp.CreateWriter('tmp.csv', getLidar())
    writer.FieldAssociation = 'Points'
    writer.Precision = 16

    for i in steps:
        app.scene.AnimationTime = getLidar().TimestepValues[i]
        writer.FileName = filenameTemplate % i
        writer.UpdatePipeline()
        # rotateCSVFile(writer.FileName)

    smp.Delete(writer)

    kiwiviewerExporter.zipDir(outDir, filename)
    kiwiviewerExporter.shutil.rmtree(tempDir)

# transform parameter indicates the coordinates system and
# the referential for the exported points clouds:
# - 0 Sensor: sensor referential, cartesian coordinate system
# - 1: Relative Geoposition: NED base centered at the first position
#      of the sensor, cartesian coordinate system
# - 2: Absolute Geoposition: NED base centered at the corresponding
#      UTM zone, cartesian coordinate system
# - 3: Absolute Geoposition Lat/Lon: Lat / Lon coordinate system
def saveLAS(filename, timesteps, transform = 0):

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
    defaultDir = settings.value('LidarPlugin/OpenData/DefaultDir', QtCore.QDir.homePath())
    defaultFileName = defaultDir if not defaultFileName else os.path.join(defaultDir, defaultFileName)

    nativeDialog = 0 if app.actions['actionNative_File_Dialogs'].isChecked() else QtGui.QFileDialog.DontUseNativeDialog

    filters = '%s (*.%s)' % (extension, extension)
    selectedFilter = '*.%s' % extension
    fileName = QtGui.QFileDialog.getSaveFileName(getMainWindow(), title,
                        defaultFileName, filters, selectedFilter, nativeDialog)

    if fileName:
        settings.setValue('LidarPlugin/OpenData/DefaultDir', QtCore.QFileInfo(fileName).absoluteDir().absolutePath())
        return fileName


def restoreNativeFileDialogsAction():
    settings = getPVSettings()
    app.actions['actionNative_File_Dialogs'].setChecked(int(settings.value('LidarPlugin/NativeFileDialogs', 1)))


def onNativeFileDialogsAction():
    settings = getPVSettings()
    defaultDir = settings.setValue('LidarPlugin/NativeFileDialogs', int(app.actions['actionNative_File_Dialogs'].isChecked()))


def getFrameSelectionFromUser(frameStrideVisibility=False, framePackVisibility=False, frameTransformVisibility=False):
    class FrameOptions(object):
        pass

    dialog = PythonQt.paraview.vvSelectFramesDialog(getMainWindow())
    dialog.frameMinimum = 0
    if app.reader is None:
        dialog.frameMaximum = 0
    elif app.reader.GetClientSideObject().GetShowFirstAndLastFrame():
        dialog.frameMaximum = app.reader.GetClientSideObject().GetNumberOfFrames() - 1
    else:
        dialog.frameMaximum = app.reader.GetClientSideObject().GetNumberOfFrames() - 3
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


def resetCenterToLidarCenter():
    view = smp.GetActiveView()
    view.CenterOfRotation = [0, 0, 0]
    smp.Render()


def resetCameraLidar():
    view = app.mainView
    view.CameraFocalPoint = [0,0,0]
    view.CameraViewUp = [0, 0, 1]

    # Position at 30 degrees [0, -(squareRoot(3)/2)*L, (1/2)*L]
    L = 100
    view.CameraPosition = [0, -0.866025 * L, (1.0/2.0) * L]

    view.CenterOfRotation = [0, 0, 0]
    smp.Render(view)


def onSaveCSV():
    # It is not possible to save as CSV during stream as we need frame numbers
    if getSensor():
        QtGui.QMessageBox.information(getMainWindow(),
                                      'Save As CSV not available during stream',
                                      'Saving as CSV is not possible during lidar stream mode. '
                                      'Please use the "Record" tool, and open the resulting pcap offline to process it.')
        return

    frameOptions = getFrameSelectionFromUser()
    if frameOptions is None:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        frameNumber = getFrameFromAnimationTime(getAnimationScene().AnimationTime)
        fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', frameId=frameNumber))
        if fileName:
            saveCSVCurrentFrame(fileName)
    else:
        if frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
            frameOptions.start = 0
            frameOptions.stop = len(getLidar().TimestepValues) - 1
        defaultFileName = getDefaultSaveFileName('zip')
        fileName = getSaveFileName('Save CSV (to zip file)', 'zip', defaultFileName)
        if fileName:
            saveFrameRange(fileName, frameOptions.start, frameOptions.stop, saveCSV)

def onSavePosition():
    fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', '-position'))
    if fileName:
        savePositionCSV(fileName)


def onSaveLAS():
    # It is not possible to save as LAS during stream as we need frame numbers
    if getSensor():
        QtGui.QMessageBox.information(getMainWindow(),
                                      'Save As LAS not available during stream',
                                      'Saving as LAS is not possible during lidar stream mode. '
                                      'Please use the "Record" tool, and open the resulting pcap offline to process it.')
        return

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
                start = 0
                stop = len(getLidar().TimestepValues) - 1
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
    # It is not possible to save as PCAP during stream as we need frame numbers
    if getSensor():
        QtGui.QMessageBox.information(getMainWindow(),
                                      'Save As PCAP not available during stream',
                                      'Saving as PCAP is not possible during lidar stream mode. '
                                      'Please use the "Record" tool, and open the resulting pcap offline to process it.')
        return

    frameOptions = getFrameSelectionFromUser(frameTransformVisibility=False)
    if frameOptions is None:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        frameOptions.start = getFrameFromAnimationTime(getAnimationScene().AnimationTime)
        frameOptions.stop = frameOptions.start
        defaultFileName = getDefaultSaveFileName('pcap', frameId=frameOptions.start)
    elif frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
        frameOptions.start = 0
        frameOptions.stop = 0 if getReader() is None else getReader().GetClientSideObject().GetNumberOfFrames() - 1
        defaultFileName = getDefaultSaveFileName('pcap')
    else:
        defaultFileName = getDefaultSaveFileName('pcap', suffix=' (Frame %d to %d)' % (frameOptions.start, frameOptions.stop))

    fileName = getSaveFileName('Save PCAP', 'pcap', defaultFileName)
    if not fileName:
        return

    PythonQt.paraview.pqLidarViewManager.saveFramesToPCAP(getReader().SMProxy, frameOptions.start, frameOptions.stop, fileName)


def getFrameFromAnimationTime(time):
    if not getReader():
        return -1

    index = bisect.bisect_right(getAnimationScene().TimeKeeper.TimestepValues, time)
    if index > 0:
        previousTime = getAnimationScene().TimeKeeper.TimestepValues[index - 1]
        nextTime     = getAnimationScene().TimeKeeper.TimestepValues[index]
        index = index - 1 if (abs(previousTime - time) < abs(nextTime - time)) else index
    return index


def onSaveScreenshot():
    nameCurrentFrame= "Frame"
    numCurrentFrame = getFrameFromAnimationTime(app.scene.AnimationTime)
    # If we did not find a frame number, we use the animation time
    if numCurrentFrame == -1:
        numCurrentFrame = app.scene.AnimationTime
        nameCurrentFrame = "Time"

    fileName = getSaveFileName('Save Screenshot', 'png', getDefaultSaveFileName('png', frameId=numCurrentFrame, baseName=nameCurrentFrame))
    if fileName:
        if fileName[-4:] != ".png":
            fileName += ".png"
        saveScreenshot(fileName)


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
    QtGui.QDesktopServices.openUrl(QtCore.QUrl('https://www.paraview.org/veloview/#developers'))


def onUserGuide():
    basePath = PythonQt.QtGui.QApplication.instance().applicationDirPath()
    filename = 'VeloView_User_Guide.pdf'
    dirs = ['../Resources', # apple
            '../../doc', # linux
            '../doc'] # windows

    # on Linux we need to temporarily change LD_LIBRARY_PATH as it points to
    # .so libraries likely to be incompatible with the PDF reader
    if "LD_LIBRARY_PATH" in os.environ:
        saved_ld_library_path = os.environ['LD_LIBRARY_PATH']
        home_dir = os.path.expanduser("~")
        os.environ['LD_LIBRARY_PATH'] = home_dir

    for d in dirs:
        path = os.path.abspath(os.path.join(basePath, os.path.join(d, filename)))
        if os.path.isfile(path):
            QtGui.QDesktopServices.openUrl(QtCore.QUrl('file:///%s' % path, QtCore.QUrl.TolerantMode))

    if "LD_LIBRARY_PATH" in os.environ:
        os.environ['LD_LIBRARY_PATH'] = saved_ld_library_path


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

    smp.GetAnimationScene().Stop()
    hideRuler()
    unloadData()
    app.scene.AnimationTime = 0
    app.reader = None
    app.sensor = None
    app.trailingFrame = None
    smp.Delete(app.grid)

    smp.HideUnusedScalarBars()

    resetCameraToForwardView()
    app.filenameLabel.setText('')
    app.statusLabel.setText('')
    disableSaveActions()
    app.actions['actionDualReturnModeDual'].setChecked(True)

    app.actions['actionSelectDualReturn'].enabled = False
    app.actions['actionSelectDualReturn2'].enabled = False
    app.actions['actionDualReturnModeDual'].enabled = False
    app.actions['actionDualReturnDistanceNear'].enabled = False
    app.actions['actionDualReturnDistanceFar'].enabled = False
    app.actions['actionDualReturnIntensityHigh'].enabled = False
    app.actions['actionDualReturnIntensityLow'].enabled = False
    app.actions['actionCorrectIntensityValues'].enabled = False
    app.actions['actionFastRenderer'].enabled = False


def _setSaveActionsEnabled(enabled):
    for action in ('SaveCSV', 'SavePCAP', 'SaveLAS',
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


def startStream():

    sensor = getSensor()
    if sensor:
        sensor.Start()


def stopStream():
    sensor = getSensor()
    if sensor:
        sensor.Stop()


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


def getNumberOfTimesteps():
    return getTimeKeeper().getNumberOfTimeStepValues()


def unloadData():
    _repCache.clear()

    for k, src in smp.GetSources().items():
        if src != app.grid and src != smp.FindSource("RPM"):
            smp.Delete(src)

    app.reader = None
    app.trailingFrame = None
    app.position = None
    app.sensor = None

    clearSpreadSheetView()

def getReader():
    return getattr(app, 'reader', None)

def getSensor():
    return getattr(app, 'sensor', None)

def getLidar():
    return getReader() or getSensor()

def getLidarPacketInterpreter():
    lidar = getLidar()
    if lidar:
      return lidar.Interpreter
    return None

def getPosition():
    return getattr(app, 'position', None)

def getLaserSelectionDialog():
    return getattr(app, 'laserSelectionDialog', None)

def onChooseCalibrationFile():

    calibration = chooseCalibration()
    if not calibration:
        return

    calibrationFile = calibration.calibrationFile

    lidar = getLidar()
    if lidar:
        lidar.CalibrationFile = calibrationFile
        lidar.Interpreter.SensorTransform.Translate = calibration.lidarTranslation
        lidar.Interpreter.SensorTransform.Rotate = calibration.lidarRotation

    lidarStream = getSensor()
    if lidarStream:
      lidarStream.ListeningPort = calibration.lidarPort
      lidarStream.ForwardedPort = calibration.lidarForwardingPort
      lidarStream.IsForwarding = calibration.isForwarding
      lidarStream.ForwardedIpAddress = calibration.ipAddressForwarding
      lidarStream.IsCrashAnalysing = calibration.isCrashAnalysing

    positionOrientation = getPosition()
    if positionOrientation:
      positionOrientation.Interpreter.SensorTransform.Translate = calibration.gpsTranslation
      positionOrientation.Interpreter.SensorTransform.Rotate = calibration.gpsRotation
      # If positionOrientation has an attribute "ListeningPort"
      # This is a PositionOrientationstream so we have to update network part too
      if(hasattr(positionOrientation, "ListeningPort")) :
        positionOrientation.ListeningPort = calibration.gpsPort
        positionOrientation.ForwardedPort = calibration.gpsForwardingPort
        positionOrientation.IsForwarding = calibration.isForwarding
        positionOrientation.ForwardedIpAddress = calibration.ipAddressForwarding
        positionOrientation.IsCrashAnalysing = calibration.isCrashAnalysing

    updateUIwithNewLidar()

    smp.Render()

def onCropReturns(show = True):
    dialog = vvCropReturnsDialog(getMainWindow())

    cropEnabled = False
    cropOutside = False
    firstCorner = QtGui.QVector3D()
    secondCorner = QtGui.QVector3D()

    lidarInterpreter = getLidarPacketInterpreter()

    # Retrieve current values to fill the UI
    if lidarInterpreter:
        cropEnabled = lidarInterpreter.CropMode != 'None'
        cropOutside = lidarInterpreter.CropOutside
        firstCorner = QtGui.QVector3D(lidarInterpreter.CropRegion[0], lidarInterpreter.CropRegion[2], lidarInterpreter.CropRegion[4])
        secondCorner = QtGui.QVector3D(lidarInterpreter.CropRegion[1], lidarInterpreter.CropRegion[3], lidarInterpreter.CropRegion[5])

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

    if lidarInterpreter:
        lidarInterpreter.CropOutside = dialog.cropOutside
        dialogCropMode = ['None', 'Cartesian', 'Spherical']
        lidarInterpreter.CropMode = dialogCropMode[dialog.GetCropMode()]
        p1 = dialog.firstCorner
        p2 = dialog.secondCorner
        lidarInterpreter.CropRegion = [p1.x(), p2.x(), p1.y(), p2.y(), p1.z(), p2.z()]
        if show:
            smp.Render()


def resetCameraToBirdsEyeView(view=None):

    view = view or smp.app.mainView
    view.CameraFocalPoint = [0, 0, 0]
    view.CameraViewUp = [0, 1, 0]
    view.CameraPosition = [0, 0, 40]
    view.CenterOfRotation = [0, 0, 0]
    smp.Render(view)


def resetCameraToForwardView(view=None):

    view = view or app.mainView
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
    statusBar = QtGui.QWidget.grab(getMainWindow().statusBar())
    composite = QtGui.QPixmap(screenshot.width(), screenshot.height() + statusBar.height())
    painter = QtGui.QPainter()
    painter.begin(composite)
    painter.drawPixmap(screenshot.rect(), screenshot, screenshot.rect())
    painter.drawPixmap(statusBar.rect().translated(0, screenshot.height()), statusBar, statusBar.rect())
    painter.end()

    # save final screenshot
    composite.save(filename)


def getSpreadSheetViewProxy():
    return smp.servermanager.ProxyManager().GetProxy("views", "main spreadsheet view")

def clearSpreadSheetView():
    view = getSpreadSheetViewProxy()
    if view is not None:
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
    grid = smp.GridSource(guiName='Measurement Grid')

    if app.gridProperties.Persist == False:
         grid.GridNbTicks = 10
#        grid.GridNbTicks = (int(math.ceil(50000 * app.DistanceResolutionM/ grid.Scale )))
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
    app.DistanceResolutionM = 0.002
    app.grid = createGrid()
    app.ruler = createRuler()

    resetCameraToForwardView()

    setupActions()
    disableSaveActions()
    app.actions['actionSelectDualReturn'].setEnabled(False)
    app.actions['actionMeasure'].setEnabled(view.CameraParallelProjection)
    setupStatusBar()
    hideColorByComponent()
    restoreNativeFileDialogsAction()
    updateRecentFiles()
    createRPMBehaviour()


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


def quit():
    PythonQt.QtGui.QApplication.instance().quit()
exit = quit


def addShortcuts(keySequenceStr, function):
    shortcut = PythonQt.QtGui.QShortcut(PythonQt.QtGui.QKeySequence(keySequenceStr), getMainWindow())
    shortcut.connect("activated()", function)


def onTrailingFramesChanged(numFrames):
    tr = smp.FindSource("TrailingFrame")
    if tr:
        tr.NumberOfTrailingFrames = numFrames
        smp.Render()


def onFiringsSkipChanged(pr):
    lidarPacketInterpreter = getLidarPacketInterpreter()
    if lidarPacketInterpreter and hasattr(lidarPacketInterpreter, "FiringsSkip"):
        lidarPacketInterpreter.FiringsSkip = pr
        smp.Render()
        smp.Render(getSpreadSheetViewProxy())


def setupStatusBar():
    # by using a QScrollArea inside the statusBar it should be possible
    # to reduce the minimum main window's width

    statusBar = getMainWindow().statusBar()
    statusBar.addPermanentWidget(app.logoLabel)
    statusBar.addWidget(app.filenameLabel)
    statusBar.addWidget(app.statusLabel)
    statusBar.addWidget(app.sensorInformationLabel)
    statusBar.addWidget(app.positionPacketInfoLabel)


def onGridProperties():
    if gridAdjustmentDialog.showDialog(getMainWindow(), app.grid, app.gridProperties):
        rep = smp.Show(app.grid, None)
        rep.LineWidth = app.grid.LineWidth
        rep.DiffuseColor = app.grid.Color
        app.actions['actionMeasurement_Grid'].setChecked(True)
        smp.Render()


def onLaserSelection():
    dialog = PythonQt.paraview.vvLaserSelectionDialog(getMainWindow())
    nchannels = 128
    currentMask = [1] * nchannels
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
    lidarPacketInterpreter = getLidarPacketInterpreter()
    if lidarPacketInterpreter is not None:
        lidarPacketInterpreter.UpdatePipelineInformation()
    if lidarPacketInterpreter is not None \
        and lidarPacketInterpreter.GetProperty("LaserSelectionInformation") is not None:
        currentMask = list(lidarPacketInterpreter.GetProperty("LaserSelectionInformation"))
        verticalCorrection = list(lidarPacketInterpreter.GetProperty("verticalCorrectionInformation"))
        rotationalCorrection = list(lidarPacketInterpreter.GetProperty("rotationalCorrectionInformation"))
        distanceCorrection = list(lidarPacketInterpreter.GetProperty("distanceCorrectionInformation"))
        distanceCorrectionX = list(lidarPacketInterpreter.GetProperty("distanceCorrectionXInformation"))
        distanceCorrectionY = list(lidarPacketInterpreter.GetProperty("distanceCorrectionYInformation"))
        verticalOffsetCorrection = list(lidarPacketInterpreter.GetProperty("verticalOffsetCorrectionInformation"))
        horizontalOffsetCorrection = list(lidarPacketInterpreter.GetProperty("horizontalOffsetCorrectionInformation"))
        focalDistance = list(lidarPacketInterpreter.GetProperty("focalDistanceInformation"))
        focalSlope = list(lidarPacketInterpreter.GetProperty("focalSlopeInformation"))
        minIntensity = list(lidarPacketInterpreter.GetProperty("minIntensityInformation"))
        maxIntensity = list(lidarPacketInterpreter.GetProperty("maxIntensityInformation"))
        nchannels = lidarPacketInterpreter.GetProperty("NumberOfChannelsInformation")[0]
    else:
        print("Could not locate interpreter with LaserSelectionInformation to use.")
        return

    dialog.setLaserSelectionSelector(currentMask)

    dialog.setLasersCorrections(verticalCorrection,
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

    def onDialogAccepted():
        mask = dialog.getLaserSelectionSelector()
        if dialog.applyOrder > 0:
            lidarPacketInterpreter = getLidarPacketInterpreter()
            if lidarPacketInterpreter is not None:
                lidarPacketInterpreter.LaserSelection = mask
                smp.Render()
        if dialog.applyOrder > 1:
            app.laserSelectionSession[nchannels] = mask
        if dialog.applyOrder > 2:
            getPVSettings().setValue('LidarPlugin/LaserSelection/{}'.format(nchannels), mask)

    dialog.connect('accepted()', onDialogAccepted)
    dialog.show()

def restoreLaserSelection():
    lidarPacketInterpreter = getLidarPacketInterpreter()
    if lidarPacketInterpreter is None:
        return
    lidarPacketInterpreter.UpdatePipelineInformation()
    if lidarPacketInterpreter.GetProperty("LaserSelectionInformation") is not None:
        nchannels = lidarPacketInterpreter.GetProperty("NumberOfChannelsInformation")[0]
    else:
        return
    if nchannels not in app.laserSelectionSession:
        settingsMask = getPVSettings().value('LidarPlugin/LaserSelection/{}'.format(nchannels))
        if settingsMask is not None:
            app.laserSelectionSession[nchannels] = [int(b) for b in settingsMask]
    if nchannels in app.laserSelectionSession:
        lidarPacketInterpreter.LaserSelection = app.laserSelectionSession[nchannels]
        smp.Render()


def hideColorByComponent():
    getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[1].hide()


def adjustScalarBarRangeLabelFormat():
    if not app.actions['actionScalarBarVisibility'].isChecked():
        return

    arrayName = getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget').findChild('QComboBox').currentText
    if arrayName != '' and hasArrayName(app.reader, arrayName):
        sb = smp.GetScalarBar(smp.GetLookupTableForArray(arrayName, []))
        sb.RangeLabelFormat = '%g'
        smp.Render()



def addRecentFile(filename):
    maxRecentFiles = 4
    recentFiles = getRecentFiles()

    # workaround to get system-locale to unicode conversion of filename
    recentFiles.insert(0, filename)
    getPVSettings().setValue('LidarPlugin/RecentFiles', recentFiles)
    unicodeFilename = getPVSettings().value('LidarPlugin/RecentFiles')[0]
    recentFiles = recentFiles[1:]

    try:
        recentFiles.remove(unicodeFilename)
    except ValueError:
        pass

    recentFiles = recentFiles[:maxRecentFiles]
    recentFiles.insert(0, unicodeFilename)

    getPVSettings().setValue('LidarPlugin/RecentFiles', recentFiles)

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
    return list(getPVSettings().value('LidarPlugin/RecentFiles', []) or [])


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
    settings.setValue('LidarPlugin/RecentFiles', [])
    updateRecentFiles()

def toggleProjectionType():

    view = app.mainView

    view.CameraParallelProjection = not view.CameraParallelProjection
    if app.actions['actionMeasure'].isChecked():
        app.actions['actionMeasure'].trigger()
        app.actions['actionMeasure'].toggle()

    app.actions['actionMeasure'].setEnabled(view.CameraParallelProjection)
    if not view.CameraParallelProjection:
        app.actions['actionMeasure'].setChecked(False)

    smp.Render(app.mainView)

def toggleRPM():
    rpm = smp.FindSource("RPM")
    if rpm:
        if app.actions['actionShowRPM'].isChecked():
            smp.Show(rpm)
        else:
            smp.Hide(rpm)
        smp.Render()

def warnNoDualReturns():
  QtGui.QMessageBox.warning(getMainWindow(), 'Dual returns not found',
  "The functionality only works with dual returns, and the current"
  " frame has no dual returns, or the interpreter used does not"
  " support dual returns.")

def toggleSelectDualReturn():
    import warnings;warnings.simplefilter(action = "ignore", category = FutureWarning) # try removing this when vtk is upgraded

    # test if we are on osx os
    osName = str(sys.platform)
    if osName == 'darwin':
        QtGui.QMessageBox.warning(getMainWindow(), 'Information', 'This functionality is not yet available on %s' % osName)
        return

    #Get the active source
    source = smp.FindSource("TrailingFrame")
    lidarPacketInterpreter = getLidarPacketInterpreter()

    #If no data are available
    if not source :
        return

    if not hasDualReturn(lidarPacketInterpreter):
        warnNoDualReturns()
        return

    #Get the selected Points
    selectedPoints = source.GetSelectionOutput(0)
    polyData = selectedPoints.GetClientSideObject().GetOutput()
    nSelectedPoints = polyData.GetNumberOfPoints()

    if nSelectedPoints > 0:
        # implementation for a non-multiblock object:
        # idArray = polyData.GetPointData().GetArray('dual_return_matching')
        # query = 'np.logical_and(dual_return_matching > -1, np.in1d(id, [{}]))'.format(','.join(selectedDualIds))

        if polyData.GetNumberOfBlocks() != 1:
            QtGui.QMessageBox.warning(getMainWindow(),
                                      'Cannot select dual return matching',
                                      'Number of blocks is not 1. Please set 0 trailing frame and retry.')
            return

        idArray = polyData.GetBlock(0).GetPointData().GetArray('dual_return_matching')
        selectedDualIds = set(str(int(idArray.GetValue(i))) for i in range(nSelectedPoints))
        query = 'dsa.VTKArray([dual_return_matching.Arrays[0][i] > -1 and i in [{}] for i in range(id.size)])' \
                .format(','.join(selectedDualIds))
    else:
        query = 'dual_return_matching > -1'
    smp.SelectPoints(query)
    smp.Render()


def toggleCrashAnalysis():
    app.EnableCrashAnalysis = app.actions['actionEnableCrashAnalysis'].isChecked()

def setFilterToDual():
    setFilterTo("Dual")

def setFilterToDistanceNear():
    setFilterTo("Near Distance")

def setFilterToDistanceFar():
    setFilterTo("Far Distance")

def setFilterToIntensityHigh():
    setFilterTo("High Intensity")

def setFilterToIntensityLow():
    setFilterTo("Low Intensity")

def hasDualReturn(interpreter):
    interpreter.UpdatePipelineInformation()
    prop = interpreter.GetProperty("HasDualReturnInformation")
    return prop != None and len(prop) == 1 and prop[0] == 1

def setFilterTo(mask):

    interp = getLidarPacketInterpreter()
    if interp:
        if  hasDualReturn(interp):
            interp.DualReturnFilter = mask
            smp.Render()
            smp.Render(getSpreadSheetViewProxy())
        else:
            app.actions['actionDualReturnModeDual'].setChecked(True)
            warnNoDualReturns()

def transformMode():
    reader = getReader()
    if not reader:
        return None

    if hasattr(reader, 'ApplyTransform') and reader.ApplyTransform:
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
    smp.Render(view=app.mainView)

def fastRendererChanged():
    """ Enable/Disable fast rendering by using the point cloud representation (currently only for VLS-128)
    this representation hardcode the color map and their LookUpTable, which improve execution speed significantly """

    source = smp.FindSource("TrailingFrame")
    if source:
        rep = smp.GetRepresentation(source)

        if app.actions['actionFastRenderer'].isChecked():
            rep.Representation = 'Point Cloud'
        else:
            rep.Representation = 'Surface'

def intensitiesCorrectedChanged():
    lidarInterpreter = getLidarPacketInterpreter()
    if lidarInterpreter:
        lidarInterpreter.CorrectIntensity = app.actions['actionCorrectIntensityValues'].isChecked()

def onToogleAdvancedGUI(updateSettings = True):
  """ Switch the GUI between advanced and classic mode"""
  # hide/show Sources menu
  menuSources = getMainWindow().findChild("QMenu", "menuSources").menuAction()
  menuSources.visible = False # not menuSources.visible
  # hide/show Filters menu
  menuFilters = getMainWindow().findChild("QMenu", "menuFilters").menuAction()
  menuFilters.visible = False # not menuFilters.visible
  # hide/show Advance menu
  menuAdvance = getMainWindow().findChild("QMenu", "menuAdvance").menuAction()
  menuAdvance.visible = False # not menuAdvance.visible
  # hide/show view decorator
  getMainWindow().centralWidget().toggleWidgetDecoration()
  # hide/show some views
  advance_action = ["Display", "Information", "Memory Inspector", "Pipeline Browser", "Properties", "View"]
  for action in getMainWindow().findChild("QMenu", "menuViews").actions():
    if action.text in advance_action:
      action.visible = not action.visible
  # update the UserSettings
  if updateSettings:
    # booleans must be store as int
    newValue = int(not int(getPVSettings().value("LidarPlugin/AdvanceFeature/Enable", 0)))
    getPVSettings().setValue("LidarPlugin/AdvanceFeature/Enable", newValue)

def switchVisibility(Proxy):
    """ Invert the Proxy visibility int the current view """
    ProxyRep = smp.GetRepresentation(Proxy)
    ProxyRep.Visibility = not ProxyRep.Visibility

def ShowPosition():
    if app.position:
        switchVisibility(app.position)
        smp.Render()


def setupActions():

    mW = getMainWindow()
    actions = mW.findChildren('QAction')

    app.actions = {}

    for a in actions:
        app.actions[a.objectName] = a

    app.actions['actionAdvanceFeature'].connect('triggered()', onToogleAdvancedGUI)

    app.actions['actionIgnoreZeroDistances'].connect('triggered()', onIgnoreZeroDistances)
    app.actions['actionHideDropPoints'].connect('triggered()', onHideDropPoints)
    app.actions['actionIntraFiringAdjust'].connect('triggered()', onIntraFiringAdjust)
    app.actions['actionIgnoreEmptyFrames'].connect('triggered()', onIgnoreEmptyFrames)

    app.actions['actionPlaneFit'].connect('triggered()', planeFit)

    app.actions['actionClose'].connect('triggered()', close)
    app.actions['actionSaveCSV'].connect('triggered()', onSaveCSV)
    app.actions['actionSavePositionCSV'].connect('triggered()', onSavePosition)
    app.actions['actionSaveLAS'].connect('triggered()', onSaveLAS)
    app.actions['actionSavePCAP'].connect('triggered()', onSavePCAP)
    app.actions['actionSaveScreenshot'].connect('triggered()', onSaveScreenshot)
    app.actions['actionGrid_Properties'].connect('triggered()', onGridProperties)
    app.actions['actionLaserSelection'].connect('triggered()', onLaserSelection)
    app.actions['actionChoose_Calibration_File'].connect('triggered()', onChooseCalibrationFile)
    app.actions['actionCropReturns'].connect('triggered()', onCropReturns)
    app.actions['actionNative_File_Dialogs'].connect('triggered()', onNativeFileDialogsAction)
    app.actions['actionAbout_LidarView'].connect('triggered()', onAbout)
    app.actions['actionLidarViewUserGuide'].connect('triggered()', onUserGuide)
    app.actions['actionLidarViewDeveloperGuide'].connect('triggered()', onDeveloperGuide)
    app.actions['actionClear_Menu'].connect('triggered()', onClearMenu)

    app.actions['actionToggleProjection'].connect('triggered()', toggleProjectionType)
    app.actions['actionMeasure'].connect('triggered()', toggleRulerContext)
    app.actions['actionShowPosition'].connect('triggered()', ShowPosition)

    app.actions['actionDualReturnModeDual'].connect('triggered()', setFilterToDual)
    app.actions['actionDualReturnDistanceNear'].connect('triggered()', setFilterToDistanceNear)
    app.actions['actionDualReturnDistanceFar'].connect('triggered()', setFilterToDistanceFar)
    app.actions['actionDualReturnIntensityHigh'].connect('triggered()', setFilterToIntensityHigh)
    app.actions['actionDualReturnIntensityLow'].connect('triggered()', setFilterToIntensityLow)
    app.actions['actionShowRPM'].connect('triggered()', toggleRPM)
    app.actions['actionCorrectIntensityValues'].connect('triggered()',intensitiesCorrectedChanged)
    app.actions['actionFastRenderer'].connect('triggered()',fastRendererChanged)
    app.actions['actionSelectDualReturn'].connect('triggered()',toggleSelectDualReturn)
    app.actions['actionSelectDualReturn2'].connect('triggered()',toggleSelectDualReturn)

    # Restore action states from settings
    settings = getPVSettings()
    app.actions['actionIgnoreZeroDistances'].setChecked(int(settings.value('LidarPlugin/IgnoreZeroDistances', 1)))
    app.actions['actionHideDropPoints'].setChecked(int(settings.value('LidarPlugin/HideDropPoints', 1)))
    app.actions['actionIntraFiringAdjust'].setChecked(int(settings.value('LidarPlugin/IntraFiringAdjust', 1)))
    app.actions['actionIgnoreEmptyFrames'].setChecked(int(settings.value('LidarPlugin/IgnoreEmptyFrames', 1)))

    advanceMode = int(settings.value("LidarPlugin/AdvanceFeature/Enable", 0))
    if not advanceMode:
      app.actions['actionAdvanceFeature'].checked = False
      onToogleAdvancedGUI(updateSettings=False)

    # Setup and add the geolocation toolbar
    geolocationToolBar = mW.findChild('QToolBar', 'geolocationToolbar')

    # Creating and adding the geolocation label to the geolocation toolbar
    geolocationLabel = QtGui.QLabel('Frame Mapping: ')
    geolocationToolBar.addWidget(geolocationLabel)

    # Creating the geolocation combobox
    geolocationComboBox = QtGui.QComboBox()

    # Add the different entries
    # Currently, as Absolute and Relative Geolocation options are broken, disable them.
    geolocationComboBox.addItem('None (RAW Data)')
    geolocationComboBox.setItemData(0, "No mapping: Each frame is at the origin", 3)

    geolocationComboBox.addItem('Absolute Geolocation')
    geolocationComboBox.setItemData(1, "Use GPS geolocation to get each frame absolute location, the first frame is shown at origin", 3)
    geolocationComboBox.model().item(1).setEnabled(False)

    geolocationComboBox.addItem('Relative Geolocation')
    geolocationComboBox.setItemData(2, "Use GPS geolocation to get each frame absolute location, the current frame is shown at origin", 3)
    geolocationComboBox.model().item(2).setEnabled(False)

    geolocationComboBox.connect('currentIndexChanged(int)', geolocationChanged)
    geolocationToolBar.addWidget(geolocationComboBox)

    # Set default toolbar visibility
    geolocationToolBar.visible = False
    app.geolocationToolBar = geolocationToolBar

    # Setup and add the playback speed control toolbar
    timeToolBar = mW.findChild('QToolBar','Player Control')

    # Place the record button at the right place
    timeToolBar.addAction(app.actions['actionRecord'])

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

    FiringsSkipLabel = QtGui.QLabel('Skip:')
    FiringsSkipLabel.toolTip = "Number of Points to Skip"
    timeToolBar.addWidget(FiringsSkipLabel)

    FiringsSkipBox = QtGui.QSpinBox()
    FiringsSkipBox.toolTip = "Number of Points to Skip"
    FiringsSkipBox.setMinimum(0)
    FiringsSkipBox.setMaximum(100)
    FiringsSkipBox.connect('valueChanged(int)', onFiringsSkipChanged)
    app.FiringsSkipSpinBox = FiringsSkipBox

    app.actions['actionFiringsSkipSelector'] = timeToolBar.addWidget(FiringsSkipBox)
    app.actions['actionFiringsSkipSelector'].setVisible(True)

    displayWidget = getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget')
    displayWidget.connect('arraySelectionChanged ()',adjustScalarBarRangeLabelFormat)
    app.actions['actionScalarBarVisibility'].connect('triggered()',adjustScalarBarRangeLabelFormat)

    app.MainToolbar = getMainWindow().findChild('QToolBar','toolBar')
    app.ColorToolbar = getMainWindow().findChild('QToolBar','colorToolBar')
    app.PlaybackToolbar = timeToolBar
    app.GeolocationToolbar = getMainWindow().findChild('QToolBar','geolocationToolbar')

    getMainWindow().findChild("lqSpreadSheetManager").connect('spreadSheetEnabled(bool)', onSpreadSheetEnabled)

    spreadSheetDock = getMainWindow().findChild('QDockWidget', 'spreadSheetDock')
    if spreadSheetDock.visible:
        # this is a hack to ensure that the spreadsheet dock which is restored
        # on startup (if it was enabled) is actually populated with a SpreadSheet
        button = spreadSheetDock.findChild("QDockWidgetTitleButton", "qt_dockwidget_closebutton")
        button.click()

    hidden_action = ["SpreadSheet"] # this is a (bugged) duplicate coming from pqViewMenuManager.cxx
    for action in getMainWindow().findChild("QMenu", "menuViews").actions():
        if action.text in hidden_action:
            action.visible = False

def createRPMBehaviour():
    # create and customize a label to display the rpm
    rpm = smp.Text(guiName="RPM", Text="No RPM")
    representation = smp.GetRepresentation(rpm)
    representation.FontSize = 8
    representation.Color = [1,1,0]
    # create an python animation cue to update the rpm value in the label
    PythonAnimationCue1 = smp.PythonAnimationCue()
    PythonAnimationCue1.Script= """
import paraview.simple as smp
def start_cue(self):
    pass

def tick(self):
    rpm = smp.FindSource("RPM")
    lidar = smp.FindSource("Data")
    if (lidar):
        value = int(lidar.Interpreter.GetClientSideObject().GetFrequency())
        rpm.Text = str(value) + " RPM"
    else:
        rpm.Text = "No RPM"

def end_cue(self):
    pass
"""
    smp.GetAnimationScene().Cues.append(PythonAnimationCue1)
    # force to be consistant with the UI
    toggleRPM()


def onIgnoreZeroDistances():
    # Get the check box value as an int to save it into the PV settings (there's incompatibility with python booleans)
    IgnoreZeroDistances = int(app.actions['actionIgnoreZeroDistances'].isChecked())

    # Save the setting for future session
    getPVSettings().setValue('LidarPlugin/IgnoreZeroDistances', IgnoreZeroDistances)

    # Apply it to the current source if any
    lidarInterpreter = getLidarPacketInterpreter()
    if lidarInterpreter:
        lidarInterpreter.IgnoreZeroDistances = IgnoreZeroDistances
        smp.Render()

def onHideDropPoints():
    # Get the check box value as an int to save it into the PV settings (there's incompatibility with python booleans)
    HideDropPoints = int(app.actions['actionHideDropPoints'].isChecked())

    # Save the setting for future session
    getPVSettings().setValue('LidarPlugin/HideDropPoints', HideDropPoints)

    # Apply it to the current source if any
    lidarInterpreter = getLidarPacketInterpreter()
    if lidarInterpreter:
        lidarInterpreter.HideDropPoints = HideDropPoints
        smp.Render()

def onIntraFiringAdjust():
    # Get the check box value as an int to save it into the PV settings (there's incompatibility with python booleans)
    intraFiringAdjust = int(app.actions['actionIntraFiringAdjust'].isChecked())

    # Save the setting for future session
    getPVSettings().setValue('LidarPlugin/IntraFiringAdjust', intraFiringAdjust)

    # Apply it to the current source if any
    lidarInterpreter = getLidarPacketInterpreter()
    if lidarInterpreter:
        lidarInterpreter.UseIntraFiringAdjustment = intraFiringAdjust
        smp.Render()


def onIgnoreEmptyFrames():
    # Get the check box value as an int to save it into the PV settings (there's incompatibility with python booleans)
    ignoreEmptyFrames = int(app.actions['actionIgnoreEmptyFrames'].isChecked())

    # Save the setting for future session
    getPVSettings().setValue('LidarPlugin/IgnoreEmptyFrames', ignoreEmptyFrames)

    # Apply it to the current source if any
    lidarInterpreter = getLidarPacketInterpreter()
    if lidarInterpreter:
        lidarInterpreter.IgnoreEmptyFrames = ignoreEmptyFrames
        smp.Render()

def updateUIwithNewLidar():
    lidar = getLidar()
    if lidar:
        app.sensorInformationLabel.setText(lidar.GetClientSideObject().GetSensorInformation(True))
        app.sensorInformationLabel.setToolTip(lidar.GetClientSideObject().GetSensorInformation())
    #Remove some array to display
    ComboBox = getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[0]
    listOfArrayToRemove = ['RotationPerMinute', 'vtkBlockColors', 'vtkCompositeIndex']
    for arrayName in listOfArrayToRemove:
        n = ComboBox.findText(arrayName)
        ComboBox.removeItem(n)
