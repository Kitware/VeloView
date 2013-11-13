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
import paraview.simple as smp
from paraview import servermanager
from paraview import vtk

import PythonQt
from PythonQt import QtCore, QtGui

from vtkIOXMLPython import vtkXMLPolyDataWriter
import kiwiviewerExporter
import gridAdjustmentDialog
import planefit

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
        self.logoLabel.setPixmap(QtGui.QPixmap(":/VelodyneHDLPlugin/velodyne_logo.png"))
        self.logoLabel.setScaledContents(True)

        self.filenameLabel = QtGui.QLabel()
        self.statusLabel = QtGui.QLabel()
        self.timeLabel = QtGui.QLabel()


class IconPaths(object):

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


def hasArrayName(sourceProxy, arrayName):
    '''
    Returns True if the data has non-zero points and has a point data
    attribute with the given arrayName.
    '''
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
    colorByIntensity(reader)

    showSourceInSpreadSheet(reader)

    smp.GetActiveView().ViewTime = 0.0

    app.reader = reader
    app.filenameLabel.setText('File: %s' % os.path.basename(filename))

    updateSliderTimeRange()
    enablePlaybackActions()
    enableSaveActions()
    addRecentFile(filename)
    app.actions['actionSave_PCAP'].setEnabled(False)
    app.actions['actionChoose_Calibration_File'].setEnabled(False)
    app.actions['actionRecord'].setEnabled(False)
    app.actions['actionGPSApply'].setEnabled(True)

    resetCamera()


def planeFit():
    planefit.fitPlane()

def colorByIntensity(sourceProxy):

    if not hasArrayName(sourceProxy, 'intensity'):
        return False

    rep = smp.GetDisplayProperties(sourceProxy)
    rep.ColorArrayName = 'intensity'
    rgbPoints = [0.0, 0.0, 0.0, 1.0,
                 100.0, 1.0, 1.0, 0.0,
                 256.0, 1.0, 0.0, 0.0]
    rep.LookupTable = smp.GetLookupTableForArray('intensity', 1, RGBPoints=rgbPoints, ColorSpace="HSV", ScalarRangeInitialized=1.0)
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
        return '%s_Velodyne-HDL-Data.%s' % (getTimeStamp(), extension)
    if reader:
        basename =  os.path.splitext(os.path.basename(getReaderFileName()))[0]
        if appendFrameNumber:
            suffix = '%s (Frame %04d)' % (suffix, int(app.scene.AnimationTime))
        return '%s%s.%s' % (basename, suffix, extension)


def openSensor(calibrationFile):

    close()

    sensor = smp.VelodyneHDLSource(guiName='Data', CalibrationFile=calibrationFile, CacheSize=100)
    sensor.UpdatePipeline()
    sensor.Start()
    rep = smp.Show(sensor)
    rep.InterpolateScalarsBeforeMapping = 0

    smp.GetActiveView().ViewTime = 0.0

    app.sensor = sensor
    app.colorByInitialized = False
    app.filenameLabel.setText('Live sensor stream.')
    enablePlaybackActions()
    enableSaveActions()
    smp.Render()

    showSourceInSpreadSheet(sensor)
    app.actions['actionGPSApply'].setEnabled(False)

    play()


def openPCAP(filename, calibrationFile):

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
                                   NumberOfTrailingFrames=app.trailingFramesSpinBox.value)
    reader.UpdatePipeline()

    handler.RemoveObserver(tag)
    handler.SetProgressFrequency(freq)
    progressDialog.close()

    # If we read the wrong kind of data abort
    if not hasArrayName(reader, 'intensity'):
        smp.Delete(reader)
        resetCameraToForwardView()
        return

    smp.GetActiveView().ViewTime = 0.0

    app.reader = reader
    app.filenameLabel.setText('File: %s' % os.path.basename(filename))

    # update overhead view
    smp.SetActiveView(app.overheadView)
    posreader = smp.VelodyneHDLPositionReader(guiName="Position",
                                              FileName=filename)
    smp.Show(posreader)

    if posreader.GetClientSideObject().GetOutput().GetNumberOfPoints():

        smp.Render()
        app.overheadView.ResetCamera()
        smp.Render()

        trange = posreader.GetPointDataInformation().GetArray('time').GetRange()

        c = smp.Contour(posreader, guiName='CurrentPosition')
        c.ContourBy = 'time'
        c.Isosurfaces = trange[0]

        smp.Show()
        smp.Render()

        smp.Hide(c)
        g = smp.Glyph(c, GlyphType='Sphere', guiName='PositionGlyph')
        g.ScaleMode = 'off'
        g.GlyphType.Radius = 5.0
        smp.Show()
        smp.Render()

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

        app.position = (posreader, c, g)
    else:
        smp.Delete(posreader)

    smp.SetActiveView(app.mainView)
    smp.SetActiveSource(reader)

    if getPosition():
        objtoshow = applyGPSTransform()
    else:
        objtoshow = reader

    rep = smp.Show(objtoshow)
    rep.InterpolateScalarsBeforeMapping = 0
    colorByIntensity(objtoshow)

    showSourceInSpreadSheet(reader)

    updateSliderTimeRange()
    updatePosition()
    enablePlaybackActions()
    enableSaveActions()
    addRecentFile(filename)
    app.actions['actionRecord'].setEnabled(False)

    resetCamera()


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


def applyGPSTransform():
    result = None
    if not app.actions['actionGPSApply'].isChecked():
        # Clean up any possible downstream
        activesrc = smp.GetActiveSource()
        if getReader() != activesrc:
            smp.Delete(activesrc)
        smp.SetActiveSource(getReader())

        result =  getReader()
    else:
        assert smp.GetActiveSource() == getReader()
        if getPosition():
            intp = app.position[0].GetClientSideObject().GetInterpolator()
            offsetfilter = smp.VelodyneOffsetFilter()
            offsetfilter.GetClientSideObject().SetInterp(intp)
            smp.Hide(getReader())
            result = offsetfilter
        else:
            result = getReader()

    if result:
        smp.Show(result)
        colorByIntensity(result)

    smp.Render()
    return result


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
    w = smp.DataSetCSVWriter(getPosition(), FileName=filename)
    w.UpdatePipeline()
    smp.Delete(w)

def saveCSVCurrentFrame(filename):
    w = smp.DataSetCSVWriter(FileName=filename)
    w.UpdatePipeline()
    smp.Delete(w)
    rotateCSVFile(filename)


def saveCSVAllFrames(filename):
    saveCSV(filename, getCurrentTimesteps())


def saveCSVFrameRange(filename, frameStart, frameStop):
    timesteps = range(frameStart, frameStop+1)
    saveCSV(filename, timesteps)


def saveCSV(filename, timesteps):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    basenameWithoutExtension = os.path.splitext(os.path.basename(filename))[0]
    outDir = os.path.join(tempDir, basenameWithoutExtension)
    filenameTemplate = os.path.join(outDir, basenameWithoutExtension + ' (Frame %04d).csv')
    os.makedirs(outDir)

    writer = smp.DataSetCSVWriter()

    for t in timesteps:
        app.scene.AnimationTime = t
        writer.FileName = filenameTemplate % t
        writer.UpdatePipeline()
        rotateCSVFile(writer.FileName)

    smp.Delete(writer)

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


def getFrameSelectionFromUser(frameStrideVisibility=False):

    dialog = PythonQt.paraview.vvSelectFramesDialog(getMainWindow())
    dialog.setFrameMinimum(app.scene.StartTime)
    dialog.setFrameMaximum(app.scene.EndTime)
    dialog.setFrameStrideVisibility(frameStrideVisibility)
    dialog.restoreState()
    accepted = dialog.exec_()
    frameMode = dialog.frameMode()
    frameStart = dialog.frameStart()
    frameStop = dialog.frameStop()
    frameStride = dialog.frameStride()
    dialog.setParent(None)

    return accepted, frameMode, frameStart, frameStop, frameStride


def onSaveCSV():

    accepted, frameMode, frameStart, frameStop, frameStride = getFrameSelectionFromUser()
    if not accepted:
        return


    if frameMode == 0:
        fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', appendFrameNumber=True))
        if fileName:
            saveCSVCurrentFrame(fileName)

    else:
        fileName = getSaveFileName('Save CSV (to zip file)', 'zip', getDefaultSaveFileName('zip'))
        if fileName:
            if frameMode == 1:
                saveCSVAllFrames(fileName)
            else:
                saveCSVFrameRange(fileName, frameStart, frameStop)


def onSavePosition():
    fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', '-position'))
    savePositionCSV(fileName)


def onSavePCAP():

    accepted, frameMode, frameStart, frameStop, frameStride = getFrameSelectionFromUser()
    if not accepted:
        return

    if frameMode == 0:
        frameStart = frameStop = app.scene.AnimationTime
    elif frameMode == 1:
        frameStart = int(app.scene.StartTime)
        frameStop = int(app.scene.EndTime)

    defaultFileName = getDefaultSaveFileName('pcap', suffix=' (Frame %d to %d)' % (frameStart, frameStop))
    fileName = getSaveFileName('Save PCAP', 'pcap', defaultFileName)
    if not fileName:
        return

    PythonQt.paraview.pqVelodyneManager.saveFramesToPCAP(getReader().SMProxy, frameStart, frameStop, fileName)


def onSaveScreenshot():

    fileName = getSaveFileName('Save Screenshot', 'png', getDefaultSaveFileName('png', appendFrameNumber=True))
    if fileName:
        saveScreenshot(fileName)


def onKiwiViewerExport():

    accepted, frameMode, frameStart, frameStop, frameStride = getFrameSelectionFromUser(frameStrideVisibility=True)
    if not accepted:
        return

    defaultFileName = getDefaultSaveFileName('zip', suffix=' (KiwiViewer)')
    fileName = getSaveFileName('Export To KiwiViewer', 'zip', defaultFileName)
    if not fileName:
        return

    if frameMode == 0:
        timesteps = [app.scene.AnimationTime]
    elif frameMode == 1:
        timesteps = range(int(app.scene.StartTime), int(app.scene.EndTime) + 1, frameStride)
    else:
        timesteps = range(frameStart, frameStop+1, frameStride)

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
  return QtGui.QApplication.instance().applicationVersion


def onDevelopperGuide():
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
    title = 'About VeloView'
    text = '''<h1>VeloView %s</h1><br/>Copyright (c) 2013, Velodyne Lidar<br />
           Sample Data Repository: <a href=http://midas3.kitware.com/midas/community/29>http://midas3.kitware.com/midas/community/29</a>'''% getVersionString()
    QtGui.QMessageBox.about(getMainWindow(), title, text)


def close():

    stop()
    unloadData()
    resetCameraToForwardView()
    app.filenameLabel.setText('')
    app.statusLabel.setText('')
    app.timeLabel.setText('')
    updateSliderTimeRange()
    disablePlaybackActions()
    disableSaveActions()
    app.actions['actionRecord'].setChecked(False)


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
    for action in ('Save_CSV', 'Save_PCAP', 'Export_To_KiwiViewer', 'Close', 'Choose_Calibration_File'):
        app.actions['action'+action].setEnabled(enabled)


def enableSaveActions():
    _setSaveActionsEnabled(True)
    if getPosition():
        app.actions['actionSave_PositionCSV'].setEnabled(True)


def disableSaveActions():
    _setSaveActionsEnabled(False)
    app.actions['actionSave_PositionCSV'].setEnabled(False)


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

        fpsDelayMilliseconds = int(1000 / app.targetFps)
        elapsedMilliseconds = int((vtk.vtkTimerLog.GetUniversalTime() - startTime)*1000)
        waitMilliseconds = fpsDelayMilliseconds - elapsedMilliseconds
        app.playTimer.start(waitMilliseconds if waitMilliseconds > 0 else 1)


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
            # TODO: Approximate time, just grabbing the first
            t = pointcloud.GetPointData().GetScalars('timestamp')
            currentTime = t.GetTuple1(0)

            trange = pos.GetPointDataInformation().GetArray('time').GetRange()

            # Clamp
            currentTime = min(max(currentTime, trange[0]+1.0e-1), trange[1]-1.0e-1)

            c = getContour()
            assert c
            c.Isosurfaces = [currentTime]
            smp.Render(view=app.overheadView)


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

    reader = getReader()
    sensor = getSensor()
    position = getPosition()

    if reader is not None:
        activesrc = smp.GetActiveSource()
        if reader != activesrc:
            smp.Delete(activesrc)

        smp.Delete(reader)
        app.reader = None

    if sensor is not None:
        sensor.Stop()
        smp.Delete(sensor)
        app.sensor = None

    if position is not None:
        # Cleanup the scalar bar reps
        toremove = [x for x in app.overheadView.Representations if type(x) == servermanager.rendering.ScalarBarWidgetRepresentation]
        for t in toremove:
            app.overheadView.Representations.remove(t)

        g = getGlyph()
        c = getContour()
        smp.Delete(g)
        smp.Delete(c)
        smp.Delete(position)
        smp.Render(app.overheadView)

        app.position = (None, None, None)

    clearSpreadSheetView()


def getReader():
    return getattr(app, 'reader', None)


def getSensor():
    return getattr(app, 'sensor', None)

def getPosition():
    return getattr(app, 'position', (None, None, None))[0]

def getContour():
    return getattr(app, 'position', (None, None, None))[1]

def getGlyph():
    return getattr(app, 'position', (None, None, None))[2]

def setCalibrationFile(calibrationFile):

    reader = getReader()
    sensor = getSensor()

    if reader is not None:
        reader.CalibrationFile = calibrationFile
        smp.Render()

    elif sensor is not None:
        sensor.CalibrationFile = calibrationFile
        # no need to render now, calibration file will be used on the next frame


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
    rep = smp.Show(grid, view)
    rep.DiffuseColor = [0.2, 0.2, 0.2]
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

    view = smp.GetActiveView()
    view.Background = [0.0, 0.0, 0.0]
    view.Background2 = [0.0, 0.0, 0.2]
    view.UseGradientBackground = True
    smp._DisableFirstRenderCameraReset()
    smp.GetActiveView().LODThreshold = 1e100
    app.grid = createGrid()
    app.ruler = createRuler()

    resetCameraToForwardView()

    setupActions()
    setupEventsListener()
    disablePlaybackActions()
    disableSaveActions()
    app.actions['actionMeasure'].setEnabled(view.CameraParallelProjection)
    setupStatusBar()
    setupTimeSliderWidget()
    hideColorByComponent()
    restoreNativeFileDialogsAction()
    updateRecentFiles()
    getTimeKeeper().connect('timeChanged()', onTimeChanged)


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
        #app.reader.UpdatePipeline()
        smp.Render()
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

    timeKeeper = getTimeKeeper()

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


def onGridProperties():
    if gridAdjustmentDialog.showDialog(getMainWindow(), app.grid):
        smp.Render()


def hideColorByComponent():
    getMainWindow().findChild('pqColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[1].hide()


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
        openPCAP(filename, calibrationFile='')
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

    smp.Render()

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


def setupEventsListener():
    pass

def setupActions():

    mW = getMainWindow()
    actions = mW.findChildren('QAction')

    app.actions = {}

    for a in actions:
        app.actions[a.objectName] = a

    app.actions['actionPlaneFit'].connect('triggered()', planeFit)

    app.actions['actionClose'].connect('triggered()', close)
    app.actions['actionPlay'].connect('triggered()', togglePlay)
    app.actions['actionRecord'].connect('triggered()', onRecord)
    app.actions['actionSave_CSV'].connect('triggered()', onSaveCSV)
    app.actions['actionSave_PositionCSV'].connect('triggered()', onSavePosition)
    app.actions['actionSave_PCAP'].connect('triggered()', onSavePCAP)
    app.actions['actionSave_Screenshot'].connect('triggered()', onSaveScreenshot)
    app.actions['actionExport_To_KiwiViewer'].connect('triggered()', onKiwiViewerExport)
    app.actions['actionReset_Camera'].connect('triggered()', resetCamera)
    app.actions['actionGrid_Properties'].connect('triggered()', onGridProperties)
    app.actions['actionSeek_Forward'].connect('triggered()', seekForward)
    app.actions['actionSeek_Backward'].connect('triggered()', seekBackward)
    app.actions['actionGo_To_End'].connect('triggered()', gotoEnd)
    app.actions['actionGo_To_Start'].connect('triggered()', gotoStart)
    app.actions['actionNative_File_Dialogs'].connect('triggered()', onNativeFileDialogsAction)
    app.actions['actionAbout_VeloView'].connect('triggered()', onAbout)
    app.actions['actionVeloViewDeveloperGuide'].connect('triggered()', onDevelopperGuide)
    app.actions['actionClear_Menu'].connect('triggered()', onClearMenu)

    app.actions['actionToggleProjection'].connect('triggered()', toggleProjectionType)
    app.actions['actionMeasure'].connect('triggered()', toggleRulerContext)

    app.actions['actionSetViewXPlus'].connect('triggered()', setViewToXPlus)
    app.actions['actionSetViewXMinus'].connect('triggered()', setViewToXMinus)
    app.actions['actionSetViewYPlus'].connect('triggered()', setViewToYPlus)
    app.actions['actionSetViewYMinus'].connect('triggered()', setViewToYMinus)
    app.actions['actionSetViewZPlus'].connect('triggered()', setViewToZPlus)
    app.actions['actionSetViewZMinus'].connect('triggered()', setViewToZMinus)

    app.actions['actionGPSApply'].connect('triggered()', applyGPSTransform)

    # Action created #
    timeToolBar = mW.findChild('QToolBar','playbackToolbar')
    trailingFramesToolBar = mW.findChild('QToolBar','trailingFramesToolbar')

    spinBoxLabel = QtGui.QLabel("Trailing frames: ")
    trailingFramesToolBar.addWidget(spinBoxLabel)

    spinBox = QtGui.QSpinBox()
    spinBox.toolTip = "Number of trailing frames"
    spinBox.setMinimum(0)
    spinBox.setMaximum(100)
    spinBox.connect('valueChanged(int)', onTrailingFramesChanged)
    app.trailingFramesSpinBox = spinBox

    app.actions['actionTrailingFramesSelector'] = trailingFramesToolBar.addWidget(spinBox)
    app.actions['actionTrailingFramesSelector'].setVisible(True)

    buttons = {}
    for button in getPlaybackToolBar().findChildren('QToolButton'):
        buttons[button.text] = button

    buttons['Seek Forward'].connect('pressed()', seekForwardPressed)
    buttons['Seek Forward'].connect('released()', seekForwardReleased)

    buttons['Seek Backward'].connect('pressed()', seekBackwardPressed)
    buttons['Seek Backward'].connect('released()', seekBackwardReleased)

