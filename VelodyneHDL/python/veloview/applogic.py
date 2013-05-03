import os
import paraview.simple as smp
from paraview import vtk


class AppLogic(object):

    def __init__(self):
        pass


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

    smp.OpenDataFile(filename)
    smp.Show()
    smp.ResetCamera()
    smp.Render()
    smp.SetActiveSource(None)


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


def openSensor(calibrationFile):

    unloadReader()

    sensor = smp.VelodyneHDLSource(guiName='Sensor', CalibrationFile=calibrationFile, CacheSize=100)

    sensor.UpdatePipeline()
    sensor.Start()
    rep = smp.Show(sensor)
    rep.InterpolateScalarsBeforeMapping = 0

    smp.GetActiveView().ViewTime = 0.0

    global app
    app.sensor = sensor
    app.colorByInitialized = False

    smp.Render()


def openPCAP(filename, calibrationFile):

    unloadReader()

    reader = smp.VelodyneHDLSource(guiName='Reader', PacketFile=filename, CorrectionsFile=calibrationFile, CacheSize=0)
    reader.ReadNextFrame()
    reader.UpdatePipeline()

    if not hasArrayName(reader, 'intensity'):
        smp.Delete(reader)
        resetCameraToForwardView()
        return

    reader.Start()
    rep = smp.Show(reader)
    rep.InterpolateScalarsBeforeMapping = 0

    colorByIntensity(reader)

    smp.GetActiveView().ViewTime = 0.0

    global app
    app.reader = reader

    resetCamera()


def hideMeasurementGrid():
    global app
    rep = smp.GetDisplayProperties(app.grid)
    rep.Visibility = 0
    smp.Render()


def showMeasurementGrid():
    global app
    rep = smp.GetDisplayProperties(app.grid)
    rep.Visibility = 1
    smp.Render()


def saveCSVCurrentFrame(filename):
    w = smp.DataSetCSVWriter(FileName=filename)
    w.UpdatePipeline()
    smp.Delete(w)


def saveCSVAllFrames(filename):
    saveCSV(filename, getCurrentTimesteps())


def saveCSVLastFrames(filename, lastFrames):
    if lastFrames >= 1:
        timesteps = getCurrentTimesteps()
        endIndex = timesteps.index(smp.GetActiveView().ViewTime)
        startIndex = endIndex - lastFrames + 1
        startIndex = max(startIndex, 0)
        timesteps = timesteps[startIndex : endIndex+1]
        saveCSV(filename, timesteps)


def saveCSV(filename, timesteps):

    writer = smp.DataSetCSVWriter()
    view = smp.GetActiveView()

    filename, extension = os.path.splitext(filename)
    filename = filename + '_%04d' + extension

    for t in timesteps:
        view.ViewTime = t
        view.StillRender()
        writer.FileName = filename % t
        writer.UpdatePipeline()

    smp.Delete(writer)


def close():

    unloadReader()
    resetCameraToForwardView()


def recordFile(filename):

    sensor = getSensor()
    if sensor:
        sensor.Stop()
        sensor.OutputFile = filename


def stopRecording():

    sensor = getSensor()
    if sensor:
        sensor.Stop()
        sensor.OutputFile = ''


def startStream():

    sensor = getSensor()
    if sensor:
        sensor.Start()


def stopStream():
    sensor = getSensor()
    if sensor:
        sensor.Stop()


def pollSource():

    source = getReader() or getSensor()
    if source is not None:
        source.Poll()
        source.UpdatePipelineInformation()
    return source


def getCurrentTimesteps():
    source = pollSource()
    return list(source.TimestepValues) if source is not None else []


def gotoStart():
    pollSource()
    smp.GetAnimationScene().GoToFirst()


def gotoEnd():
    pollSource()
    smp.GetAnimationScene().GoToLast()


def gotoNext():
    pollSource()
    smp.GetAnimationScene().GoToNext()


def gotoPrevious():
    pollSource()
    smp.GetAnimationScene().GoToPrevious()


playDirection = 1

def playDirectionForward():
    global playDirection
    playDirection = 1


def playDirectionReverse():
    global playDirection
    playDirection = 0


def onPlayTimer():

  global app
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

      smp.GetAnimationScene().GoToLast()

  elif reader is not None:
      timesteps = getCurrentTimesteps()
      if not timesteps:
          return

      global playDirection
      if playDirection == 0 and view.ViewTime != timesteps[0]:
          smp.GetAnimationScene().GoToPrevious()
      elif playDirection == 1 and view.ViewTime != timesteps[-1]:
          smp.GetAnimationScene().GoToNext()


def unloadReader():

    global app
    reader = getReader()
    sensor = getSensor()

    if reader is not None:
        reader.Stop()
        smp.Delete(reader)
        app.reader = None

    if sensor is not None:
        sensor.Stop()
        smp.Delete(sensor)
        app.sensor = None


def getReader():
    global app
    return getattr(app, 'reader', None)


def getSensor():
    global app
    return getattr(app, 'sensor', None)


def setCalibrationFile(calibrationFile):

    reader = getReader()
    sensor = getSensor()

    if reader is not None:
        filename = reader.PacketFile
        openPCAP(filename, calibrationFile)


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


def createGrid(view=None):

    view = view or smp.GetActiveView()
    grid = smp.VelodyneHDLGridSource()
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


def start():

    global app
    app = AppLogic()

    view = smp.GetActiveView()
    view.Background = [0.0, 0.0, 0.0]
    view.Background2 = [0.0, 0.0, 0.2]
    view.UseGradientBackground = True
    smp._DisableFirstRenderCameraReset()
    smp.GetActiveView().LODThreshold = 1e100
    app.grid = createGrid()

    resetCameraToForwardView()
