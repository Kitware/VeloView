"""This module contains tools that help visualization

You can use its functions by executing from the Python shell:
"from  lidarview import visualizationTools"
"visualizationTools.someFunction()"

Features:
    - visualize the orientation part of a TemporalTransforms using functions
      createTemporalTransformsAxes and deleteAllTemporalTransformsAxes
"""

import paraview.simple as smp

def getCalculatorName(arrayName):
    return "axes_" + arrayName + "_direction"

def setupCalculator(source, arrayName, color, fctString, renderView):
    calculator = smp.Calculator(Input=source)
    smp.RenameSource(getCalculatorName(arrayName), calculator)
    calculator.Function = ''
    calculator.ResultArrayName = arrayName
    calculator.Function = fctString
    calculatorDisplay = smp.GetDisplayProperties(calculator, view=renderView)
    calculatorDisplay.SetRepresentationType('3D Glyphs')
    calculatorDisplay.DiffuseColor = color
    calculatorDisplay.Orient = 1
    calculatorDisplay.SelectOrientationVectors = 'Orientation(AxisAngle)'
    calculatorDisplay.SelectOrientationVectors = arrayName

# This is an application of the Rodrigue rotation formula
# src: https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
# This formula is used to compute the direction of axe x (resp y, z)
# of the referential provided by the TemporalTransforms polydata at current
# time. The rotation that allows computing this direction is stored in an
# axis-angle representation where components _0 to _2 is the rotation axis
# (normalized) and component _3 is the rotation angle in radian.
# iHat, jHat and kHat are ParaView representations of unitary vectors x,y,z.
fx = "(cos(Orientation(AxisAngle)_3) + 0.0                                                      + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_0 * Orientation(AxisAngle)_0) * iHat \
+ (                              + sin(Orientation(AxisAngle)_3) * Orientation(AxisAngle)_2 + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_0 * Orientation(AxisAngle)_1) * jHat \
+ (                              - sin(Orientation(AxisAngle)_3) * Orientation(AxisAngle)_1 + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_0 * Orientation(AxisAngle)_2) * kHat"

fy = "  (                              - sin(Orientation(AxisAngle)_3) * Orientation(AxisAngle)_2 + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_1 * Orientation(AxisAngle)_0) * iHat \
+ (cos(Orientation(AxisAngle)_3) + 0.0                                                      + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_1 * Orientation(AxisAngle)_1) * jHat \
+ (                              + sin(Orientation(AxisAngle)_3) * Orientation(AxisAngle)_0 + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_1 * Orientation(AxisAngle)_2) * kHat"

fz = "  (                              + sin(Orientation(AxisAngle)_3) * Orientation(AxisAngle)_1 + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_2 * Orientation(AxisAngle)_0) * iHat \
+ (                              - sin(Orientation(AxisAngle)_3) * Orientation(AxisAngle)_0 + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_2 * Orientation(AxisAngle)_1) * jHat \
+ (cos(Orientation(AxisAngle)_3) + 0.0                                                      + (1.0 - cos(Orientation(AxisAngle)_3)) * Orientation(AxisAngle)_2 * Orientation(AxisAngle)_2) * kHat"

def createTemporalTransformsAxes(source="default", renderView="default"):
    """ Display orientation part of a TemporalTransforms

    Allow visualizing the orientations part of the TemporalTransforms by
    displaying the x, y and z unitary vector of the sensor reference frame.

    The source must provide transforms, for example it could be:
    a TemporalTransformReader, or the trajectory output of Lidar SLAM

    ParaView pipeline elements are created. These elements can be deleted
    using function deleteAllTemporalTransformsAxes().
    """
    # GetActiveSource() and GetRenderView() do not work if used as default
    # parameter value, so they are called here if needs be:
    if source == "default":
        source = smp.GetActiveSource()
    if renderView == "default":
        renderView = smp.GetRenderView()
    smp.SetActiveSource(source) # maybe useless
    calculatorX = setupCalculator(source, "RX", [1.0, 0.0, 0.0], fx, renderView)
    calculatorY = setupCalculator(source, "RY", [0.0, 1.0, 0.0], fy, renderView)
    calculatorZ = setupCalculator(source, "RZ", [0.0, 0.0, 1.0], fz, renderView)
    smp.Render()

# I wanted to delete the axes for a specific source, but I did not find a way
# to access a source children. (I tried using return value of Python's id()
# as a key but the id was not constant).
# Note that there can be multiples sources with the same name in the pipeline.
def deleteAllTemporalTransformsAxes():
    namesToDelete = [getCalculatorName("RX"), getCalculatorName("RY"), getCalculatorName("RZ")]
    sToDelete = []
    for s in smp.GetSources():
        if s[0] in namesToDelete:
            smp.Delete(smp.GetSources()[s])
    smp.Render()
