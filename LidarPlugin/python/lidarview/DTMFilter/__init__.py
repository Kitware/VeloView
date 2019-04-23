import paraview.simple as smp
import inspect
import os

# unusual dependencies:
# scipy

# To instantiate this filter: do "import lidarview.DTMFilter" from LidarView's
# python console.
# notes: if executed from PV python with execfile(), __file__ is not defined

thisModuleDir = os.path.dirname(os.path.realpath(__file__))
Script_file = thisModuleDir + "/DTMPythonFilter_Script.py"

def cleanup():
    global DTMPythonFilter
    smp.Delete(DTMPythonFilter)

def setup():
    global DTMPythonFilter
    DTMPythonFilter = smp.ProgrammableFilter()
    with open(Script_file, 'r') as f:
        Script_src = f.read()
    DTMPythonFilter.Script = Script_src
    DTMPythonFilter.PythonPath = "\"{}\"".format(thisModuleDir)

setup()
