
# if VeloView runs from a build directory then we need
# to add ParaView python modules to the sys.path.

import sys
import os

def getParaViewBuildDir():
    appDir = os.path.dirname(sys.executable)
    for searchDir in ['../../../../', '../']:
        cmakeCache = os.path.join(appDir, searchDir, 'CMakeCache.txt')
        if os.path.isfile(cmakeCache):
            for line in open(cmakeCache, 'r'):
                if line.startswith('ParaView_DIR'):
                    return line.strip().split('=')[1]


def addParaViewPath():
    paraviewBuildDir = getParaViewBuildDir()
    if paraviewBuildDir:
        sys.path.append(os.path.join(paraviewBuildDir, 'lib'))
        sys.path.append(os.path.join(paraviewBuildDir, 'lib/site-packages'))


addParaViewPath()
