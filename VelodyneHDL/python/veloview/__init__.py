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
