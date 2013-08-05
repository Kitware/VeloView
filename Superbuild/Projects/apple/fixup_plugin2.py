#!/usr/bin/env python
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

import subprocess
import os

plugin = 'libVelodyneHDLPlugin.dylib'
paraviewBuildDir = '/source/paraview/build'

nameprefix = '@executable_path/../Libraries/'
prefix = '@executable_path/../Libraries/'

# The official ParaView OSX binaries are built with hdf5, not vtkhdf5.
# Also, they are built with Python 2.6, not 2.7
namechanges = {
  'libvtkhdf5_hl-pv3.98.1.dylib' : 'libhdf5.1.8.9.dylib',
  'libvtkhdf5-pv3.98.1.dylib' : 'libhdf5_hl.1.8.9.dylib',
  'libvtkWrappingPython27-pv3.98.1.dylib' : 'libvtkWrappingPython26-pv3.98.1.dylib'
}

changePythonFramework = False


def fixupPlugin():

    output = subprocess.check_output(['otool', '-L', plugin])
    lines = output.split('\n')

    libs = []
    qtlibs = []
    for l in lines:

      l = l.strip().split(' ')[0]
      if l.startswith(paraviewBuildDir):
        libs.append(l)
      if l.startswith('Qt'):
        qtlibs.append(l)



    for qtlib in qtlibs:

      command = 'install_name_tool -change %s @executable_path/../Frameworks/%s %s' % (qtlib, qtlib, plugin)
      subprocess.call(command.split())


    if changePythonFramework:
        command = 'install_name_tool -change /System/Library/Frameworks/Python.framework/Versions/2.7/Python /System/Library/Frameworks/Python.framework/Versions/2.6/Python %s' % (plugin)
        subprocess.call(command.split())


    for lib in libs:

      name = os.path.basename(lib)

      if name in namechanges:
        name = namechanges[name]

      command = 'install_name_tool -change %s %s%s %s' % (lib, prefix, name, plugin)
      subprocess.call(command.split())

      pvlib = '/Applications/paraview.app/Contents/Libraries/' + name
      if not os.path.exists(pvlib):
        print 'notfound:', pvlib


    command = 'install_name_tool -id %s%s %s' % (nameprefix, os.path.basename(plugin), plugin)
    subprocess.call(command.split())


if __name__ == '__main__':
    fixupPlugin()
