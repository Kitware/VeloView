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
from __future__ import print_function
import VelodynePluginPython as vpmod
import paraview.simple as smp
from paraview import vtk
import math

def GetSelectionSource(proxy=None):
    """If a selection has exists for the proxy (if proxy is not specified then
       the active source is used), returns that selection source"""
    if not proxy:
        proxy = smp.GetActiveSource()
    if not proxy:
        raise RuntimeError("GetSelectionSource() needs a proxy argument of that an active source is set.")
    return proxy.GetSelectionInput(proxy.Port)

def fitPlane():
    src = smp.GetActiveSource()
    if not src:
        return

    selection = GetSelectionSource(src)

    if not selection:
        return

    extracter = smp.ExtractSelection()
    extracter.Selection = selection
    extracter.Input = src
    smp.Show(extracter)

    try:
        pd = extracter.GetClientSideObject().GetOutput()

        if pd.IsTypeOf("vtkMultiBlockDataSet"):
            appendFilter = vtk.vtkAppendFilter()
            for i in range(pd.GetNumberOfBlocks()):
                appendFilter.AddInputData(pd.GetBlock(i))
            appendFilter.Update()
            pd = appendFilter.GetOutput()

        max_laser_id = pd.GetPointData().GetArray("laser_id").GetRange()[1]
        nchannels = 2**vtk.vtkMath.CeilLog2(int(max_laser_id))

        origin = list(range(3))
        normal = list(range(3))
        mind, maxd, stddev = vtk.mutable(0), vtk.mutable(0), vtk.mutable(0)

        channelMean = list(range(nchannels))
        channelStdDev = list(range(nchannels))
        channelNpts = list(range(nchannels))

        vpmod.vtkPlaneFitter.PlaneFit(pd, origin, normal, mind, maxd, stddev, channelMean, channelStdDev, channelNpts, nchannels)
        rows = [['overall', origin, normal, 0.0, stddev, stddev, pd.GetNumberOfPoints()]]
        rows = rows + [['%d' % i, origin, normal,
                        channelMean[i], channelStdDev[i],
                        math.sqrt(channelMean[i]**2 + channelStdDev[i]**2),
                        channelNpts[i]]
                       for i in range(nchannels)]

        def rowconverter(x):
            try:
                return '\t'.join(['%.4f' % d for d in x])
            except TypeError:
                try:
                    x = x.get()
                except AttributeError:
                    pass
                if type(x) == float:
                    return '%.4f' % x
                elif type(x) is int:
                    return '%d' % x
                else:
                    return x

        print('\t'.join(['channel','originx','originy','originz','normalx','normaly','normalz','mean','stddev','RMS','npts']))
        for r in rows:
            if r[-1] == 0:
                r[-4:-1] = ['nan', 'nan', 'nan']
            print('\t'.join([rowconverter(x) for x in r]))
    finally:
        smp.Delete(extracter)
        smp.SetActiveSource(src)
