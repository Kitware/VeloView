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

# Clean last planefitting source and the spreadsheet view
def cleanStats():
    planeFitter1 = smp.FindSource('PlaneFitter1')
    if planeFitter1 is not None:
        smp.Delete(planeFitter1)
        del planeFitter1

    PVTrivialProducer1 = smp.FindSource('PVTrivialProducer1')
    if PVTrivialProducer1 is not None:
        smp.Delete(PVTrivialProducer1)
        del PVTrivialProducer1

# Find or create a 'SpreadSheet View' to display plane fitting statistics
def showStats():
    planeFitter1 = smp.FindSource('PlaneFitter1')
    if planeFitter1 is None:
        print("Unable to create spreadsheet view : PlaneFitter1 source missing")
        return

    renderView1 = smp.FindViewOrCreate('RenderView1', viewtype='RenderView')
    if not renderView1:
        print("Unable to find main renderView")
        return

    spreadSheetView1 = smp.FindViewOrCreate('SpreadSheetView1', viewtype='SpreadSheetView')
    spreadSheetView1.ColumnToSort = ''
    spreadSheetView1.BlockSize = 1024

    # show plane fit data in view
    smp.Show(planeFitter1, spreadSheetView1)
    spreadSheetView1.Update()
    spreadSheetView1.FieldAssociation = 'Row Data'

def fitPlane():
    src = smp.GetActiveSource()
    if not src:
        print("A source need to be selected before running plane fitting")
        return

    selection = GetSelectionSource(src)

    if not selection:
        print("Several points has to be selected before running plane fitting")
        return

    extracter = smp.ExtractSelection()
    extracter.Selection = selection
    extracter.Input = src
    smp.Show(extracter)

    # Clean last plane fitting stats before processing a new one
    cleanStats()

    pd = extracter.GetClientSideObject().GetOutput()

    if pd.IsTypeOf("vtkMultiBlockDataSet"):
        appendFilter = vtk.vtkAppendFilter()
        for i in range(pd.GetNumberOfBlocks()):
            appendFilter.AddInputData(pd.GetBlock(i))
        appendFilter.Update()
        pd = appendFilter.GetOutput()

    PVTrivialProducer1 = smp.PVTrivialProducer()
    PVTrivialProducer1Client = PVTrivialProducer1.GetClientSideObject()
    PVTrivialProducer1Client.SetOutput(pd)

    # Create and apply plane fitter filter
    planeFitter1 = smp.PlaneFitter(Input=PVTrivialProducer1)
    planeFitter1.UpdatePipeline()

    # Display results on a spreadsheet view
    showStats()

    smp.Delete(extracter)
    smp.SetActiveSource(src)
