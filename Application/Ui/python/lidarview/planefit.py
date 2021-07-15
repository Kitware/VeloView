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
import paraview.simple as smp
from paraview import vtk

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
def showStats(actionSpreadsheet=None):
    if actionSpreadsheet is None:
        print("Unable to display stats : SpreadSheet action is not defined")
        return
    
    planeFitter1 = smp.FindSource('PlaneFitter1')
    if planeFitter1 is None:
        print("Unable to create spreadsheet view : PlaneFitter1 source missing")
        return

    renderView1 = smp.FindView('RenderView1')
    if renderView1 is None:
        print("Unable to find main renderView")
        return

    # Check if main spreadsheet view exist or can be created
    spreadSheetView1 = smp.FindView('main spreadsheet view')
    if spreadSheetView1 is None:
        # try to trigger actionSpreadsheet to display main spreadsheet view
        actionSpreadsheet.trigger()
        spreadSheetView1 = smp.FindView('main spreadsheet view')
        if spreadSheetView1 is None:
            print("Unable to get main spreadsheet view")
            return

    # display stats in main spreadsheet view
    spreadSheetView1.ColumnToSort = ''
    spreadSheetView1.BlockSize = 1024
    spreadSheetView1.FieldAssociation = 'Row Data'
    smp.Show(planeFitter1, spreadSheetView1)

def fitPlane(actionSpreadsheet=None):
    src = smp.GetActiveSource()
    if src is None:
        print("A source need to be selected before running plane fitting")
        return

    selection = src.GetSelectionInput(src.Port)

    if selection is None:
        print("A selection has to be defined to run plane fitting")
        return

    extracter = smp.ExtractSelection()
    extracter.Selection = selection
    extracter.Input = src
    smp.Show(extracter)

    # Clean last plane fitting stats before processing a new one
    cleanStats()

    try:
        pd = extracter.GetClientSideObject().GetOutput()
        if not pd.GetNumberOfPoints():
            print("An empty selection is defined")
            return

        # Append data from each block
        if pd.IsTypeOf("vtkMultiBlockDataSet"):
            if not pd.GetNumberOfBlocks():
                print("An empty selection is defined")
                return

            appendFilter = vtk.vtkAppendFilter()
            for i in range(pd.GetNumberOfBlocks()):
                appendFilter.AddInputData(pd.GetBlock(i))
            appendFilter.Update()
            pd = appendFilter.GetOutput()

        # Create a data source from selected points
        PVTrivialProducer1 = smp.PVTrivialProducer()
        PVTrivialProducer1Client = PVTrivialProducer1.GetClientSideObject()
        PVTrivialProducer1Client.SetOutput(pd)

        # Create and apply plane fitter filter
        planeFitter1 = smp.PlaneFitter(Input=PVTrivialProducer1)

        # if laser_id is the name of the array in Legacy and Special Velarray mode
        # LCN is the name of the array in APF mode
        if pd.GetPointData().GetArray("laser_id") :
            planeFitter1.laserIDArray = "laser_id"
        elif pd.GetPointData().GetArray("LCN"):
            planeFitter1.laserIDArray = "LCN"
        planeFitter1.UpdatePipeline()

        # Display results on the main spreadsheet view
        showStats(actionSpreadsheet)

    finally:
        smp.Delete(extracter)
        smp.SetActiveSource(src)
