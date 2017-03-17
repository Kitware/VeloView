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
from PythonQt import QtCore, QtGui, QtUiTools
import math

def showDialog(mainWindow, grid, gridProperties):

    loader = QtUiTools.QUiLoader()
    uifile = QtCore.QFile(':/VelodyneHDLPlugin/vvGridAdjustmentDialog.ui')
    if not uifile.open(uifile.ReadOnly):
        print 'error opening file'
        return

    dialog = loader.load(uifile, mainWindow)
    uifile.close()

    def w(name):
        for widget in dialog.children():
            if widget.objectName == name:
                return widget

    w('SensorUpX').setValue(grid.Normal[0])
    w('SensorUpY').setValue(grid.Normal[1])
    w('SensorUpZ').setValue(grid.Normal[2])

    w('SensorOriginX').setValue(-grid.Origin[0])
    w('SensorOriginY').setValue(-grid.Origin[1])
    w('SensorOriginZ').setValue(-grid.Origin[2])

    w('GridResolution').setValue(grid.Scale)
    w('GridWidth').setValue(grid.Scale*grid.GridNbTicks)
    w('GridLineWidth').setValue(grid.LineWidth)

    r = grid.Color[0] * 255
    g = grid.Color[1] * 255
    b = grid.Color[2] * 255
    w('GridColorPicker').setStyleSheet("background-color: rgb(" + str(r) + "," + str(g) + "," + str(b) +");")

    w('ShouldPropertiesPersist').checked = gridProperties.Persist

    def pickColor():
        colorPicker = QtGui.QColorDialog()
        qColor = colorPicker.getColor()

        if not qColor.isValid():
            return False

        r = qColor.red()
        g = qColor.green()
        b = qColor.blue()
        w('GridColorPicker').setStyleSheet("background-color: rgb(" + str(r) + "," + str(g) + "," + str(b) +");")

    w('GridColorPicker').connect('clicked()', pickColor)

    accepted = dialog.exec_()
    if not accepted:
        return False

    grid.Normal = [w('SensorUpX').value, w('SensorUpY').value, w('SensorUpZ').value]
    grid.Origin = [-w('SensorOriginX').value, -w('SensorOriginY').value, -w('SensorOriginZ').value]
    grid.Scale = w('GridResolution').value
    grid.GridNbTicks = int(math.ceil(w('GridWidth').value / w('GridResolution').value))
    grid.LineWidth = w('GridLineWidth').value
    color = w('GridColorPicker').palette.color(QtGui.QPalette.Background)
    grid.Color = [color.redF(), color.greenF(), color.blueF()]
    gridProperties.Persist = w('ShouldPropertiesPersist').checked

    return True
