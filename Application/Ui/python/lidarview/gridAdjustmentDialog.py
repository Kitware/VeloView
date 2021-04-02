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
from PythonQt import QtCore, QtGui, QtUiTools
import math

def showDialog(mainWindow, app):

    loader = QtUiTools.QUiLoader()
    uifile = QtCore.QFile(':/LidarViewPlugin/vvGridAdjustmentDialog.ui')
    if not uifile.open(uifile.ReadOnly):
        print("error opening file")
        return

    dialog = loader.load(uifile, mainWindow)
    uifile.close()

    # Delete "?" Button that appears on windows os
    # Rewrite the flags without QtCore.Qt.WindowContextHelpButtonHint
    flags = QtCore.Qt.Dialog | QtCore.Qt.WindowStaysOnTopHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint
    dialog.setWindowFlags(flags)

    def w(name):
        for widget in dialog.children():
            if widget.objectName == name:
                return widget

    w('SensorUpX').setValue(app.grid.Normal[0])
    w('SensorUpY').setValue(app.grid.Normal[1])
    w('SensorUpZ').setValue(app.grid.Normal[2])

    w('SensorOriginX').setValue(-app.grid.Origin[0])
    w('SensorOriginY').setValue(-app.grid.Origin[1])
    w('SensorOriginZ').setValue(-app.grid.Origin[2])

    w('GridResolution').setValue(app.grid.Scale)
    w('GridWidth').setValue(app.grid.Scale*app.grid.GridNbTicks)
    w('GridLineWidth').setValue(app.grid.LineWidth)

    r = app.grid.Color[0] * 255
    g = app.grid.Color[1] * 255
    b = app.grid.Color[2] * 255
    w('GridColorPicker').setStyleSheet("background-color: rgb(" + str(r) + "," + str(g) + "," + str(b) +");")

    w('ShouldPropertiesPersist').setChecked(app.gridPropertiesPersist)

    def pickColor():
        colorPicker = QtGui.QColorDialog()
        colorPicker.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)

        if(colorPicker.exec()):
            qColor = colorPicker.selectedColor()

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

    app.grid.Normal = [w('SensorUpX').value, w('SensorUpY').value, w('SensorUpZ').value]
    app.grid.Origin = [-w('SensorOriginX').value, -w('SensorOriginY').value, -w('SensorOriginZ').value]
    app.grid.Scale = w('GridResolution').value
    app.grid.GridNbTicks = int(math.ceil(w('GridWidth').value / w('GridResolution').value))
    app.grid.LineWidth = w('GridLineWidth').value
    color = w('GridColorPicker').palette.color(QtGui.QPalette.Background)
    app.grid.Color = [color.redF(), color.greenF(), color.blueF()]
    app.gridPropertiesPersist = w('ShouldPropertiesPersist').checked

    return True
