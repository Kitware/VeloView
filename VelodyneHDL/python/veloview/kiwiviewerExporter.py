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
import json
import shutil
import zipfile
import contextlib
import os
import tempfile
import datetime


def colorMapProperties(t):
    p = {}
    p['rgb_points'] = list(t.RGBPoints)
    if t.ColorSpace == 'HSV' and t.HSVWrap:
        p['color_space'] = 'WrappedHSV'
    else:
        p['color_space'] = t.ColorSpace
    p['number_of_values'] = t.NumberOfTableValues
    p['vector_component'] = t.VectorComponent
    return p


def cameraProperties(view):
    p = {}
    p['focal_point'] = list(view.CameraFocalPoint)
    p['position'] = list(view.CameraPosition)
    p['view_up'] = list(view.CameraViewUp)
    p['view_angle'] = view.CameraViewAngle
    if view.CameraParallelProjection:
        p['parallel_scale'] = list(view.CameraParallelScale)
    return p


def getObjectMetaData(rep):

    metaData = {}
    if rep.ColorArrayName:
        metaData['color_map'] = colorMapProperties(rep.LookupTable)
        metaData['color_by'] = rep.ColorArrayName[1]
    else:
        metaData['color'] = list(rep.DiffuseColor)

    return metaData


def getSceneMetaData(view):

    scene = {}
    objects = []

    scene['background_color'] = list(view.Background)
    if view.UseGradientBackground:
        # switch the ordering, paraview's Background2 means the top gradient color
        scene['background_color'] = list(view.Background2)
        scene['background_color2'] = list(view.Background)
    if view.BackgroundTexture:
        scene['background_image'] = exportTexture(view.BackgroundTexture, baseDir)

    scene['camera'] = cameraProperties(view)

    return scene


def writeJsonData(outDir, view, rep, dataFilenames):

    scene = getSceneMetaData(view)

    objectMetaData = getObjectMetaData(rep)
    objectMetaData['point_size'] = 2

    if len(dataFilenames) > 1:
        objectMetaData['filenames'] = dataFilenames
        objectMetaData['frames_per_second'] = 18
    else:
        objectMetaData['filename'] = dataFilenames[0]

    scene['objects'] = [objectMetaData]

    jsonStr = json.dumps(scene, indent=4)

    sceneFile = os.path.join(outDir, 'scene.kiwi')
    open(sceneFile, 'w').write(jsonStr)


def zipDir(inputDirectory, zipName):

    assert os.path.isdir(inputDirectory)

    inputDirectory = os.path.abspath(inputDirectory)
    parentDirectory = os.path.dirname(inputDirectory) + os.sep

    with contextlib.closing(zipfile.ZipFile(zipName, 'w', zipfile.ZIP_DEFLATED)) as archive:
        for root, dirs, files in os.walk(inputDirectory):
            for filename in files:
                absoluteFilename = os.path.join(root, filename)
                relativeFilename = absoluteFilename[len(parentDirectory):]
                archive.write(absoluteFilename, relativeFilename)

