from scipy.spatial.transform import Rotation
import numpy as np
from veloview.camera_path import *
import os

N_frames = 100
output_directory = "/mnt/ramdisk"


# example of ruler
r1 = lv.createRuler()
r1.Point1WorldPosition = [-1.0, 0.0, 0.0]
r1.Point2WorldPosition = [1.0, 0.0, 0.0]
r1.Visibility = True


# example of camera path
center = [0.0, 0.0, 0.0]
up_vector = [0.0, 0.0, 1.0]
initial_pos = [0.0, -5.0, 2.0]
focal_point = [0.0, 0.0, 0.0]
c = AbsoluteOrbit(0, N_frames,
                  center=center,
                  up_vector=up_vector,
                  initial_pos = initial_pos,
                  focal_point=focal_point)


view = GetRenderView()
view.Background = [0, 0, 0]
view.Background2 = [0, 0, 0.2]

for i in range(0, N_frames):
    view.CameraPosition = c.interpolate_position(i, None, None, np.asarray(list(view.CameraPosition)))
    view.CameraFocalPoint = c.interpolate_focal_point(i, None, None)
    view.CameraViewUp = c.interpolate_up_vector(i, None,)
    Render()

    imageName = os.path.join(output_directory, "image_%04d.png" % (i))
    WriteImage(imageName)
    print i
