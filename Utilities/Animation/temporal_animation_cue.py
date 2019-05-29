import os
import math
import numpy as np
import paraview.simple as smp
from scipy.spatial.transform import Rotation
from veloview.camera_path import *
from vtk.util import numpy_support

# This script defines the animation
# You NEED to set the correct names below as well as the correct orientation R_cam_to_lidar
# The camera paths can be changed (at the end of the function start_cue)

# -----------------------------TO MODIFY--------------------------------------

# Pipeline parameters. They need to be filled with the names in your GUI pipeline.
temporal_source_name = "Data"
pointclouds_name = "TrailingFrame1"
trajectory_name = "la-doua-lidar-slam-version2.poses"
cad_model_name = "Transform1"

# Camera / Lidar orientation (need to be specified)
# rotation which transforms the lidar tri-axe into the camera tri-axe
#veloview.camera_path.R_cam_to_lidar = Rotation.from_euler('XYZ', [0.0, -90.0, 90.0], degrees=True) 		# example calibration ENS drone
veloview.camera_path.R_cam_to_lidar = Rotation.from_euler('ZYZ', [8, 90.0, -90.0], degrees=True)			# example calibration la doua car


# view parameters
CameraParallelProjection = 0
Background = [0.0, 0.0, 0.0]
Background2 = [0.0, 0.0, 0.2]
UseGradientBackground = 1
CameraViewAngle = 60
opacity = 0.3


# Output directory for the generated frames ("" to disable saving)
frames_output_dir = ""
# ----------------------------------------------------------------------------

def start_cue(self):
	""" function called at the beginning of the animation """
	self.image_index = 0	# image index

	# setup view parameters
	view = smp.GetActiveView()
	view.CameraParallelProjection = CameraParallelProjection
	view.Background = Background
	view.Background2 = Background2
	view.UseGradientBackground = UseGradientBackground
	view.CameraViewAngle = CameraViewAngle

	# setup initial opacity
	frames = smp.FindSource(pointclouds_name)
	self.frames_repr = smp.Show(frames)
	self.frames_repr.Opacity = opacity

	# get trajectory positions and orientations
	trajectory = smp.FindSource(trajectory_name)
	traj = trajectory.GetClientSideObject().GetOutput()
	self.pts = numpy_support.vtk_to_numpy(traj.GetPoints().GetData()).copy()

	# convert veloview axis angle to scipy Rotation
	orientations_data = traj.GetPointData().GetArray("Orientation(AxisAngle)")
	orientations = numpy_support.vtk_to_numpy(orientations_data).copy()
	axis = orientations[:, :3]
	angles = orientations[:, 3].reshape((-1, 1))
	axis_angles = axis * angles
	self.orientations = [Rotation.from_rotvec(a) for a in axis_angles]


	# get the 3D model
	self.model = None
	if len(cad_model_name) > 0:
		self.model = smp.FindSource(cad_model_name)


	# get all available timesteps and find the index corresponding to the start time
	time = view.ViewTime
	source_frames = smp.FindSource(temporal_source_name)
	timesteps = list(source_frames.TimestepValues)
	self.i = np.argmin(np.abs(np.asarray(timesteps) - time))
	print "Start timestep: ", self.i



# -----------------------------TO MODIFY--------------------------------------
	# Camera path definition (need to be specified)
	# This is an example, you can define your custom camera path
	c1 = FirstPersonView(self.i, self.i+40, focal_point=[0, 0, 1])

	c2 = FixedPositionView(self.i+40, self.i+100)
	c2.set_transition(c1, 5, "s-shape")		# transition from c1

	c3 = AbsoluteOrbit(self.i+100, self.i+200,
						center=[99.65169060331509, 35.559305816556, 37.233268868598536],
						up_vector=[0, 0, 1.0],
						initial_pos = [85.65169060331509, 35.559305816556, 37.233268868598536],
						focal_point=[99.65169060331509, 35.559305816556, 7.233268868598536])
	c3.set_transition(c2, 20, "s-shape")

	c4 = ThirdPersonView(self.i+200, self.i+280)
	c4.set_transition(c3, 20, "s-shape")

	c5 = RelativeOrbit(self.i+280, self.i+350, up_vector=[0, 0, 1.0], initial_pos = [0.0, -10, 10])
	c5.set_transition(c4, 20, "square")

	self.cameras = [c1, c2, c3, c4, c5]
# ----------------------------------------------------------------------------

def tick(self):
	""" function called at each timestep """
	view = smp.GetActiveView()

	# lidar orientation and position
	R_l = self.orientations[self.i]
	T_l = self.pts[self.i, :]

	# move camera
	for c in self.cameras:
		if c.timestep_inside_range(self.i):
			print c.type
			view.CameraPosition = c.interpolate_position(self.i, R_l, T_l, np.asarray(list(view.CameraPosition)))
			view.CameraFocalPoint = c.interpolate_focal_point(self.i, R_l, T_l)
			view.CameraViewUp = c.interpolate_up_vector(self.i, R_l)
			break

	# move 3d model (vtk Transform rotation are angle axis in degrees)
	if self.model is not None:
		self.model.Transform.Translate = self.pts[self.i, :3]
		o = R_l.as_rotvec()
		angle_rad = np.linalg.norm(o)
		angle_deg = np.rad2deg(angle_rad)
		self.model.Transform.Rotate = o * angle_deg / angle_rad

	smp.Render()

	# save frame
	if len(frames_output_dir) > 0:
		imageName = os.path.join(frames_output_dir, "image_%04d.png" % (self.image_index))
		smp.WriteImage(imageName)

	self.image_index += 1
	self.i += 1


def end_cue(self):
	""" function called at the end of an animation """
	pass
