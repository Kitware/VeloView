from scipy.spatial.transform import Rotation
import numpy as np

"""
=======================================================================================================
Camera path classes.
It defines a camera path with a start and end timestep.

To add a new camera path, you need to create a new class deriving from CameraPath and
define the following methods: raw_position, raw_vector_up and raw_focal_point

The position, up vector and focal points will be automatically interpolated if a transition is defined.

The complete animation holds a list of consecutive camera path. The start and end timesteps of
each camera path need to be coherent.
=======================================================================================================
"""

# global variable defining the orientation of the camera w.r.t the lidar
R_cam_to_lidar = Rotation.from_dcm(np.eye(3))	# identity by default

class CameraPath:
	""" Base class for camera path.
		The following interpolation modes are available:
			- linear
			- square
			- s-shape
	"""
	def __init__(self, start, end):
		self.start = start
		self.end = end

		# default
		self.previous_camera_path = None
		self.transition_duration = 0
		self.transition_mode = "linear"

	def timestep_inside_range(self, t):
		""" Check if the timestep is inside the camera range """
		return t >= self.start and t < self.end

	def set_transition(self, previous_camera_path, duration, mode):
		""" set the transition parameters from the previous camera path """
		self.transition_duration = duration
		self.transition_mode = mode
		self.previous_camera_path = previous_camera_path

	def compute_transition_weight(self, step):
		""" return the weight corresponding to the current timestep """
		if self.transition_duration == 0:
			return 1.0
		w = max(0.0, min(1.0, float(step - self.start) / self.transition_duration))
		if self.transition_mode == "linear":
			return w
		elif self.transition_mode == "square":
			return w*w
		elif self.transition_mode == "s-shape":
			beta = 2
			if w < 1e-5:
				return 0.0
			elif w > 1.0 - 1e-5:
				return 1.0
			else:
				return 1.0 / (1.0 + (w / (1.0 - w)) ** (-beta))
		else:
			print "Invalid transition mode: ", self.transition_mode
			return 0.0

	def interpolate_position(self, step, R_lidar, T_lidar, prev_cam_pos):
		""" Interpolate the position at the passed timestep using the transition """
		T_lidar = np.asarray(T_lidar)
		pos = self.raw_position(step, R_lidar, T_lidar, prev_cam_pos)
		if self.previous_camera_path is not None:
			prev_pos = self.previous_camera_path.raw_position(step, R_lidar, T_lidar, prev_cam_pos)
			weight = self.compute_transition_weight(step)
			return pos * weight + (1.0 - weight) * prev_pos
		else:
			return pos

	def interpolate_up_vector(self, step, R_lidar):
		""" Interpolate the up vector at the passed timestep using the transition """
		up = self.raw_up_vector(R_lidar)
		if self.previous_camera_path is not None:
			# interpolate up vector and prev_up vector using weight
			prev_up = self.previous_camera_path.raw_up_vector(R_lidar)
			axis = np.cross(prev_up, up)
			r = np.arctan2(np.linalg.norm(axis), np.dot(prev_up, up))
			R = Rotation.from_rotvec(axis * (self.compute_transition_weight(step) * r))
			return R.apply(prev_up)
		else:
			return up

	def interpolate_focal_point(self, step, R_lidar, T_lidar):
		""" Interpolate the focal point at the passed timestep using the transition """
		T_lidar = np.asarray(T_lidar)
		fp = self.raw_focal_point(R_lidar, T_lidar)
		if self.previous_camera_path is not None:
			prev_fp = self.previous_camera_path.raw_focal_point(R_lidar, T_lidar)
			weight = self.compute_transition_weight(step)
			return fp * weight + (1.0 - weight) * prev_fp
		else:
			return fp


class ThirdPersonView(CameraPath):
	""" Third person view defined with a position, a focal point
		and a up vector relative to the camera frame """
	def __init__(self, start, end,
				 position=np.array([0.0, 0.0, -2.0]),
				 focal_point=np.array([0.0, 0.0, 1.0]),
				 up_vector=np.array([0.0, -1.0, 0.0])):
		CameraPath.__init__(self, start, end)
		self.position = position
		self.focal_point = focal_point
		self.up_vector = up_vector
		self.type = "Third Person View"

	def raw_position(self, step, R_lidar, T_lidar, prev_cam_pos):
		return T_lidar + R_lidar.apply(R_cam_to_lidar.apply(self.position))

	def raw_up_vector(self, R_lidar):
		return R_lidar.apply(R_cam_to_lidar.apply(self.up_vector))

	def raw_focal_point(self, R_lidar, T_lidar):
		return T_lidar + R_lidar.apply(R_cam_to_lidar.apply(self.focal_point))


class FirstPersonView(ThirdPersonView):
	""" Specialized class for first person view """
	def __init__(self, start, end,
				 focal_point=np.array([0.0, 0.0, 1.0]),
				 up_vector=np.array([0.0, -1.0, 0.0])):
		ThirdPersonView.__init__(self, start, end, focal_point=focal_point, up_vector=up_vector)
		self.position = np.array([0.0, 0.0, 0.0])
		self.type = "First Person View"


class FixedPositionView(CameraPath):
	""" Fixed absolute position camera. """
	def __init__(self, start, end, position=None, focal_point=None, up_vector=np.array([0.0, 0.0, 1.0])):
		CameraPath.__init__(self, start, end)
		self.position = position
		self.focal_point = focal_point
		self.up_vector = up_vector
		self.type = "Fixed position"

	def raw_position(self, step, R_lidar, T_lidar, prev_cam_pos):
		""" if no position was specified the camera position at the start of the camera path is used """
		if self.position is not None:
			return self.position
		else:
			return prev_cam_pos

	def raw_up_vector(self, R_lidar):
		return self.up_vector

	def raw_focal_point(self, R_lidar, T_lidar):
		""" if no focal point was specified, the lidar position is used """
		if self.focal_point is not None:
			return self.focal_point
		else:
			return T_lidar


class AbsoluteOrbit(CameraPath):
	""" Absolute orbit defined by a center of rotation (center), a rotation axis (up_vector),
		an initial position (initial_pos) and a focal point (focal_point). By default,
		the rotation is counter-clockwise. """
	def __init__(self, start, end, center, initial_pos, up_vector, focal_point, ccw=1):
		CameraPath.__init__(self, start, end)
		self.center = np.asarray(center, dtype=np.float64)
		self.up_vector = np.asarray(up_vector, dtype=np.float64)
		self.up_vector /= np.linalg.norm(self.up_vector)
		self.initial_pos = np.asarray(initial_pos, dtype=np.float64)
		self.focal_point = np.asarray(focal_point, dtype=np.float64)
		self.ccw = ccw
		self.type = "Absolute Orbit"

	def raw_position(self, step, R_lidar, T_lidar, prev_cam_pos):
		n = self.end - self.start
		d_angle = 2 * np.pi / n
		angle = self.ccw * (step - self.start) * d_angle
		v = self.initial_pos - self.center
		angle_axis = self.up_vector * angle
		R = Rotation.from_rotvec(angle_axis)
		return self.center + R.apply(v)

	def raw_up_vector(self, R_lidar):
		return self.up_vector

	def raw_focal_point(self, R_lidar, T_lidar):
		return self.focal_point


class RelativeOrbit(CameraPath):
	""" Relative orbit defined by an initial position relative and a up vector to the camera frame.
		By default, the rotation is counter-clockwise """
	def __init__(self, start, end, initial_pos, up_vector, ccw=1):
		CameraPath.__init__(self, start, end)
		self.up_vector = np.asarray(up_vector, dtype=np.float64)
		self.up_vector /= np.linalg.norm(self.up_vector)
		self.initial_pos = np.asarray(initial_pos, dtype=np.float64)
		self.ccw = ccw
		self.type = "Relative Orbit"

	def raw_position(self, step, R_lidar, T_lidar, prev_cam_pos):
		n = self.end - self.start
		d_angle = 2 * np.pi / n
		angle = self.ccw * (step - self.start) * d_angle
		v = self.initial_pos
		angle_axis = self.up_vector * angle
		R = Rotation.from_rotvec(angle_axis)
		return T_lidar + R.apply(v)

	def raw_up_vector(self, R_lidar):
		return self.up_vector

	def raw_focal_point(self, R_lidar, T_lidar):
		return T_lidar

