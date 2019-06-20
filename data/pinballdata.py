import pybullet as b
import pybullet_data, time, cv2, types, math, pkgutil
import numpy as np
PI = np.pi


class RGBSpace(object):
	def __init__(self, low=0., high=1., sep=5):
		rgb = np.mgrid[
			slice(low, high, np.complex(0, sep)),
			slice(low, high, np.complex(0, sep)),
			slice(low, high, np.complex(0, sep))]
		rgb = rgb.reshape(3, -1)
		index = np.logical_not(np.logical_and(rgb[0, :] == rgb[1, :], rgb[1, :] == rgb[2, :]))
		rgb = rgb[:, index]
		rgba = np.vstack([rgb, np.ones((1, rgb.shape[1]))])

		self.rgba = rgba.T.tolist()
		self.index = 0
		np.random.shuffle(self.rgba)

	def get_rgba(self):
		rgba_item = self.rgba[self.index]
		self.index = (self.index + 1) % len(self.rgba)
		if self.index == 0:
			np.random.shuffle(self.rgba)
		return rgba_item


class MeshObject(object):
	def __init__(self, filename, 
				 mass=0, scale=None, position=None, orientation=None, 
				 rgba_color=None, specular_color=None, visual_frame_position=None, visual_frame_orientation=None,
				 collision_frame_position=None, collision_frame_orientation=None, 
				 scale2mass=None):

		if isinstance(scale2mass, types.FunctionType):
			mass = scale2mass(scale)

		self.visual_shape = b.createVisualShape(
			b.GEOM_MESH,
			fileName=filename,
			meshScale=scale,
			rgbaColor=rgba_color,
			specularColor=specular_color,
			visualFramePosition=visual_frame_position,
			visualFrameOrientation=visual_frame_orientation)

		self.collision_shape = b.createCollisionShape(
			b.GEOM_MESH,
			fileName=filename,
			meshScale=scale,
			collisionFramePosition=collision_frame_position,
			collisionFrameOrientation=collision_frame_orientation)

		self.body = b.createMultiBody(
			baseMass=mass,
			baseCollisionShapeIndex=self.collision_shape,
			baseVisualShapeIndex=self.visual_shape,
			basePosition=position,
			baseOrientation=orientation)


class PinBallDataGenerator(object):
	def __init__(self, connect):
		self.egl_plugin = None

		if connect = b.DIRECT:
			self.client = b.connect(connect)
			# Try hardware acceleration
			egl = pkgutil.get_loader('eglRenderer')
			if egl is not None:
				self.egl_plugin = b.loadPlugin(egl.get_filename(), '_eglRendererPlugin')
		elif connect = b.GUI:
			self.client = b.connect(connect)




rgba_picker = RGBSpace()

# Connect to simulator service
id_client = b.connect(b.SHARED_MEMORY)
if id_client < 0:
	id_client = b.connect(b.GUI)

# Configure GUI
b.configureDebugVisualizer(b.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
b.configureDebugVisualizer(b.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
b.configureDebugVisualizer(b.COV_ENABLE_TINY_RENDERER, 0)

b.setAdditionalSearchPath(pybullet_data.getDataPath())
# b.setPhysicsEngineParameter(numSolverIterations=10)

# b.resetDebugVisualizerCamera(cameraDistance=10.0, cameraYaw=50., cameraPitch=-35, cameraTargetPosition=[-1., 0., 0.])
b.resetDebugVisualizerCamera(cameraDistance=5., cameraYaw=0., cameraPitch=0, cameraTargetPosition=[0., 0., 0.])
b.setGravity(0., 0., -10.)

# print('Is Numpy enabled? ', b.isNumpyEnabled())

plane = b.loadURDF('plane.urdf')
boundary = MeshObject('./arena.obj', mass=0, rgba_color=[.66, .66, .66, 1.])
ball = MeshObject('sphere_smooth.obj', mass=1, scale=[.6, .6, .6], position=[-4, -4, .6], rgba_color=rgba_picker.get_rgba())
wall1 = MeshObject('./cube.obj', mass=2, scale=[.8, .2, 2], position=[-2, 0, 0], orientation=b.getQuaternionFromEuler([0., 0., -PI/4]), rgba_color=rgba_picker.get_rgba())
wall2 = MeshObject('./cube.obj', mass=2, scale=[.6, .5, 1], position=[1, -2, 0], orientation=b.getQuaternionFromEuler([0., 0., 0]), rgba_color=rgba_picker.get_rgba())



# print('Plane lateral friction: ', b.getDynamicsInfo(plane, -1)[1])


b.changeDynamics(plane, -1, restitution=1., lateralFriction=1.0)
b.changeDynamics(boundary.body, -1, restitution=.999, lateralFriction=0.0)

b.changeDynamics(wall1.body, -1, restitution=.999, lateralFriction=0.0)

b.changeDynamics(ball.body, -1, restitution=.999, lateralFriction=0.0)
# b.resetBaseVelocity(ball.body, linearVelocity=[2., 0., 0.])



id_distance = b.addUserDebugParameter('distance', 1, 10, 5)
id_yaw = b.addUserDebugParameter('yaw', 0, 359, 180)
id_pitch = b.addUserDebugParameter('pitch', -90, -10, -35)


# gui_camera_state = b.getDebugVisualizerCamera()

target = [0., 0., 0.]
distance = 5.
degree_yaw = 225
degree_pitch = -35.
degree_roll = 15.
view_matrix = b.computeViewMatrixFromYawPitchRoll(target, distance, degree_yaw, degree_pitch, 0, 2)
projection_matrix = [0.7499999403953552, 0., 0., 0., 0., 1., 0., 0., 0., 0., -1.0000200271606445, -1., 0., 0., -0.02000020071864128, 0.]

for t in range(32 * 240):

	distance = b.readUserDebugParameter(id_distance)
	degree_yaw = b.readUserDebugParameter(id_yaw)
	degree_pitch = b.readUserDebugParameter(id_pitch)
	view_matrix = b.computeViewMatrixFromYawPitchRoll(target, distance, degree_yaw, degree_pitch, 0, 2)

	radian_pitch = np.deg2rad(degree_pitch)
	radian_yaw = np.deg2rad(degree_yaw - 90)
	z = - np.sin(radian_pitch) * distance + target[2]
	x = np.cos(radian_pitch) * np.cos(radian_yaw) * distance + target[0]
	y = np.cos(radian_pitch) * np.sin(radian_yaw) * distance + target[1]
	print(x, y, z)

	width, height, rgb, depth, mask = b.getCameraImage(64, 64, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=b.ER_BULLET_HARDWARE_OPENGL)

	if t < 240:
		ball_pos, _ = map(np.array, b.getBasePositionAndOrientation(ball.body))
		wall1_pos, _ = map(np.array, b.getBasePositionAndOrientation(wall1.body))
		force = np.array([1., 1., 1.])  # (wall1_pos - ball_pos)
		force = force / np.linalg.norm(force) * 4
		b.applyExternalForce(ball.body, -1, force, ball_pos, b.WORLD_FRAME)
	
	# if t == 0 or (t + 1) % 120 == 0:
		# position, orientation = b.getBasePositionAndOrientation(ball.body)
		# linear_velocity, angular_velocity = b.getBaseVelocity(ball.body)
		# print('T={} position: '.format((t+1) % 240), position)
		# print('LV=', linear_velocity)
		# print('AV=', angular_velocity)
		# width, height, rgb, depth, mask = b.getCameraImage(64, 64, viewMatrix=view_matrix, projectionMatrix=gui_camera_state[3], renderer=b.ER_BULLET_HARDWARE_OPENGL)

		# time.sleep(1.)

	b.stepSimulation()


b.disconnect()

import pdb
pdb.set_trace()
