import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector
import numpy as np

class Viewer():
	def __init__(self, simulation_state, aircraft_state, app):
		self.app = app
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state
		self.define_colors()
		self.app = pg.QtGui.QApplication([])
		self.window = gl.GLViewWidget()
		self.left_winglet_trail = gl.GLLinePlotItem(color=tuple(self.red.tolist()), width=3, antialias=True, mode="line_strip")
		self.right_winglet_trail = gl.GLLinePlotItem(color=tuple(self.green.tolist()), width=3, antialias=True, mode="line_strip")
		self.left_winglet_trail.setGLOptions('translucent')
		self.right_winglet_trail.setGLOptions('translucent')
		self.window.setWindowTitle("Viewer")
		self.window.setGeometry(0, 0, 750, 750)
		ground = gl.GLGridItem(color='k')
		ground.scale(10, 10, 0)
		self.body = gl.GLMeshItem(drawEdges=False, smooth=False, computeNormals=False)
		self.window.addItem(ground)
		self.window.addItem(self.left_winglet_trail)
		self.window.addItem(self.right_winglet_trail)
		self.window.addItem(self.body)
		self.window.setCameraPosition(distance = 20);
		self.window.setBackgroundColor('k')
		self.window.show()
		self.window.raise_()
		self.uav_colors = self.get_uav_mesh_colors()

	def define_colors(self):
		self.red = np.array([1., 0., 0., 1])
		self.green = np.array([0., 0.5, 0., 1])
		self.blue = np.array([0., 0., 1., 1])
		self.yellow = np.array([1., 1., 0., 1])

	def update(self):
		# Draw the UAV
		self.uav_points = self.get_uav_points()
		self.uav_points = self.rotate(self.uav_points)
		self.uav_points = self.translate(self.uav_points)
		self.uav_mesh = self.get_uav_mesh(self.uav_points)
		self.body.setMeshData(vertexes=self.uav_mesh, faceColors=self.uav_colors)
		# Logging
		self.winglet_log(self.uav_points)
		# Draw winglet trails
		self.draw_winglet_trails(self.simulation_state.left_winglet_log, self.simulation_state.right_winglet_log)

	def winglet_log(self, points):
		right_loc = points[5]
		left_loc = points[4]
		self.simulation_state.right_winglet_log = np.append(self.simulation_state.right_winglet_log, [right_loc], axis=0)
		self.simulation_state.left_winglet_log = np.append(self.simulation_state.left_winglet_log, [left_loc], axis=0)

	def draw_winglet_trails(self, left_trail, right_trail):
		points = int(self.simulation_state.trail_length/self.simulation_state.delta)
		if(self.simulation_state.show_winglet_trails):
			self.right_winglet_trail.setData(pos=right_trail[-points:])
		if(self.simulation_state.show_winglet_trails):
			self.left_winglet_trail.setData(pos=left_trail[-points:])

	def translate(self, model):
		return model + np.array([self.aircraft_state.get_pn(), self.aircraft_state.get_pe(), -self.aircraft_state.get_pd()]).T

	def rotate(self, model):
		return (self.inertial_to_body() @ model.T).T

	def get_uav_points(self, scale=0.5):
		points = np.array([
			[.5, 0, 0],      # Nose 0
			[0, 1, 0],       # Leading Edge Left 1
			[0, 0, 0],       # Center 2
			[0, -1, 0],      # Leading Edge Right 3
			[-0.25, 1, 0],   # Trailing edge left 4
			[-0.25, -1, 0],  # Trailing edge right 5
			[-0.25, 1, 0.2], # Winglet top left 6
			[-0.25, -1, 0.2],# Winglet top right 7
		])
		points *= scale
		return points

	def get_uav_mesh(self, points):
		mesh = np.array([
			[points[0], points[1], points[2]], # Bottom front left wing
			[points[0], points[3], points[2]], # Bottom front right wing
			[points[2], points[1], points[4]], # Bottom back left wing
			[points[2], points[3], points[5]], # Bottom back right wing
			[points[1], points[4], points[6]], # Left winglet
			[points[3], points[5], points[7]], # Right winglet
		])
		return mesh

	def get_uav_mesh_colors(self):
		red = np.array([self.red, self.red, self.red])
		green = np.array([self.green, self.green, self.green])
		blue = np.array([self.blue, self.blue, self.blue])
		yellow = np.array([self.yellow, self.yellow, self.yellow])
		colors = np.array([
			blue, blue, blue, blue, red, green
		])
		return colors

	def vehicle_to_vehicle_one(self):
		cp = np.cos(self.aircraft_state.get_psi())
		sp = np.sin(self.aircraft_state.get_psi())
		R = np.array([
			[cp, -sp, 0],
			[sp,  cp, 0],
			[ 0,   0, 1]
		])
		return R.T

	def vehicle_one_to_vehicle_two(self):
		ct = np.cos(self.aircraft_state.get_theta())
		st = np.sin(self.aircraft_state.get_theta())
		R = np.array([
			[ ct, 0, st],
			[  0, 1,  0],
			[-st, 0, ct]
		])
		return R.T

	def vehicle_two_to_body(self):
		cp = np.cos(self.aircraft_state.get_phi())
		sp = np.sin(self.aircraft_state.get_phi())
		R = np.array([
			[1,  0, 0],
			[0, cp,sp],
			[0,-sp,cp]
		])
		return R.T

	def inertial_to_body(self):
		R = self.vehicle_to_vehicle_one() @ self.vehicle_one_to_vehicle_two() @ self.vehicle_two_to_body()
		return R

	def quit(self):
		self.app.quit()