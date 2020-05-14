import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector
import numpy as np

class Viewer():
	def __init__(self, simulation_state, aircraft_state):
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state
		self.app = pg.QtGui.QApplication([])
		self.window = gl.GLViewWidget()
		self.window.setWindowTitle("Viewer")
		self.window.setGeometry(0, 0, 750, 750)
		ground = gl.GLGridItem(color='k')
		ground.scale(10, 10, 0)
		self.body = gl.GLMeshItem(drawEdges=False, smooth=False, computeNormals=False)
		self.window.addItem(self.body)
		self.window.addItem(ground)
		self.window.setCameraPosition(distance = 20);
		self.window.setBackgroundColor('w')
		self.window.show()
		self.window.raise_()
		self.uav_colors = self.get_uav_mesh_colors()

	def update(self):
		#self.uav_points = self.vehicle_two_to_body() @  self.vehicle_one_to_vehicle_two() @ self.vehicle_to_vehicle_one() @ self.get_uav_points()
		#self.uav_points = self.uav_points.T
		self.rotate()
		self.uav_mesh = self.get_uav_mesh(self.uav_points.T)
		self.body.setMeshData(vertexes=self.uav_mesh, faceColors=self.uav_colors)
		self.app.processEvents()

	def rotate(self):
		self.uav_points = self.inertial_to_body() @ self.get_uav_points().T

	def get_uav_points(self, scale=10):
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
		red = np.array([1., 0., 0., 1])
		green = np.array([0., 1., 0., 1])
		blue = np.array([0., 0., 1., 1])
		yellow = np.array([1., 1., 0., 1])
		red = np.array([red, red, red])
		green = np.array([green, green, green])
		blue = np.array([blue, blue, blue])
		yellow = np.array([yellow, yellow, yellow])
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