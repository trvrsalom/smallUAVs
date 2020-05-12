import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector
import numpy as np

class Viewer():
	def __init__(self):
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
		self.uav_mesh, self.uav_colors = self.draw_uav()

	def update(self):
		self.body.setMeshData(vertexes=self.uav_mesh, faceColors=self.uav_colors)
		self.app.processEvents()

	def draw_uav(self, scale=10):
		red = np.array([1., 0., 0., 1])
		green = np.array([0., 1., 0., 1])
		blue = np.array([0., 0., 1., 1])
		yellow = np.array([1., 1., 0., 1])
		red = np.array([red, red, red])
		green = np.array([green, green, green])
		blue = np.array([blue, blue, blue])
		yellow = np.array([yellow, yellow, yellow])

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

		mesh = np.array([
			[points[0], points[1], points[2]], # Bottom front left wing
			[points[0], points[3], points[2]], # Bottom front right wing
			[points[2], points[1], points[4]], # Bottom back left wing
			[points[2], points[3], points[5]], # Bottom back right wing
			[points[1], points[4], points[6]], # Left winglet
			[points[3], points[5], points[7]], # Right winglet
		])

		colors = np.array([
			blue, blue, blue, blue, red, green
		])

		return mesh, colors
