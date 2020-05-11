import pyqtgraph
import pyqtgraph.opengl as gl
import pyqtgraph.Vector
import numpy as np

class Viewer():
	def __init__(self):
		self.app = pyqtgraph.QtGui.QApplication([])
		self.window = gl.GLViewWidget()
		self.window.setWindowTitle("Viewer")
		self.window.setGeometry(0, 0, 500, 500)
		ground = gl.GLGridItem()
		ground.scale(20, 20, 0)
		self.window.addItem(ground)
		self.window.setCameraPosition(distance = 200);
		self.window.setBackgroundColor('k')
		self.window.show()
		self.window.raise_()

	def update(self):
		self.app.processEvents()