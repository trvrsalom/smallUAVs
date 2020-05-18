import dataclasses 
from numpy import ndarray,empty

class SimulationState:
	# Simulation parameters
	time: float = 5      # Simulation runtime in seconds
	delta: float = 0.02  # Simulation delta in seconds
	curr_time: float = 0  # Current simulation time in seconds
	g: float = 9.8
	# Show simulation viewer
	show_viewer: bool = True
	show_plotter: bool = True
	# Log
	# - Winglet Trails
	right_winglet_log: ndarray = empty((1,3))
	left_winglet_log: ndarray = empty((1,3))
	show_winglet_trails: bool = True
	trail_length: float = 60 # Seconds

	def __str__(self):
		ret = "Simulation State:"
		ret += "\n\tTime: " + str(self.curr_time)
		ret += "\n\tRuntime: " + str(self.time)
		ret += "\n\tDelta: " + str(self.delta)
		return ret