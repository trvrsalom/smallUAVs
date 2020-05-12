import dataclasses 

class SimulationState:
	time: float = 5      # Simulation runtime in seconds
	delta: float = 0.02  # Simulation delta in seconds
	curr_time: float = 0  # Current simulation time in seconds

	def __str__(self):
		ret = "Simulation State:"
		ret += "\n\tTime: " + str(self.curr_time)
		ret += "\n\tRuntime: " + str(self.time)
		ret += "\n\tDelta: " + str(self.delta)
		return ret