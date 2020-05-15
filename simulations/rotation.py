import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.time = 25.0  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state

	def update(self):
		if self.simulation_state.curr_time <= 5:
			self.aircraft_state.psi += math.pi*2/250
		elif self.simulation_state.curr_time <= 10:
			self.aircraft_state.theta += math.pi*2/250
		elif self.simulation_state.curr_time <= 15:
			self.aircraft_state.phi += math.pi*2/250
		elif self.simulation_state.curr_time <= 17:
			self.aircraft_state.psi += math.pi/400
		elif self.simulation_state.curr_time <= 19:
			self.aircraft_state.theta += math.pi/400
		elif self.simulation_state.curr_time <= 21:
			self.aircraft_state.phi += math.pi/400