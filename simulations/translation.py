class Sim:
	def __init__(self, simulation_state, aircraft_state):
		time = 10.0  # Simulation time in seconds
		delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state

	def update(self):
		pass
