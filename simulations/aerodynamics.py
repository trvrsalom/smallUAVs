import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.time = 6  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state

	def start(self):
		#self.aircraft_state.theta = math.pi/4
		self.aircraft_state.u = 10
		
	def update(self):
		self.simulation_state.wn = 0
		self.aircraft_state.dT = 1
		self.aircraft_state.dE = 0.4
		#self.aircraft_state.dA = 0.1