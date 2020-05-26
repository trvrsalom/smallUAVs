import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.time = 6  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state
		self.simulation_state.g = 0
		
	def update(self):
		self.simulation_state.wn = -1
		if self.simulation_state.curr_time <= 2:
			self.aircraft_state.q = math.pi
		elif self.simulation_state.curr_time <= 4:
			self.aircraft_state.q = 0
			self.aircraft_state.r = math.pi
		else:
			self.aircraft_state.r = 0
		'''
		if self.simulation_state.curr_time <= 2:
			self.simulation_state.wn = -1
		elif self.simulation_state.curr_time <= 4:
			self.aircraft_state.theta = math.pi/4
		elif self.simulation_state.curr_time <= 6:
			self.aircraft_state.theta = -math.pi/4
		elif self.simulation_state.curr_time <= 8:
			self.aircraft_state.theta = 0
			self.aircraft_state.psi = math.pi/4
'''
