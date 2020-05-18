import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.speed = 1 # m/s
		self.time = 8  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state
		
	def update(self):
		if self.simulation_state.curr_time <= 1:
			self.aircraft_state.fz = -10
		elif self.simulation_state.curr_time <= 2:
			self.aircraft_state.fz = 0
		elif self.simulation_state.curr_time <= 3:
			self.aircraft_state.fz = 10
		elif self.simulation_state.curr_time <= 4:
			self.aircraft_state.fz = 0
			self.aircraft_state.theta	= -math.pi/4
		elif self.simulation_state.curr_time <= 5:
			self.aircraft_state.fx = -10
		elif self.simulation_state.curr_time <= 6:
			self.aircraft_state.fx = 0
		elif self.simulation_state.curr_time <= 7:
			self.aircraft_state.fx = 10
		else: 
			self.aircraft_state.fx = 0
