import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.speed = 1 # m/s
		self.time = 16  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state
		self.simulation_state.g = 0
		
	def update(self):
		# Try testing with Jxz at 0 and default to test yaw/roll coupling
		if self.simulation_state.curr_time <= 1:
			self.aircraft_state.fx = -10
		elif self.simulation_state.curr_time <= 2:
			self.aircraft_state.fx = 0
		elif self.simulation_state.curr_time <= 3:
			self.aircraft_state.fx = 10
		elif self.simulation_state.curr_time <= 4:
			self.aircraft_state.fx = 0
		elif self.simulation_state.curr_time <= 5:
			self.aircraft_state.m = 0.5
		elif self.simulation_state.curr_time <= 6:
			self.aircraft_state.m = 0
		elif self.simulation_state.curr_time <= 7:
			self.aircraft_state.m = -0.5
		elif self.simulation_state.curr_time <= 8:
			self.aircraft_state.m = 0
			self.aircraft_state.p = 0
			self.aircraft_state.q = 0
			self.aircraft_state.r = 0
		elif self.simulation_state.curr_time <= 9:
			self.aircraft_state.l = 0.5
		elif self.simulation_state.curr_time <= 10:
			self.aircraft_state.l = 0
		elif self.simulation_state.curr_time <= 11:
			self.aircraft_state.l = -0.5
		elif self.simulation_state.curr_time <= 12:
			self.aircraft_state.l = 0
			self.aircraft_state.p = 0
			self.aircraft_state.q = 0
			self.aircraft_state.r = 0
		elif self.simulation_state.curr_time <= 13:
			self.aircraft_state.n = 0.5
		elif self.simulation_state.curr_time <= 14:
			self.aircraft_state.n = 0
		elif self.simulation_state.curr_time <= 15:
			self.aircraft_state.n = -0.5
		elif self.simulation_state.curr_time <= 16:
			self.aircraft_state.n = 0