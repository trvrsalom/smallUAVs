import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.len = 5 # Length of each block
		self.speed = 1 # m/s
		self.time = 7*self.len  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state
		self.simulation_state.trail_length = 7*self.len
		
	def update(self):
		if self.simulation_state.curr_time <= self.len:
			self.aircraft_state.n += self.delta*self.speed
		elif self.simulation_state.curr_time <= 2*self.len:
			self.aircraft_state.e += self.delta*self.speed
		elif self.simulation_state.curr_time <= 3*self.len:
			self.aircraft_state.d += self.delta*self.speed
		elif self.simulation_state.curr_time <= 4*self.len:
			self.aircraft_state.n -= self.delta*self.speed
		elif self.simulation_state.curr_time <= 5*self.len:
			self.aircraft_state.e -= self.delta*self.speed
		elif self.simulation_state.curr_time <= 6*self.len:
			self.aircraft_state.d -= self.delta*self.speed

		if self.simulation_state.curr_time <= 5:
			self.aircraft_state.psi += 360/250
		elif self.simulation_state.curr_time <= 10:
			self.aircraft_state.theta += 360/250
		elif self.simulation_state.curr_time <= 15:
			self.aircraft_state.phi += 360/250
		elif self.simulation_state.curr_time <= 17:
			self.aircraft_state.psi += 45/100
		elif self.simulation_state.curr_time <= 19:
			self.aircraft_state.theta += 45/100
		elif self.simulation_state.curr_time <= 21:
			self.aircraft_state.phi += 45/100