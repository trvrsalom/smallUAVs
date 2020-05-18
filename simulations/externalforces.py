import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.speed = 1 # m/s
		self.time = 8  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state
		
	def update(self):
		self.aircraft_state.theta = math.pi/4
		self.aircraft_state.phi = math.pi/4