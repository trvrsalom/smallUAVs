import math

class Sim:
	def __init__(self, simulation_state, aircraft_state):
		self.time = 20  # Simulation time in seconds
		self.delta = 0.02 # Simulation delta in seconds
		self.simulation_state = simulation_state
		self.aircraft_state = aircraft_state

	def update(self):
		#self.aircraft_state.r = 10
		self.aircraft_state.fy = 1000