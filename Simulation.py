import sys
from Viewer import Viewer
from importlib import import_module
from dataclasses.SimulationState import SimulationState
from dataclasses.AircraftState import AircraftState
import Simulation

class Simulation:
	def __init__(self, simfile=None):
		if simfile is not None:
			try: 
				print("Loading simulation file: " + simfile)
				globals()["sim"] = import_module(simfile)
				self.sim = sim.Sim()
			except:
				print("Failed to load simulation file")
				self.sim = None
		else:
			print("No simulation file provided")
			self.sim = None
		self.simulation_state = SimulationState()
		self.aircraft_state = AircraftState()
		self.viewer = Viewer()

	def load_state(self):
		if self.sim is not None:
			if hasattr(self.sim, "time"):
				self.simulation_state.time = self.sim.time
			if hasattr(self.sim, "delta"):
				self.simulation_state.delta = self.sim.delta

	def sim_loop(self):
		while True:
			self.viewer.update(self.simulation_state, self.aircraft_state)

	def start(self):
		self.load_state()
		self.sim_loop()