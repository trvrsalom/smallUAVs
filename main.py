from Simulation import Simulation
import argparse

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--sim", help="Simulation file to run")
	parser.add_argument("-noview", action="store_true")
	args = parser.parse_args()
	sim = Simulation(args.sim)
	sim.simulation_state.show_viewer = not args.noview
	sim.start()
