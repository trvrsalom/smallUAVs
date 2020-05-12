from Simulation import Simulation
import argparse

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--sim", help="Simulation file to run")
	args = parser.parse_args()
	sim = Simulation(args.sim)
	sim.start()
