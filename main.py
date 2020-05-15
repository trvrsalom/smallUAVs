from Simulation import Simulation
import argparse

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--sim", help="Simulation file to run")
	parser.add_argument("-noview", action="store_true")
	parser.add_argument("-noplot", action="store_true")
	parser.add_argument("-nogui", action="store_true")
	args = parser.parse_args()
	sim = Simulation(args.sim)
	sim.simulation_state.show_viewer = not args.noview
	sim.simulation_state.show_plotter = not args.noplot
	if args.nogui:
		sim.simulation_state.show_plotter = False
		sim.simulation_state.show_viewer = False
	sim.run()
