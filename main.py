from Simulation import Simulation
import argparse

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--sim", help="Simulation file to run")
	parser.add_argument("--simtime", help="Override the provided or default simulation time", type=float)
	parser.add_argument("-noview", help="Disable the simulation viewer window", action="store_true")
	parser.add_argument("-noplot", help="Disable the plot viewer window", action="store_true")
	parser.add_argument("-nogui", help="Disable all viewer windows", action="store_true")
	parser.add_argument("-aar", help="After action report. Show the plots after the simulation if they werent shown during", action="store_true")
	args = parser.parse_args()
	sim = Simulation(args.sim)
	sim.simulation_state.show_viewer = not args.noview
	sim.simulation_state.show_plotter = not args.noplot
	if args.simtime:
		sim.simulation_state.time = args.simtime
	if args.nogui:
		sim.simulation_state.show_plotter = False
		sim.simulation_state.show_viewer = False
	if args.nogui or args.noplot:
		if args.aar:
			sim.simulation_state.aar = True
	sim.run()
