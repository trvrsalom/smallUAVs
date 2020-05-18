import sys
import pyqtgraph as pg
from Viewer import Viewer
from importlib import import_module
from dataclasses.SimulationState import SimulationState
from dataclasses.AircraftState import AircraftState
from dataclasses.Zagi import Zagi
from utils import angles
import numpy as np
import math
import Simulation
from utils.Log import Log
from Plotter import Plotter
import faulthandler

class Simulation:
	def __init__(self, simfile=None):
		faulthandler.enable()
		self.app = pg.QtGui.QApplication([])
		self.simulation_state = SimulationState()
		self.aircraft_state = AircraftState()
		self.aircraft_state.set_airframe(Zagi())
		if simfile is not None:
			try:
				simfile = simfile.replace("/", ".").replace(".py", "")
				print("Loading simulation file: " + simfile)
				globals()["sim"] = import_module(simfile)
				self.sim = sim.Sim(self.simulation_state, self.aircraft_state)
			except Exception as e:
				print("Failed to load simulation file: " + str(e))
				self.sim = None
		else:
			print("No simulation file provided")
			self.sim = None
		self.init_log()

	def load_state(self):
		if self.sim is not None:
			if hasattr(self.sim, "time"):
				self.simulation_state.time = self.sim.time
			if hasattr(self.sim, "delta"):
				self.simulation_state.delta = self.sim.delta

	def init_log(self):
		time_source = lambda: self.simulation_state.curr_time
		log = Log()
		log.set_time_source(time_source)
		#log.add_data_source("pn", self.aircraft_state.get_pn)
		#log.add_data_source("pe", self.aircraft_state.get_pe)
		#log.add_data_source("pd", self.aircraft_state.get_pd)
		#log.add_data_source("phi", self.aircraft_state.get_phi)
		#log.add_data_source("theta", self.aircraft_state.get_theta)
		#log.add_data_source("psi", self.aircraft_state.get_psi)
		#log.add_data_source("p", self.aircraft_state.get_p)
		#log.add_data_source("q", self.aircraft_state.get_q)
		#log.add_data_source("r", self.aircraft_state.get_r)
		#log.add_data_source("u", self.aircraft_state.get_u)
		#log.add_data_source("v", self.aircraft_state.get_v)
		#log.add_data_source("w", self.aircraft_state.get_w)
		#log.add_data_source("fx", self.aircraft_state.get_fx)
		#log.add_data_source("fy", self.aircraft_state.get_fy)
		#log.add_data_source("fz", self.aircraft_state.get_fz)
		log.add_data_source("l", self.aircraft_state.get_l)
		log.add_data_source("m", self.aircraft_state.get_m)
		log.add_data_source("n", self.aircraft_state.get_n)
		self.log = log

	def sim_loop(self):
		print("Starting simulation.")
		while self.simulation_state.curr_time <= self.simulation_state.time:
			# Check the control file for updates
			if self.sim is not None:
				self.sim.update()
			# Calculate forces
			self.dynamics()
			# Do kinematics
			self.kinematics()
			# Log the state
			self.log.sample()
			# Update the simulation viewer
			if self.simulation_state.show_viewer:
				self.viewer.update()
			if self.simulation_state.show_plotter:
				self.plotter.update_plots()
			self.simulation_state.curr_time += self.simulation_state.delta
			self.app.processEvents()
			print("Simulated time: {:.0F}s - {:.2%}".format(self.simulation_state.curr_time, self.simulation_state.curr_time/self.simulation_state.time), end='\r', flush=True)
		print("")
		print("Simulation completed.")

	def export_logs(self): 
		file = "log.csv"
		print("Writing log to " + file)
		self.log.export_csv(file)
		print("Finished writing log file.")

	def run(self):
		if self.simulation_state.show_viewer:
			self.viewer = Viewer(self.simulation_state, self.aircraft_state, self.app)
		if self.simulation_state.show_plotter:
			self.plotter = Plotter(self.log, self.app)
		self.load_state()
		self.sim_loop()
		self.export_logs()
		if self.simulation_state.show_viewer or self.simulation_state.show_plotter:
			input("Press enter to quit.")
		'''try:
			# Hoping to catch the gross pyqt errors here
			if self.simulation_state.show_viewer or self.simulation_state.show_plotter:
				self.app.quit()
		except: 
			pass'''

	def kinematics(self):
		# Create a temporary state with quaternions instead of euler coordinates.
		# Also math will be easier with an array than an object
		state = self.aircraft_state
		#e = angles.euler_to_quaternion(state.get_phi(), state.get_theta(), state.get_psi())

		quat_state = np.array([
			state.get_pn(), state.get_pe(), state.get_pd(),
			state.get_u(), state.get_v(), state.get_w(),
			state.get_phi(), state.get_theta(), state.get_psi(),
			state.get_p(), state.get_q(), state.get_r()])
		# RK4
		h = self.simulation_state.delta
		k1 = self.find_dots(quat_state)
		k2 = self.find_dots(quat_state + (h/2.)*k1)
		k3 = self.find_dots(quat_state + (h/2.)*k2)
		k4 = self.find_dots(quat_state + h*k3)
		ksum = (h/6.)*(k1 + 2.*k2 + 2.*k3 + k4)
		next_state = quat_state + ksum#h*self.find_dots(quat_state)#ksum

		# Make sure that it's a unit quaternion. [1] Appendix B
		#next_state[6:10] = angles.normal(*next_state[6:10])

		# Finally, update state
		#phi,theta,psi = angles.quaternion_to_euler(*next_state[6:10])
		self.aircraft_state.pn = next_state[0]
		self.aircraft_state.pe = next_state[1]
		self.aircraft_state.pd = next_state[2]
		self.aircraft_state.u = next_state[3]
		self.aircraft_state.v = next_state[4]
		self.aircraft_state.w = next_state[5]
		self.aircraft_state.phi = next_state[6]
		self.aircraft_state.theta = next_state[7]
		self.aircraft_state.psi = next_state[8]
		self.aircraft_state.p = next_state[9]
		self.aircraft_state.q = next_state[10]
		self.aircraft_state.r = next_state[11]

	def find_dots(self, state): 
		# Find the derivatives of the internal state array from state forces
		# Unpack the state array
		pn,pe,pd,u,v,w,phi,theta,psi,p,q,r = state
		fx = self.aircraft_state.fx
		fy = self.aircraft_state.fy
		fz = self.aircraft_state.fz
		l = self.aircraft_state.l
		m = self.aircraft_state.m
		n = self.aircraft_state.n
		airframe = self.aircraft_state.airframe
		mass = airframe.m
		# Body velocities to inertial translation
		ctheta = math.cos(theta)
		cpsi = math.cos(psi)
		cphi = math.cos(phi)
		stheta = math.sin(theta)
		spsi = math.sin(psi)
		sphi = math.sin(phi)
		ttheta = math.tan(theta)
		sectheta = 1/math.cos(theta)
		body_vel_inertial_pos_matrix = np.array([
			[ctheta*cpsi, sphi*stheta*cpsi - cphi*spsi, cphi*stheta*cpsi + sphi*spsi],
			[ctheta*spsi, sphi*stheta*spsi + cphi*cpsi, cphi*stheta*spsi - sphi*cpsi],
			[-stheta, sphi*ctheta, cphi*ctheta]
		])
		pn_dot,pe_dot,pd_dot = (body_vel_inertial_pos_matrix @ np.array([u,v,w]).T).T

		# Moments to velocities
		u_dot,v_dot,w_dot = np.array([r*v-q*w, p*w-r*u, q*u-p*v]) + np.array([fx,fy,fz])/mass

		# attitude dots
		att_to_att_dot_matrix = np.array([
			[1, sphi*ttheta, cphi*ttheta],
			[0, cphi, -sphi],
			[0, sphi*sectheta, cphi*sectheta]
		])
		phi_dot,theta_dot,psi_dot = (att_to_att_dot_matrix @ np.array([p,q,r]).T).T

		# Attitude accelerations
		p_dot = airframe.gamma1*p*q - airframe.gamma2*q*r + airframe.gamma3*l + airframe.gamma4*n 
		q_dot = airframe.gamma5*p*r - airframe.gamma6*((p**2)-(r**2)) + m*1/airframe.Jy
		r_dot = airframe.gamma7*p*q - airframe.gamma1*q*r + airframe.gamma4*l + airframe.gamma8*n
		state_dot = np.array([
			pn_dot, pe_dot, pd_dot,
			u_dot, v_dot, w_dot,
			phi_dot, theta_dot, psi_dot,
			p_dot, q_dot, r_dot])
		return state_dot

	def dynamics(self):
		pass