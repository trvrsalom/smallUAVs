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
		log.add_dataset("Inertial Position (m)", ["pn", "pe", "-pd"], [self.aircraft_state.get_pn, self.aircraft_state.get_pe, lambda: -1*self.aircraft_state.get_pd()])
		log.add_dataset("Attitude (rad)", ["ϕ", "ϴ", "ψ"], [self.aircraft_state.get_phi, self.aircraft_state.get_theta, self.aircraft_state.get_psi])
		log.add_dataset("Attitude Rates (rad/s)", ["p","q","r"], [self.aircraft_state.get_p, self.aircraft_state.get_q, self.aircraft_state.get_r])
		log.add_dataset("Body Velocity (m/s)", ["u", "v", "w"], [self.aircraft_state.get_u, self.aircraft_state.get_v, self.aircraft_state.get_w])
		log.add_dataset("Airspeed (m/s)", ["ur", "vr", "wr"], [self.aircraft_state.get_ur, self.aircraft_state.get_vr, self.aircraft_state.get_wr])
		log.add_dataset("Body Frame Forces (N)", ["fx", "fy", "fz"], [self.aircraft_state.get_fx, self.aircraft_state.get_fy, self.aircraft_state.get_fz])
		log.add_dataset("Body Moments (Nm)", ["l", "m", "n"], [self.aircraft_state.get_l, self.aircraft_state.get_m, self.aircraft_state.get_n])
		log.add_dataset("Wind (m/s)", ["wn", "we", "wd"], [self.simulation_state.get_wn, self.simulation_state.get_we, self.simulation_state.get_wd])
		log.add_dataset("Wind Triangle (m/s)", ["α", "β"], [self.aircraft_state.get_alpha, self.aircraft_state.get_beta])
		
		self.log = log

	def sim_loop(self):
		print("Starting simulation.")
		while self.simulation_state.curr_time <= self.simulation_state.time:
			# Check the control file for updates
			if self.sim is not None:
				self.sim.update()
			# Update stability frame
			self.update_stability_frame()
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
		#self.export_logs()
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
		# TODO: Try quaternions again another time
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
		self.aircraft_state.fx = 0
		self.aircraft_state.fy = 0
		self.aircraft_state.fz = 0
		self.gravity()

	def gravity(self):
		m = self.aircraft_state.airframe.m
		g = self.simulation_state.g
		stheta = math.sin(self.aircraft_state.get_theta())
		ctheta = math.cos(self.aircraft_state.get_theta())
		sphi = math.sin(self.aircraft_state.get_phi())
		cphi = math.cos(self.aircraft_state.get_phi())
		self.aircraft_state.fx += -m*g*stheta
		self.aircraft_state.fy += m*g*ctheta*sphi
		self.aircraft_state.fz += m*g*ctheta*cphi

	def update_stability_frame(self):
		relative_wind = self.inertial_to_body().T @ np.array([self.simulation_state.wn, self.simulation_state.we, self.simulation_state.wd]).T
		self.aircraft_state.ur = self.aircraft_state.u - relative_wind[0]
		self.aircraft_state.vr = self.aircraft_state.v - relative_wind[1]
		self.aircraft_state.wr = self.aircraft_state.w - relative_wind[2]
		self.aircraft_state.alpha = -np.arctan2(self.aircraft_state.wr, self.aircraft_state.ur)
		self.aircraft_state.beta = np.arcsin(self.aircraft_state.vr/np.sqrt(self.aircraft_state.vr**2 + self.aircraft_state.wr**2 + self.aircraft_state.ur**2))

	def vehicle_to_vehicle_one(self):
		cp = np.cos(self.aircraft_state.get_psi())
		sp = np.sin(self.aircraft_state.get_psi())
		R = np.array([
			[cp, -sp, 0],
			[sp,  cp, 0],
			[ 0,   0, 1]
		])
		return R.T

	def vehicle_one_to_vehicle_two(self):
		ct = np.cos(self.aircraft_state.get_theta())
		st = np.sin(self.aircraft_state.get_theta())
		R = np.array([
			[ ct, 0, st],
			[  0, 1,  0],
			[-st, 0, ct]
		])
		return R.T

	def vehicle_two_to_body(self):
		cp = np.cos(self.aircraft_state.get_phi())
		sp = np.sin(self.aircraft_state.get_phi())
		R = np.array([
			[1,  0, 0],
			[0, cp,sp],
			[0,-sp,cp]
		])
		return R.T

	def inertial_to_body(self):
		R = self.vehicle_to_vehicle_one() @ self.vehicle_one_to_vehicle_two() @ self.vehicle_two_to_body()
		return R