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
import traceback
from colored import fg, bg, attr
import warnings

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
		self.load_state()
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
		print("\t- Simulation Time: " + str(self.simulation_state.time))
		print("\t- delta T: " + str(self.simulation_state.delta))
		if self.sim is not None:
			self.sim.start()
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
		self.sim_loop()
		self.export_logs()
		if not self.simulation_state.show_plotter and self.simulation_state.aar:
			self.plotter = Plotter(self.log, self.app)
			self.plotter.update_plots()
			self.app.processEvents()
			print("Opening After Action Report")
		if self.simulation_state.show_viewer or self.simulation_state.show_plotter or self.simulation_state.aar:
			input("Press enter to quit.")
			if self.simulation_state.show_viewer:
				self.viewer.window.close()
			if self.simulation_state.show_plotter or self.simulation_state.aar:
				self.plotter.window.close()
			self.app.quit()
	
	def aerodynamic_forces(self):
		model = self.aircraft_state.airframe
		s_alpha = np.sin(self.aircraft_state.alpha)
		c_alpha = np.cos(self.aircraft_state.alpha)


		# Eq 4.12/4.13 [1] (Simplified model)
		CL = model.CL0 + model.CLa*self.aircraft_state.alpha
		CD = model.CD0 + model.CDa*self.aircraft_state.alpha

		# Eq 4.19 [1]
		CX = -CD*c_alpha + CL*s_alpha
		CXq = -model.CDq*c_alpha + model.CLq*s_alpha
		CXdE = -model.CD_deltaE*c_alpha + model.CL_deltaE*s_alpha
		CZ = -CD*s_alpha - CL*c_alpha
		CZq = -model.CDq*s_alpha - model.CLq*c_alpha
		CZdE = -model.CD_deltaE*s_alpha - model.CL_deltaE*c_alpha

		e_minus = np.exp(-model.M * (self.aircraft_state.alpha - model.alpha0))
		e_plus = np.exp(model.M * (self.aircraft_state.alpha + model.alpha0))
		sigma = (1 + e_minus + e_plus) / ((1 + e_minus) * (1 + e_plus))
		CL = (1 - sigma)*(model.Cl0 + model.CLa*self.aircraft_state.alpha) + sigma*(2*np.sign(self.aircraft_state.alpha)*(np.sin(self.aircraft_state.alpha)**2)*np.cos(self.aircraft_state.alpha))
		CD = model.CDp+((model.CL0 + model.CLa*self.aircraft_state.alpha)**2)/(np.pi*model.e*model.AR)

		# Eq 4.18 [1] 
		air_const = 0.5*model.rho*(self.aircraft_state.va**2)*model.S
		if self.aircraft_state.va == 0: return # No speed, no aero
		spd_const = model.c/(2*self.aircraft_state.va)
		#dFx = air_const*(CX + CXq*model.c/(2*self.aircraft_state.va)*self.aircraft_state.q + CXdE*self.aircraft_state.dE)
		#dFy = air_const*(model.CY0 + model.CYB*self.aircraft_state.beta + model.CYp*model.b/(2*self.aircraft_state.va)*self.aircraft_state.p + model.CYr*model.b/(2*self.aircraft_state.va)*self.aircraft_state.r + model.CY_deltaA*self.aircraft_state.dA + model.CY_deltaR*self.aircraft_state.dR)
		#dFz = air_const*(CZ + CZq*model.c/(2*self.aircraft_state.va)*self.aircraft_state.q + CZdE*self.aircraft_state.dE)
		lift = air_const*(CL + model.CLq*spd_const*self.aircraft_state.q + model.CL_deltaE*self.aircraft_state.dE)
		drag = air_const*(CD + model.CDq*spd_const*self.aircraft_state.q + model.CD_deltaE*self.aircraft_state.dE)
		m = air_const*model.c*(model.Cm0 + model.Cma*self.aircraft_state.alpha + model.Cmq*spd_const*self.aircraft_state.q + model.Cm_deltaE*self.aircraft_state.dE)
		dFy = air_const*(model.CY0 + model.CYB*self.aircraft_state.beta + model.CYp*model.b/(2*self.aircraft_state.va)*self.aircraft_state.p + model.CYr*model.b/(2*self.aircraft_state.va)*self.aircraft_state.r + model.CY_deltaA*self.aircraft_state.dA + model.CY_deltaR*self.aircraft_state.dR)
		dFx = -drag*c_alpha + lift*s_alpha
		dFz = -drag*s_alpha - lift*c_alpha
		self.aircraft_state.fx += dFx
		self.aircraft_state.fy += dFy
		self.aircraft_state.fz -= dFz
	
	def aerodynamic_moments(self):
		# Eq 4.20 [1] 
		model = self.aircraft_state.airframe
		if self.aircraft_state.va == 0: return # No speed, no aero
		air_const = 0.5*model.rho*(self.aircraft_state.va**2)*model.S
		self.aircraft_state.l += air_const*model.b*(model.Cl0 + model.ClB*self.aircraft_state.beta + model.Clp*model.b*self.aircraft_state.p/(2*self.aircraft_state.va) + model.Clr*model.b*self.aircraft_state.r/(2*self.aircraft_state.va) + model.Cl_deltaA*self.aircraft_state.dA + model.Cl_deltaR*self.aircraft_state.dR)
		self.aircraft_state.m -= air_const*model.c*(model.Cm0 + model.Cma*self.aircraft_state.alpha + model.Cmq*model.c*self.aircraft_state.q/(2*self.aircraft_state.va) + model.Cm_deltaE*self.aircraft_state.dE)
		self.aircraft_state.n += air_const*model.b*(model.Cn0 + model.CnB*self.aircraft_state.beta + model.Cnp*model.b*self.aircraft_state.p/(2*self.aircraft_state.va) + model.Cnr*model.b*self.aircraft_state.r/(2*self.aircraft_state.va) + model.Cn_deltaA*self.aircraft_state.dA + model.Cn_deltaR*self.aircraft_state.dR)
	
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
		k1 = self.find_derivatives(quat_state)
		k2 = self.find_derivatives(quat_state + (h/2.)*k1)
		k3 = self.find_derivatives(quat_state + (h/2.)*k2)
		k4 = self.find_derivatives(quat_state + h*k3)
		ksum = (h/6.)*(k1 + 2.*k2 + 2.*k3 + k4)
		next_state = quat_state + ksum
		# Euler for debugging
		x = h*self.find_derivatives(quat_state)
		#next_state = quat_state + x

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

	def find_derivatives(self, state): 
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
		self.zero_dynamics()
		self.gravity()
		self.aerodynamic_forces()
		self.aerodynamic_moments()
		self.motor_forces()

	def motor_forces(self):
		model = self.aircraft_state.airframe
		self.aircraft_state.fx += 0.5*model.rho*model.Sprop*model.Cprop*(((model.kmotor*self.aircraft_state.dT)**2) - (self.aircraft_state.va**2))

	def zero_dynamics(self):
		self.aircraft_state.fx = 0
		self.aircraft_state.fy = 0
		self.aircraft_state.fz = 0
		self.aircraft_state.l = 0
		self.aircraft_state.m = 0
		self.aircraft_state.n = 0

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
		self.aircraft_state.va = np.linalg.norm([self.aircraft_state.ur, self.aircraft_state.vr, self.aircraft_state.wr])#np.sqrt(self.aircraft_state.ur**2 + self.aircraft_state.vr**2 + self.aircraft_state.wr**2)
		self.aircraft_state.alpha = -np.arctan2(self.aircraft_state.wr, self.aircraft_state.ur)
		if self.aircraft_state.va != 0 and not np.isnan(self.aircraft_state.vr/self.aircraft_state.va):
			self.aircraft_state.beta = np.arcsin(self.aircraft_state.vr/self.aircraft_state.va)
		else:
			self.aircraft_state.beta = 0

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