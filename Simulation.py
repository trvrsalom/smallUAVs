import sys
from Viewer import Viewer
from importlib import import_module
from dataclasses.SimulationState import SimulationState
from dataclasses.AircraftState import AircraftState
from dataclasses.Zagi import Zagi
from utils import angles
import numpy as np
import math
import Simulation

class Simulation:
	def __init__(self, simfile=None):
		self.simulation_state = SimulationState()
		self.aircraft_state = AircraftState()
		self.aircraft_state.set_airframe(Zagi())
		if simfile is not None:
			try: 
				print("Loading simulation file: " + simfile)
				globals()["sim"] = import_module(simfile)
				self.sim = sim.Sim(self.simulation_state, self.aircraft_state)
			except Exception as e:
				print("Failed to load simulation file: " + str(e))
				self.sim = None
		else:
			print("No simulation file provided")
			self.sim = None
		if self.simulation_state.show_viewer:
			self.viewer = Viewer(self.simulation_state, self.aircraft_state)

	def load_state(self):
		if self.sim is not None:
			if hasattr(self.sim, "time"):
				self.simulation_state.time = self.sim.time
			if hasattr(self.sim, "delta"):
				self.simulation_state.delta = self.sim.delta

	def sim_loop(self):
		while self.simulation_state.curr_time <= self.simulation_state.time:
			# Check the control file for updates
			if self.sim is not None:
				self.sim.update()
			# Calculate forces
			self.dynamics()
			# Do kinematics
			self.kinematics()
			# Update the simulation viewer
			if(self.simulation_state.show_viewer):
				self.viewer.update()
			self.simulation_state.curr_time += self.simulation_state.delta
		print("Simulation completed")

	def start(self):
		self.load_state()
		self.sim_loop()

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
		k1 = h*self.find_dots(quat_state)
		k2 = h*self.find_dots(quat_state + (h/2.)*k1)
		k3 = h*self.find_dots(quat_state + (h/2.)*k2)
		k4 = h*self.find_dots(quat_state + k3)
		ksum = (h/6.)*(k1 + 2.*k2 + 2.*k3 + k4)
		next_state = quat_state + ksum

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
		attitude_accel_a = np.array([airframe.gamma1*p*q - airframe.gamma2*q*r, 
		                             airframe.gamma5*p*r - airframe.gamma6*(p**2-r**2),
		                             airframe.gamma7*p*q - airframe.gamma1*q*r])
		attitude_accel_b = np.array([airframe.gamma3*l + airframe.gamma4*n,m/airframe.Jy, airframe.gamma4*l + airframe.gamma8*n])
		p_dot,q_dot,r_dot = attitude_accel_a + attitude_accel_b
		state_dot = np.array([
			pn_dot, pe_dot, pd_dot,
			u_dot, v_dot, w_dot,
			phi_dot, theta_dot, psi_dot,
			p_dot, q_dot, r_dot])
		return state_dot

	def dynamics(self):
		pass