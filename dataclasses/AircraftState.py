import dataclasses
import math

class AircraftState:
	# Inertial Position (meters)
	pn: float = 0    # North
	pe: float = 0    # East
	pd: float = 0    # Altitude
	# Attitude (ang_units)
	phi: float = 0   # Roll
	theta: float = 0 # Pitch
	psi: float = 0   # Yaw
	# Attitude rates (ang_unts/second)
	p: float = 0     # Roll rate
	q: float = 0     # pitch rate
	r: float = 0     # Yaw rate
	# Body frame velocity (meters/second)
	u: float = 0     # i velocity (forward)
	v: float = 0     # j velocity (right)
	w: float = 0     # k velocity (down) 
	# Body frame forces (N)
	fx: float = 0    # X force
	fy: float = 0    # Y force
	fz: float = 0    # Z force
	# Body moments (Nm)
	l: float = 0     # Moment about i 
	m: float = 0     # Moment about j
	n: float = 0     # Moment about k
	# Units
	ang_units: str = "rad"


	# Defining getters so that we can switch units on the fly.
	# Internal simulation/visualization units will be np defaults.
	# Will also need setters, which I forgot todo in advance, so that'll suck
	def get_phi(self):
		if(self.ang_units == "deg"):
			return math.radians(self.phi)
		return self.phi

	def get_theta(self):
		if(self.ang_units == "deg"):
			return math.radians(self.theta)
		return self.theta

	def get_psi(self):
		if(self.ang_units == "deg"):
			return math.radians(self.psi)
		return self.psi

	def get_pn(self):
		return self.pn

	def get_pe(self):
		return self.pe

	def get_pd(self):
		return self.pd

	def get_u(self):
		return self.u

	def get_v(self):
		return self.v

	def get_w(self):
		return self.w

	def get_p(self):
		return self.p

	def get_q(self):
		return self.q

	def get_r(self):
		return self.r

	def set_airframe(self, airframe):
		self.airframe = airframe

