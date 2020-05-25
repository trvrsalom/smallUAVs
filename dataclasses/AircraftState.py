import dataclasses
import math

class AircraftState:
	# Inertial Position (meters)
	pn: float = 0    # North
	pe: float = 0    # East
	pd: float = -10    # Altitude
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
	# Airspeed (m/s)
	ur: float = 0    # i airspeed (forward)
	vr: float = 0    # j airspeed (right)
	wr: float = 0    # k airspeed (down)
	# Wind triangle (m/s)
	alpha: float = 0 # angle of attack
	beta: float = 0  # sideslip
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

	def get_asl(self):
		return -self.pd

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

	def get_fx(self):
		return self.fx

	def get_fy(self):
		return self.fy

	def get_fz(self): 
		return self.fz

	def get_l(self):
		return self.l

	def get_m(self):
		return self.m

	def get_n(self):
		return self.n

	def set_airframe(self, airframe):
		self.airframe = airframe

	def get_ur(self):
		return self.ur

	def get_vr(self):
		return self.vr

	def get_wr(self):
		return self.wr

	def get_alpha(self):
		return self.alpha

	def get_beta(self):
		return self.beta