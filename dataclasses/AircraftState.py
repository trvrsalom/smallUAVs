import dataclasses
import math

class AircraftState:
	# Inertial Position (meters)
	n: float = 0     # North
	e: float = 0     # East
	d: float = 0     # Altitude
	# Attitude (ang_units[/sec])
	phi: float = 0   # Roll
	theta: float = 0 # Pitch
	psi: float = 0   # Yaw
	alpha: float = 0 # Angle of Attack
	beta: float = 0  # Slideslip Angle
	p: float = 0     # Roll rate
	q: float = 0     # pitch rate
	r: float = 0     # Yaw rate
	gamma: float = 0 # Flight path
	chi: float = 0   # Course angle
	# Speed (meters/second)
	vg: float = 0    # Groundspeed
	va: float = 0    # Airspeed
	wn: float = 0    # Inertial windspeed north
	we: float = 0    # Inertial windspeed east
	# Bias (ang_units/second)
	bx: float = 0    # Gyro bias roll axis
	by: float = 0    # Gyro bias pitch axis
	bz: float = 0    # Gyro bias yaw axis
	# Units
	ang_units: str = "deg"

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

	def get_n(self):
		return self.n

	def get_e(self):
		return self.e

	def get_d(self):
		return self.d

