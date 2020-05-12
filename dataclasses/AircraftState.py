import dataclasses 

class AircraftState:
	# Inertial Position (meters)
	pn: float = 0    # North
	pe: float = 0    # East
	h: float = 0     # Altitude
	# Attitude (radians[/sec])
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
	# Bias (radians/second)
	bx: float = 0    # Gyro bias roll axis
	by: float = 0    # Gyro bias pitch axis
	bz: float = 0    # Gyro bias yaw axis