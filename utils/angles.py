import math
import numpy as np

def quaternion_to_euler(e0, e1, e2, e3):
	phi = math.atan2(2*(e0*e1 + e2*e3), e0**2 + e3**2 - e1**2 - e2**2)
	theta = math.asin(2*(e0*e2-e1*e3))
	psi = math.atan2(2*(e0*e3+e1*e3), e0**2 + e1**2 - e2**2 - e3**2)
	return phi,theta,psi

def euler_to_quaternion(phi,theta,psi):
	cpsi = math.cos(psi/2)
	ctheta = math.cos(theta/2)
	cphi = math.cos(theta/2)
	spsi = math.sin(psi/2)
	stheta = math.sin(theta/2)
	sphi = math.sin(phi/2)
	e0 = cpsi*ctheta*cphi + spsi*stheta*sphi
	e1 = cpsi*ctheta*sphi - spsi*stheta*cphi
	e2 = cpsi*stheta*cphi + spsi*ctheta*sphi
	e3 = spsi*ctheta*cphi - cpsi*stheta*sphi
	return e0,e1,e2,e3

def normal(e0,e1,e2,e3):
	mag = np.sqrt(e0**2 + e1**2 + e2**2 + e3**2)
	e1 /= mag
	e2 /= mag
	e3 /= mag
	return e0,e1,e2,e3