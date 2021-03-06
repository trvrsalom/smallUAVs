import dataclasses

class Zagi:
	m: float = 1.56 #kg
	Jx: float = 0.1147 #kg m^2
	Jy: float = 0.0576 #kg m^2
	Jz: float = 0.1712 #kg m^2
	Jxz: float = 0.0015 #kg m^2
	S: float = 0.2589 #m^2
	b: float = 1.4224 #m
	c: float = 0.3302 #m
	Sprop: float = 0.0314 #m^2
	rho: float = 1.2682 #kg/m^3
	kmotor: float = 20
	kTp: float = 0
	kOmega: float = 0
	e: float = 0.9
	CL0: float = 0.09167
	CD0: float = 0.01631
	Cm0: float = -0.02338
	CLa: float = 3.5016
	CDa: float = 0.2108
	Cma: float = -0.5675
	CLq: float = 2.8932
	CDq: float = 0
	Cmq: float = -1.3990
	CL_deltaE: float = 0.2724
	CD_deltaE: float = 0.3045
	Cm_deltaE: float = -0.3254
	Cprop: float = 1.0
	M: float = 50
	alpha0: float = 0.4712
	epsilon: float = 0.1592
	CDp: float = 0.0254
	CY0: float = 0
	Cl0: float = 0
	Cn0: float = 0
	CYB: float = -0.07359
	ClB: float = -0.02854
	CnB: float = -0.00040
	CYp: float = 0
	Clp: float = -0.3209
	Cnp: float = -0.01297
	CYr: float = 0
	Clr: float = 0.03066
	Cnr: float = -0.00434
	CY_deltaA: float = 0
	Cl_deltaA: float = 0.1682
	Cn_deltaA: float = -0.00328
	CY_deltaR: float = 0
	Cl_deltaR: float = 0
	Cn_deltaR: float = 0
	# Gamma values
	gamma: float = Jx*Jz - Jxz*Jxz
	gamma1: float = (Jx*(Jx-Jy+Jz))/gamma
	gamma2: float = ((Jz*(Jz-Jy))+Jxz*Jxz)/gamma
	gamma3: float = Jz/gamma
	gamma4: float = Jxz/gamma
	gamma5: float = (Jz-Jx)/Jy
	gamma6: float = Jxz/Jy
	gamma7: float = ((Jx-Jy)*Jx+Jxz*Jxz)/gamma
	gamma8: float = Jx/gamma
	AR: float = (b**2)/S