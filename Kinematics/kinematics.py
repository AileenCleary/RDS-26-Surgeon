import numpy as np
import matplotlib.pyplot as plt
import sympy as sym

# To Do
# 1. Map out tendon routing on diagram.
# 2. Check signs on matrix D.

# Link Lengths (mm)
L0 = 20; L1 = 40; L2 = 25; L3 = 20

# Joint Angles (Radians)
th0 = np.deg2rad(0)
th1 = np.deg2rad(15)
th2 = np.deg2rad(30)

# Pulley Radii (mm) [r_<shaft#><pulley#>]
r0 = 10 # splay
r11 = 10; r12 = 10; r13 = 10
r21 = 10; r22 = 10; r23 = 10; r24 = 10
r31 = 10
r41 = 10; r42 = 10; r43 = 10
r51 = 10
r61 = 10; r62 = 10

# Shaft Distances (mm) (?) - Not Used
L_s1 = 1; L_s2 = 1; L_s3 = 1
L_s4 = 1; L_s5 = 1; L_s6 = 1

# Bearing Location (mm) B<shaft#><Left or Right> (?) - Not Used
B1L = 1; B1R = 1; B2L = 1; B2R = 1; B3L = 1; B3R = 1
B4L = 1; B4R = 1; B5L = 1; B5R = 1; B6L = 1; B6R = 1

# Tendon Position On Shaft (?) - Not Used
# *** 1. What does "tendon position on shaft" mean?
L_t1 = 1; L_t2 = 1; L_t3 = 1

# Motor Parameters
P = 20 # Watts
N = 240.667 # RPM (max motor speed from selection)
eff = 0.85

# Safety Factor
SF = 2

# Low Carbon Steel Material Properties
# (https://www.mcmaster.com/products/shafts/diameter~4-mm/steel-2~/)
Sy = 413.68544 # MPa (60000 psi)
Sut = 585 # MPa
t_allow = .577 * Sy # Allowing Shear Torsion
T = 2.2598 # Motor Selection (Nm)

# Tendon Tension
""" ~Estimated 'actuator-side' available tension. Assumes each tendon has
its own actuator delivering full motor torque T, no gearing, no effect from
friction/wrap angle/etc, and pulley radius is the effective moment arm. """
# *** 2. How does tendon number map to tendon? L -> R ?
# *** 3. How was the pulley chosen?
T1 = (T * eff)/r21
T2 = (T * eff)/r21
T3 = (T * eff)/r21
T4 = (T * eff)/r21
T5 = (T * eff)/r11

# Structure Matrix
""" Rows correspond to joints [MCP-s2,PIP-s4,DIP-s6].
Columns correspond to tendon routing. """
# *** 4. So it doesn't need/include idler pulley config.?
R = np.array([[r21, r22, 0, r23, r24],
      [0, r41, r43, r42, 0],
      [0, r61, r62, 0, 0]])
# So T1->T5 has internal coupling tendon as 2nd col (0th-4th).

# Sign/Incidence Matrix
""" Sign/incidence matrix for which tendon contributes pos/neg torque
to each joint. +1 for routed over, -1 for under, 0 for no interaction. """
D = np.array([[1, 1, 0, -1, -1],
              [0, 1, -1, -1, 0],
              [0, 1, -1, 0, 0]])

# NICK
""" Structure matrix is R.T ((3x5)->(5x3)) @ D (3x5) => (5x5) """
S = R.T @ D
Tt_array = np.array([T1,T2,T3,T4,T5]) # Tendon tension array
JT_array = S @ Tt_array # Joint Torque array
""" The Joint Torque array size is a 5 vector.
ISSUE? : Joint torques should be a 3 vector, right? (MCP, PIP, DIP)"""

# AILEEN
""" Define moment arm matrix as A=D*R (element-wise mult). """
A = D * R
""" The joint torque vector then becomes tau=A@T (1x3, 3-vec). """
tau = np.dot(A, Tt_array) # Equivalent to A @ Tt_array

# Denavit-Hartenberg (DH) Method
# NICK - ISSUE: Used the variable a (link length) in place of alpha (twist angle).
def DH(theta, d, a, alpha):
    return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                     [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                     [0, np.sin(alpha), np.cos(alpha), d],
                     [0, 0, 0, 1]])

T_01 = DH(th0, d=0, a=0, alpha=np.pi/2)
T_12 = DH(th1, d=0, a=L1, alpha=0)
T_23 = DH(th2, d=0, a=L2, alpha=0)
T_34 = DH(0, d=0, a=L3, alpha=0)
T_04 = T_01 @ T_12 @ T_23 @ T_34
tip_pos = T_04[0:3, 3]
tip_orientation_matrix = T_04[0:3, 0:3]

# Pulley Force
# NICK - ISSUE: Incorrect Python indexing (["T"]) and incorrect sign in equation (+ -> -).
def pulley_force(p):
    """ stuff about wrap angle/phi calc"""
    T = p["T"]
    th = p["phi"]
    F = np.sqrt(2) * np.sqrt(T**2 * (1 - np.cos(th)))
    Fx = F * np.cos(p["phi"])
    Fy = F * np.sin(p["phi"])
    return np.array([Fx,Fy])

