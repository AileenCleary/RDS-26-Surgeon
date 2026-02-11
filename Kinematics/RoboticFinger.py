import numpy as np
import matplotlib.pyplot as plt
import sympy as sym

class Motor:
    """ Motor
    Pmax : Maximum power (Watts).
    Vmax : Maximum velocity (RPM).
    eff : Motor efficiency.
    T : Torque (Nm).
    """
    def __init__(self, max_power, max_vel, eff, torque):
        self.Pmax = max_power
        self.Vmax = max_vel
        self.eff = eff
        self.T = torque

class Material:
    """ Material
    Sy : Yield strength (MPA).
    Sut : Ultimate tensile strength? (MPA).
    T_allow : Allowable shear torsion.
    """
    def __init__(self, sy, sut):
        self.Sy = sy
        self.Sut = sut
        self.T_allow = 0.577 * sy

class Shaft:
    def __init__(self, material, diameter, length):
        self.Material = material
        self.D = diameter
        self.L = length

class Pulley:
    def __init__(self, radius, s_num, p_num, wrap_angle=None): # Need to add wrap angle equation.
        self.R = radius
        self.ShaftID = s_num
        self.PulleyID = p_num
        self.Phi = wrap_angle

class Pullies:
    def __init__(self, pulleys):
        self._p = {(p.ShaftID, p.PulleyID): p for p in pulleys}

    def r(self, shaft, pid):
        return self._p[(shaft,pid)].R
    
    def phi(self, shaft, pid):
        return self._p[(shaft,pid)].Phi
    
class RoboticFinger:
    """ Robotic Finger 
    L_L : 4-vector of link lengths (mm).
    Th_J : 3-vector of joint angles (SPLAY, MCP, PIP) (rads).
    Motor : Motor object with parameters.
    SF : Safety factor.
    T_T : 5-vector of estimated tension in tendons.
    P : Collected pulleys for given configuration.
    R : Tendon route matrix.
    D : Tendon sign/incidence matrix. +1 for CCW, -1 for CW, 0 for no interaction.
    A : Moment arm matrix.
    Tau_J : 3-vector of joint torques. 3rd torque is tau_PIP generalized, i.e., accounting for coupling with DIP.
    """
    def __init__(self, link_lengths, joint_angles, motor, sf, pulleys, tendon_sign, fingertip_pos):
        self.L_L = np.array(link_lengths)
        self._Th_J = np.array(joint_angles)

        self.Motor = motor
        self.SF = sf
        self.P = pulleys
        self.D = np.array(tendon_sign)

        self.fingertip_pos = float(fingertip_pos)

        self.coupling_ratio = self.P.r(4,2)/self.P.r(6,2)

    @property
    def Th_J(self):
        return self._Th_J
    
    @Th_J.setter
    def Th_J(self, joint_angles):
        self._Th_J = np.array(joint_angles)
        self.update_state()

    def update_state(self):
        self.R = self.tendon_route()
        self.A = self.R * self.D
        tau_ref = np.array([0.0, 0.0, 0.0])
        self.T_T = self.solve_tendon_tensions(tau_ref, 5.0)
        self.Tau_J = self.A@self.T_T

        self.Tmatrix = self.transformation_matrix(self.fingertip_pos)
        self.T_0F = self.Tmatrix[-1]
        self.tip_pos = self.T_0F[0:3, 3]
        self.tip_orientation_matrix = self.T_0F[0:3, 0:3]


    def estimate_tendon_tension(self):
        """ Estimated 'actuator-side' available tension. Assumes each tendon has its own
        actuator with full motor torque T, no gearing, and no friction/wrap angle effects.
        Pulley radius is the effective moment arm and is assumed constant."""
        T0 = self.Motor.eff * self.Motor.T / self.P.r(0,1)
        return np.array([T0] * 6)
    
    def tendon_route_full(self):
        R = np.array([[self.P.r(0,1), 0,            0,             0,              0,                0],
                      [0,             self.P.r(2,1),self.P.r(2,2), 0,              self.P.r(2,3),    self.P.r(2,4)],
                      [0,             0,            self.P.r(4,1), self.P.r(4,2),  self.P.r(4,3),    0],
                      [0,             0,            self.P.r(6,1), self.P.r(6,2),  0,                0]])
        return R
    
    def tendon_route_gen(self, R4):
        R3 = np.zeros((3, R4.shape[1]))
        R3[0, :] = R4[0, :]
        R3[1, :] = R4[1, :]
        R3[2, :] = R4[2, :] + self.coupling_ratio*R4[3, :]
        print(R3)
        return R3

    def tendon_route(self):
        R4 = self.tendon_route_full()
        R3 = self.tendon_route_gen(R4)
        return R3
    
    def solve_tendon_tensions(self, tau_ref, preload):
        A = self.A
        m = A.shape[1]
        AAT_inv = np.linalg.inv(A@A.T)
        T_particular = A.T@(AAT_inv@tau_ref)
        A_pinv = A.T@AAT_inv
        H=np.eye(m)-A_pinv@A
        f0 = np.zeros(m)
        f0[3] = preload
        T = T_particular+H@f0
        T[T<0]=0
        return T
    
    def within_limits(self, x, min, max):
        return x <= max and x >= min
        
    def joint_torque(self):
        tau = self.A @ self.T_T
        return tau
    
    def DH(self, theta, d, a, alpha):
        return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                     [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                     [0, np.sin(alpha), np.cos(alpha), d],
                     [0, 0, 0, 1]])
    
    def transformation_matrix(self, fingertip_pos):
        T_01 = self.DH(self.Th_J[0], d=0, a=self.L_L[0], alpha=np.pi/2)
        T_12 = self.DH(self.Th_J[1], d=0, a=self.L_L[1], alpha=0)
        T_23 = self.DH(self.Th_J[2], d=0, a=self.L_L[2], alpha=0)
        T_34 = self.DH(self.Th_J[2]*self.coupling_ratio, d=0, a=self.L_L[3], alpha=0)
        T_4F = self.DH(0, d=0, a=fingertip_pos, alpha=0)
        return [T_01, T_12, T_23, T_34, T_4F, T_01 @ T_12 @ T_23 @ T_34 @ T_4F]
    
    def forward_kinematics(self, joint_angles=None):
        if joint_angles is not None:
            self.Th_J = joint_angles
        return self.tip_pos
    
    def inverse_kinematics(self, pos):
        x, y, z = pos

        th0 = float(np.arctan2(y, x))
        if not self.within_limits(th0, np.deg2rad(-15), np.deg2rad(15)):
            print("Error: Exceeded joint limit of joint0.")
            return None
        
        T_01 = self.DH(th0, d=0, a=self.L_L[0], alpha=np.pi/2)
        T_10 = np.linalg.inv(T_01)
        pos_H = np.array([x, y, z, 1])
        pos_10 = T_10 @ pos_H

        th_mcp, th_pip= sym.symbols('th_mcp, th_pip', real=True)
        l1 = self.L_L[0]; l2 = self.L_L[1]; l3 = self.L_L[2]; x_d = pos_10[0]; y_d = pos_10[1]

        eq1 = sym.Eq(4 * l1 * l3 * (sym.cos(th_pip)) * (sym.cos(th_pip)) + 2 * (l1 * l2 + l2 * l3) * sym.cos(th_pip) - (x_d**2 + y_d**2 - l1**2 - l2**2 - l3**2 + 2 * l1 * l3), 0)
        lhs = sym.atan2(sym.Float(y_d), sym.Float(x_d)) - th_mcp
        num = (l2 * sym.sin(th_pip) + l3 * sym.sin(2 * th_pip))
        dem = (l1 + l2 * sym.cos(th_pip) + l3 * sym.cos(2 * th_pip))
        rhs = sym.atan2(num, dem)
        eq2 = sym.Eq(lhs,rhs)
        sol1= sym.solve(eq1, th_pip, real=True)
        sol1 = [(a+np.pi)%(2*np.pi)-np.pi for a in sol1 if self.within_limits(a, 0, np.deg2rad(110))]

        sol2 = []
        solutions = []
        for sol in sol1:
            eq2 = eq2.subs(th_pip, sol)
            a = sym.solve(eq2, th_mcp, real=True)
            for x in a:
                if self.within_limits(x, 0, np.deg2rad(90)):
                    sol2.append((x+np.pi)%(2*np.pi)-np.pi)
                    solutions.append([th0, x, sol, sol*self.coupling_ratio])

        return solutions
    
    def pulley_force(self, i, j, T):
        phi = self.P.phi(i,j)
        F = np.sqrt(2) * np.sqrt(T**2 * (1 - np.cos(phi)))
        Fx = F * np.cos(phi)
        Fy = F * np.sin(phi)
        return np.array([Fx,Fy])
        
    
def main():
    link_lengths = [30, 30, 30, 30]
    joint_angles = [np.deg2rad(x) for x in [0, 0, 0]]


    motor = Motor(20, 240.667, 0.85, 2.2598)
    sf = 2
    low_carbon_steel = Material(413.68544, 585)
    
    pulley_group = [[0, 1], [1, 1], [1, 2], [1, 3], [2, 1], [2, 2], [2, 3], \
                    [2, 4], [3, 1], [4, 1], [4, 2], [4, 3], [5, 1], [6, 1], [6, 2]]
    pulley_group = [Pulley(10, x[0], x[1]) for x in pulley_group]
    pullies = Pullies(pulley_group)

    tendon_sign = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 1, 0, 1, -1],
                            [0, 0, 1, 1, -1, 0]])
                            # [0, 0, 1, -1, 0, 0]])
    
    test = RoboticFinger(link_lengths, joint_angles, motor, sf, pullies, tendon_sign, 10)
    
    print(test.forward_kinematics([np.deg2rad(x) for x in [2, 30, 5]]))

if __name__ == "__main__":
    main()