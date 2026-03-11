import numpy as np
np.set_printoptions(linewidth=np.inf)

r_11 = 12
r_12 = 6.15
r_13 = 6.15
r_14 = 4.15
r_15 = 4.15
r_21 = 9
r_22 = 9
r_23 = 6.1
r_24 = 6.1
r_31 = 9
r_32 = 9
r_33 = 6.1
r_34 = 6.1
r_41 = 9
r_42 = 9
l1 = 21
l2 = 44
l3 = 29
l4 = 32

R_A = np.array([5.5, 5.5, 5.5, 5.5, 5.5, 5.5])
S = np.array([[-r_11, 0, 0, 0],
              [r_11, 0, 0, 0],
              [r_12, -r_21, 0, 0],
              [-r_13, r_22, 0, 0],
              [r_14, r_23, -r_31, -r_33*(r_33/r_41)],
              [-r_15, -r_24, r_32, r_34*(r_34/r_42)]]).T

# J = np.array([[0, 0, 0, 0],
#               [0, 0, 0, 0],
#               [0, 0, 0, 0],
#               [],
#               [],
#               []])

print("")
print("R_A=")
print(R_A)
print("")
print("S=")
print(S)
print("")