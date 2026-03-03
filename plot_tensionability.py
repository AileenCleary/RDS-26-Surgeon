import numpy as np

import matplotlib
matplotlib.use('TkAgg')  
import matplotlib.pyplot as plt

from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from route_configs import CONFIG_LIST

def check_origin(S):
    S = np.asarray(S, dtype=float)
    m, n = S.shape

    c = np.zeros(n)  
    A_eq = np.vstack([S, np.ones((1, n))]) # sum(T)=1 ; normalization
    b_eq = np.array([0.0, 0.0, 0.0, 1.0])
    bounds = [(0, None)] * n

    res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method="highs")
    return res.success, (res.x if res.success else None)

def plot_convex_hull(S, inside, color):
    P = S.T
    hull = ConvexHull(P)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    faces = [P[s] for s in hull.simplices]
    poly = Poly3DCollection(faces, alpha=0.25)
    poly.set_edgecolor(color)
    poly.set_facecolor(color)
    ax.add_collection3d(poly)

    ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=35)
    ax.scatter([0], [0], [0], marker="x", s=90)

    ax.set_xlabel("tau_1")
    ax.set_ylabel("tau_2")
    ax.set_zlabel("tau_3")
    ax.set_title("INSIDE (tensionable)" if inside else "OUTSIDE (not tensionable)")

    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=20, azim=35)

    plt.tight_layout()
    plt.show()

#COLORS = [(255, 0, 0, 0.8),(16, 255, 0, 0.8),(0, 143, 255, 0.8),(112, 0, 255, 0.8)]
COLORS = ['b','g','r','c']

def tensionability(r_config, color_idx):
    print(f"CONFIG NAME: {r_config.name}\n")
    print(f"{r_config.desc}\n")
    print(f"S4 = \n{r_config.S4}\n")
    print(f"D4 = \n{r_config.D4}\n")
    print(f"S = \n{r_config.S}\n")
    S = r_config.S
    face_color = COLORS[color_idx]

    inside, T = check_origin(S)
    print("Origin inside convex hull of columns?", inside)
    if inside:
        print("Example normalized tension weights T (>=0, sum=1) that give zero torque:")
        print(T)
        print("S @ T =", S @ T, "  sum(T) =", T.sum())

    plot_convex_hull(S, inside, face_color)

if __name__ == "__main__":
    for idx, config_option in enumerate(CONFIG_LIST):
        tensionability(config_option, idx)

