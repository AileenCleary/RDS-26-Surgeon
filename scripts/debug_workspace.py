import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, Simulator
)

def map_workspace():
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    parser.package_map().Add("finger_assembly", "../models")
    parser.AddModels("../models/urdf/finger_sliding.sdf")
    plant.Finalize()

    ctx = plant.CreateDefaultContext()
    pencil_link = plant.GetBodyByName("pencil")
    pencil_tip_local = [0, 0, 0.04] # Tip of the sphere
    world_frame = plant.world_frame()

    # Joint limits (approximate based on your SDF)
    slider_range = np.linspace(-0.1, 0.1, 5)
    mcp_range = np.linspace(0, 1.57, 10)
    pip_range = np.linspace(0, 1.57, 10)
    
    reachable_x, reachable_y, reachable_z = [], [], []

    print("Sweeping Forward Kinematics to map workspace...")
    for s in slider_range:
        for m in mcp_range:
            for p in pip_range:
                # Set joint positions (assuming slider is q[0], mcp is q[2], pip is q[3])
                # DIP is coupled:
                q = np.zeros(plant.num_positions())
                q[0] = s
                q[2] = m
                q[3] = p
                q[4] = p * ( (0.00635 + 0.0002794) / (0.0090 + 0.0002794) ) # DIP coupling
                
                plant.SetPositions(ctx, q)
                
                # Get the pencil tip's world coordinate
                pos = plant.CalcPointsPositions(ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
                
                reachable_x.append(pos[0])
                reachable_y.append(pos[1])
                reachable_z.append(pos[2])

    # Plot the resulting point cloud
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(reachable_x, reachable_y, reachable_z, alpha=0.3, s=10, c=reachable_z, cmap='viridis')
    
    ax.set_title("Reachable Workspace of Pencil Tip")
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_zlabel("World Z (m)")
    
    # Print the min/max bounds so you know exactly where to place the paper
    print("\n--- Workspace Bounds ---")
    print(f"X range: {min(reachable_x):.3f} to {max(reachable_x):.3f}")
    print(f"Y range: {min(reachable_y):.3f} to {max(reachable_y):.3f}")
    print(f"Z range: {min(reachable_z):.3f} to {max(reachable_z):.3f}")
    
    plt.show()

if __name__ == "__main__":
    map_workspace()