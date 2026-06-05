import numpy as np
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, Simulator,
    StartMeshcat, MeshcatVisualizer, JointSliders
)

def run_teleop():
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    parser = Parser(plant)
    parser.package_map().Add("finger_assembly", "../models")
    parser.AddModels("../models/urdf/finger_sliding.sdf")
    plant.Finalize()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    # Adds interactive sliders to Meshcat for every joint
    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    
    diagram = builder.Build()
    simulator = Simulator(diagram)
    
    print(f"\n---> Open Meshcat: {meshcat.web_url()} <---")
    print("Use the 'Controls' menu in the top right of Meshcat to move the joints.")
    print("Press Ctrl+C in the terminal to exit.")
    
    # Run indefinitely so you can play with the sliders
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(np.inf)

if __name__ == "__main__":
    run_teleop()