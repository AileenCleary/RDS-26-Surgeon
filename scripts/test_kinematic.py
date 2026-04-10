import numpy as np
from pydrake.all import (
    CoulombFriction, DiagramBuilder, AddMultibodyPlantSceneGraph,
    Parser, Simulator, MeshcatVisualizer, StartMeshcat
)
from pydrake.all import Box, RigidTransform, RotationMatrix

PACKAGE_ROOT = "/home/zach/472/finger_drake_sim/models"
SDF_PATH     = PACKAGE_ROOT + "/urdf/finger.sdf"

import logging

class FilterPTCWarning(logging.Filter):
    def filter(self, record):
        return "PTC_onshape_metadata" not in record.getMessage()

drake_logger = logging.getLogger("drake")
drake_logger.addFilter(FilterPTCWarning())

# Diagram 
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

parser = Parser(plant)
parser.package_map().Add("finger_assembly", PACKAGE_ROOT)
parser.AddModels(SDF_PATH)
plant.set_adjacent_bodies_collision_filters(True)

plant.RegisterCollisionGeometry(
    plant.world_body(),
    RigidTransform([0, 0, -0.025]),  # ground center z=-0.025    
    Box(0.3, 0.3, 0.05),            # 2m x 2m x 0.1m plate
    "ground_collision",
    CoulombFriction(0.9, 0.8)
)

plant.RegisterVisualGeometry(
    plant.world_body(),
    RigidTransform([0, 0, -0.025]),
    Box(0.3, 0.3, 0.05),
    "ground_visual",
    [1.0, 1.0, 1.0, 1.0]   
)

plant.Finalize()

# visualizer
meshcat = StartMeshcat()
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
diagram = builder.Build()

# initial conditions
simulator = Simulator(diagram)
ctx = simulator.get_mutable_context()
plant_ctx = plant.GetMyMutableContextFromRoot(ctx)

# initial pose: Splay 0°, MCP 30°，PIP 30°，DIP 30°
plant.GetJointByName("mcp_splay").set_angle(plant_ctx,   0.0)  
plant.GetJointByName("mcp_flexion").set_angle(plant_ctx, -0.523) 
plant.GetJointByName("pip").set_angle(plant_ctx,         -0.523) 
plant.GetJointByName("dip").set_angle(plant_ctx,         -0.523) 

# run
print(f"\nMeshcat: {meshcat.web_url()}")
meshcat.StartRecording()
simulator.set_target_realtime_rate(1.0)
simulator.Initialize()
simulator.AdvanceTo(3.0)
meshcat.PublishRecording()

input("press Enter to exit...")