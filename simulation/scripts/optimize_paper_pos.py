import logging
import numpy as np
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
    RollPitchYaw, JacobianWrtVariable
)

# Mute Onshape GLTF warnings
class _FilterPTC(logging.Filter):
    def filter(self, record):
        return "PTC_onshape_metadata" not in record.getMessage()
logging.getLogger("drake").addFilter(_FilterPTC())

PACKAGE_ROOT = "../models"
SDF_PATH     = PACKAGE_ROOT + "/urdf/finger_sliding.sdf"

# ==============================================================================
# KINEMATICS SETUP
# ==============================================================================
R_TENDON = 0.0002794
DIP_COUPLING_RATIO = (0.00635 + R_TENDON) / (0.0090 + R_TENDON)

LETTERS = {
    'R': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5), (0.5, 0.5), (1, 0)],
    'D': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (1, 0.25), (0.75, 0), (0, 0)],
    'S': [(1, 1), (0.25, 1), (0, 0.75), (0.25, 0.5), (0.75, 0.5), (1, 0.25), (0.75, 0), (0, 0)]
}

def generate_strokes(demo_type="write", dt=0.05, base_x=0.10):
    """Returns a coarse list of continuous strokes for fast testing."""
    strokes = []
    scale = 0.02  
    start_y = 0.05
    
    if demo_type == "write":
        current_offset_y = start_y  
        for letter in ['R', 'D', 'S']:
            pts = LETTERS[letter]
            x_tot, y_tot = [], []
            for i in range(len(pts) - 1):
                p1, p2 = np.array(pts[i]), np.array(pts[i+1])
                dist = np.linalg.norm(p2 - p1)
                steps = max(2, int((dist * 1.5) / dt))
                
                y_tot.extend(current_offset_y - np.linspace(p1[0], p2[0], steps) * scale)
                x_tot.extend(base_x + np.linspace(p1[1], p2[1], steps) * scale)
            
            strokes.append({'x': np.array(x_tot), 'y': np.array(y_tot)})
            current_offset_y -= 0.03 
            
    return strokes

# ==============================================================================
# OPTIMIZER SETUP
# ==============================================================================
def build_headless_diagram():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    parser.package_map().Add("finger_assembly", PACKAGE_ROOT)
    parser.AddModels(SDF_PATH)
    plant.Finalize()
    return builder.Build(), plant

def solve_ik_state(target_pos, q_guess, plant, plant_ctx, pencil_link, pencil_tip_local, world_frame):
    """Isolated IK solver that returns None if it fails to reach the target."""
    MAX_ITERATIONS = 50
    TOLERANCE = 1e-4  
    LEARNING_RATE = 0.5 
    q = q_guess.copy()
    
    for _ in range(MAX_ITERATIONS):
        plant.SetPositions(plant_ctx, q)
        current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
        err = target_pos - current_pos
        
        if np.linalg.norm(err) < TOLERANCE: 
            return q # Success
            
        J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
        dq = J.T @ np.linalg.inv(J @ J.T + 1e-4*np.eye(3)) @ err
        q += dq * LEARNING_RATE
        
        q[1] = np.clip(q[1], np.radians(-10), np.radians(10)) 
        q[2] = np.clip(q[2], 0.0, np.radians(90))             
        q[3] = np.clip(q[3], 0.0, np.radians(90))             
        q[4] = np.clip(q[3] * DIP_COUPLING_RATIO, 0.0, np.radians(90))             
        
    return None # Failed to converge

def run_optimization():
    print("\n========================================")
    print("  ROBOTIC FINGER WORKSPACE OPTIMIZER  ")
    print("========================================")
    
    diagram, plant = build_headless_diagram()
    ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(ctx)
    
    pencil_link = plant.GetBodyByName("pencil")
    pencil_tip_local = [0, 0, 0.02] 
    world_frame = plant.world_frame()
    
    best_score = -np.inf
    best_params = {"pitch": 20, "z": -0.05, "x": 0.10}
    
    # 1. Define the search grid
    pitches = [-5, 0, 5, 10, 15, 20, 25, 30]       # degrees
    paper_zs = [-0.06, -0.05, -0.04, -0.03] # meters relative to tip
    base_xs = [0.08, 0.10, 0.12]            # meters
    paper_xs = [-0.025, -0.02, -0.015] # meters
    
    q_home = np.zeros(5)
    total_configs = len(pitches) * len(paper_zs) * len(base_xs) * len(paper_xs)
    configs_tested = 0
    
    for pitch in pitches:
        for p_z in paper_zs:
            for b_x in base_xs:
                configs_tested += 1
                if configs_tested % 10 == 0:
                    print(f"Evaluating configuration {configs_tested}/{total_configs}...")
                
                # Setup test scenario
                R_paper = RollPitchYaw(0, np.radians(pitch), 0.0).ToRotationMatrix()
                paper_normal = R_paper.multiply([0, 0, 1])
                paper_origin = np.array([b_x + p_z, 0.0, p_z])
                
                # Get a sparse, fast trajectory to test
                test_strokes = generate_strokes("write", dt=0.05, base_x=b_x)
                
                score = 0
                valid = True
                
                # Test the extreme points of the drawing
                for stroke in test_strokes:
                    # Just test 5 evenly spaced points per stroke to save time
                    indices = np.linspace(0, len(stroke['x'])-1, 5, dtype=int)
                    for i in indices:
                        tx, ty = stroke['x'][i], stroke['y'][i]
                        dz_slope = -(paper_normal[0]*(tx - paper_origin[0]) + paper_normal[1]*(ty - paper_origin[1])) / paper_normal[2]
                        target_pos = np.array([tx, ty, p_z + dz_slope])
                        
                        q_res = solve_ik_state(target_pos, q_home, plant, plant_ctx, pencil_link, pencil_tip_local, world_frame)
                        
                        if q_res is None:
                            valid = False
                            break
                            
                        # Calculate Yoshikawa Manipulability
                        plant.SetPositions(plant_ctx, q_res)
                        J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
                        J_finger = J[:, 1:4] # Only use rotational joints
                        mu = np.sqrt(max(0, np.linalg.det(J_finger @ J_finger.T)))
                        
                        # Penalize being within 5 degrees of a joint limit
                        margin = np.radians(5)
                        if q_res[2] < margin or q_res[2] > np.radians(90)-margin: mu -= 0.05
                        if q_res[3] < margin or q_res[3] > np.radians(90)-margin: mu -= 0.05
                        
                        score += mu
                    if not valid: break
                
                if valid and score > best_score:
                    best_score = score
                    best_params = {"pitch": pitch, "z": p_z, "x": b_x}

    print("\n========================================")
    print(f"  OPTIMAL SETUP FOUND (Score: {best_score:.3f})  ")
    print("========================================")
    print(f"  Paper Pitch : {best_params['pitch']} degrees")
    print(f"  Paper Z-off : {best_params['z']} meters")
    print(f"  Base X-pos  : {best_params['x']} meters\n")

if __name__ == "__main__":
    run_optimization()