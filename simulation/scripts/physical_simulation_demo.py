import logging
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import lsq_linear
from PIL import Image
from pydrake.all import (
    StartMeshcat, DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, Simulator,
    RigidTransform, RollPitchYaw, JacobianWrtVariable, Box, CoulombFriction,
    MeshcatVisualizer, Rgba
)

# Mute Onshape GLTF warnings
class _FilterPTC(logging.Filter):
    def filter(self, record):
        return "PTC_onshape_metadata" not in record.getMessage()
logging.getLogger("drake").addFilter(_FilterPTC())

PACKAGE_ROOT = "../models" # Make sure this points to your specific models folder
SDF_PATH  = PACKAGE_ROOT + "/urdf/finger.sdf"

# ==============================================================================
# TENDON KINEMATICS SETUP
# ==============================================================================
R_TENDON = 0.0002794
DIP_COUPLING_RATIO = (0.00635 + R_TENDON) / (0.0090 + R_TENDON)

D = np.array([
    [ 1.0,  0.0,  0.0],
    [-1.0, -1.0,  0.0],
    [-1.0, -1.0,  1.0],
    [ 1.0, -1.0, -1.0],
    [ 1.0,  1.0,  0.0]
])

S = np.array([
    [0.0090+R_TENDON, 0.0, 0.0],
    [0.0064+R_TENDON, 0.0091+R_TENDON, 0.0],
    [0.0029+R_TENDON, 0.00635+R_TENDON, 0.0091+R_TENDON],
    [0.0029+R_TENDON, 0.0126+R_TENDON,  0.0091+R_TENDON],
    [0.0064+R_TENDON, 0.0124+R_TENDON,  0.0]
])

M = D * S

# ==============================================================================
# INNER LOOP MOTOR CONTROLLER & EXACT DECOUPLING
# ==============================================================================

def solve_tendon_tensions(tau_req, min_tension=2.0):
    """
    Analytical solver perfectly mapping Joint Torques to Motor Tensions via exact decoupling.
    Mirrors mapJointTorquesToMotorTorques().
    """
    T = np.full(5, min_tension)
    T[0] = 0.0
    
    # 1. PIP (M2, M3) -> row 2 of M.T
    if np.sign(M[2, 2]) == np.sign(tau_req[2]):
        T[3] = min_tension
        T[2] = (tau_req[2] - M[3, 2] * T[3]) / M[2, 2]
    else:
        T[2] = min_tension
        T[3] = (tau_req[2] - M[2, 2] * T[2]) / M[3, 2]
        
    # 2. MCP (M1, M4) -> row 1 of M.T
    tau_mcp_dist = M[2, 1] * T[2] + M[3, 1] * T[3]
    tau_req_mcp = tau_req[1] - tau_mcp_dist
    
    if np.sign(M[1, 1]) == np.sign(tau_req_mcp):
        T[4] = min_tension
        T[1] = (tau_req_mcp - M[4, 1] * T[4]) / M[1, 1]
    else:
        T[1] = min_tension
        T[4] = (tau_req_mcp - M[1, 1] * T[1]) / M[4, 1]
        
    # 3. Splay (M0) -> row 0 of M.T
    tau_splay_dist = sum(M[i, 0] * T[i] for i in range(1, 5))
    T[0] = (tau_req[0] - tau_splay_dist) / M[0, 0]
    
    T[0] = np.clip(T[0], -60.0, 60.0)
    T[1:] = np.clip(T[1:], min_tension, 60.0)
    return T

class AntagonisticMotorController:
    """
    Inner loop motor PID simulating the ODrive antagonistic tensioning 
    and friction feedforward.
    """
    def __init__(self):
        self.Kp_strong = 0.02
        self.Kd_strong = 0.0005
        self.Kp_soft = 0.005
        self.Kd_soft = 0.0002
        self.pretension = 0.0006
        self.friction = 0.003
        self.R_MOTOR = 0.0042794 # 4.0mm + 0.2794mm
        self.prev_error = np.zeros(5)

    def compute_torque(self, motor_id, target_turns, actual_turns, kp, kd, dt):
        error = target_turns - actual_turns
        derivative = (error - self.prev_error[motor_id]) / dt
        self.prev_error[motor_id] = error
        
        total_torque = (kp * error) + (kd * derivative)
        
        # Friction Feedforward
        if error > 0.01:
            total_torque += self.friction
        elif error < -0.01:
            total_torque -= self.friction
            
        return total_torque

    def compute_tensions(self, q_target, q_actual, dt):
        q_t = q_target[:3].copy()
        q_a = q_actual[:3].copy()
        
        q_t[1] = -q_t[1]  # Invert MCP to match C++
        q_t[2] = -q_t[2]  # Invert PIP to match C++
        q_a[1] = -q_a[1]
        q_a[2] = -q_a[2]
        
        tendon_target = M @ q_t
        tendon_actual = M @ q_a
        
        target_turns = tendon_target / (2 * np.pi * self.R_MOTOR)
        actual_turns = tendon_actual / (2 * np.pi * self.R_MOTOR)
        
        torques = np.zeros(5)
        
        # Motor 0 (Splay)
        torques[0] = self.compute_torque(0, target_turns[0], actual_turns[0], self.Kp_strong, self.Kd_strong, dt)
        
        # MCP (Joint 1: M1 Ext, M4 Flex)
        mcp_err = q_t[1] - q_a[1]
        ext_kp, ext_kd = (self.Kp_strong, self.Kd_strong) if mcp_err > 0.0087 else (self.Kp_soft, self.Kd_soft)
        flex_kp, flex_kd = (self.Kp_strong, self.Kd_strong) if mcp_err < -0.0087 else (self.Kp_soft, self.Kd_soft)
        
        torques[1] = self.compute_torque(1, target_turns[1], actual_turns[1], ext_kp, ext_kd, dt) - self.pretension
        torques[4] = self.compute_torque(4, target_turns[4], actual_turns[4], flex_kp, flex_kd, dt) - self.pretension
        
        # PIP (Joint 2: M3 Ext, M2 Flex)
        pip_err = q_t[2] - q_a[2]
        ext_kp, ext_kd = (self.Kp_strong, self.Kd_strong) if pip_err > 0.0087 else (self.Kp_soft, self.Kd_soft)
        flex_kp, flex_kd = (self.Kp_strong, self.Kd_strong) if pip_err < -0.0087 else (self.Kp_soft, self.Kd_soft)
        
        torques[3] = self.compute_torque(3, target_turns[3], actual_turns[3], ext_kp, ext_kd, dt) - self.pretension
        torques[2] = self.compute_torque(2, target_turns[2], actual_turns[2], flex_kp, flex_kd, dt) - self.pretension
        
        # Convert to tensions (Negative torque pulls tendon)
        tensions = -torques / self.R_MOTOR
        tensions[0] = np.clip(tensions[0], -60.0, 60.0)
        tensions[1:] = np.clip(tensions[1:], 0.0, 60.0)
        
        return tensions

# ==============================================================================
# OUTER LOOP PID CONTROLLER
# ==============================================================================
class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limit = output_limit
        
    def compute(self, target, actual, dt):
        if dt <= 0: return 0.0
        error = target - actual
        
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.output_limit, self.output_limit)
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return np.clip(output, -self.output_limit, self.output_limit)

# ==============================================================================
# TRAJECTORY GENERATION 
# ==============================================================================
LETTERS = {
    'A': [(0, 0), (0.5, 1), (1, 0), (0.75, 0.5), (0.25, 0.5)],
    'B': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5), (0.75, 0.5), (1, 0.25), (0.75, 0), (0, 0)],
    'C': [(1, 1), (0.25, 1), (0, 0.75), (0, 0.25), (0.25, 0), (1, 0)],
    'D': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (1, 0.25), (0.75, 0), (0, 0)],
    'E': [(1, 1), (0, 1), (0, 0.5), (0.75, 0.5), (0, 0.5), (0, 0), (1, 0)],
    'F': [(0, 0), (0, 1), (1, 1), (0, 1), (0, 0.5), (0.75, 0.5)],
    'G': [(1, 1), (0.25, 1), (0, 0.75), (0, 0.25), (0.25, 0), (1, 0), (1, 0.5), (0.5, 0.5)],
    'H': [(0, 1), (0, 0), (0, 0.5), (1, 0.5), (1, 1), (1, 0)],
    'I': [(0.25, 1), (0.75, 1), (0.5, 1), (0.5, 0), (0.25, 0), (0.75, 0)],
    'J': [(0, 0.5), (0.25, 0), (0.75, 0), (1, 0.25), (1, 1)],
    'K': [(0, 1), (0, 0), (0, 0.5), (1, 1), (0, 0.5), (1, 0)],
    'L': [(0, 1), (0, 0), (1, 0)],
    'M': [(0, 0), (0, 1), (0.5, 0.5), (1, 1), (1, 0)],
    'N': [(0, 0), (0, 1), (1, 0), (1, 1)],
    'O': [(0.5, 1), (0.2, 0.8), (0, 0.5), (0.2, 0.2), (0.5, 0), (0.8, 0.2), (1, 0.5), (0.8, 0.8), (0.5, 1)],
    'P': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5)],
    'Q': [(0.8, 0.2), (1, 0.5), (0.8, 0.8), (0.5, 1), (0.2, 0.8), (0, 0.5), (0.2, 0.2), (0.5, 0), (0.8, 0.2), (0.5, 0.5), (1, 0)],
    'R': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5), (0.5, 0.5), (1, 0)],
    'S': [(1, 1), (0.25, 1), (0, 0.75), (0.25, 0.5), (0.75, 0.5), (1, 0.25), (0.75, 0), (0, 0)],
    'T': [(0, 1), (1, 1), (0.5, 1), (0.5, 0)],
    'U': [(0, 1), (0, 0.25), (0.25, 0), (0.75, 0), (1, 0.25), (1, 1)],
    'V': [(0, 1), (0.5, 0), (1, 1)],
    'W': [(0, 1), (0.25, 0), (0.5, 0.5), (0.75, 0), (1, 1)],
    'X': [(0, 1), (1, 0), (0.5, 0.5), (0, 0), (1, 1)],
    'Y': [(0, 1), (0.5, 0.5), (1, 1), (0.5, 0.5), (0.5, 0)],
    'Z': [(0, 1), (1, 1), (0, 0), (1, 0)]
}

def generate_strokes(demo_type="write", word="RDS", image_path="", dt=0.01):
    strokes = []
    scale = 0.02  
    start_y = 0.01
    base_x = 0.105
    
    if demo_type == "write":
        current_offset_y = start_y  
        for letter in word:
            if letter.upper() not in LETTERS: continue
            pts = LETTERS[letter.upper()]
            x_tot, y_tot, f_tot = [], [], []
            for i in range(len(pts) - 1):
                p1, p2 = np.array(pts[i]), np.array(pts[i+1])
                dist = np.linalg.norm(p2 - p1)
                steps = max(2, int((dist * 1.5) / dt))
                
                y_tot.extend(current_offset_y - np.linspace(p1[0], p2[0], steps) * scale)
                x_tot.extend(base_x + np.linspace(p1[1], p2[1], steps) * scale)
                f_tot.extend(np.full(steps, 3.5)) 
            
            strokes.append({'x': np.array(x_tot), 'y': np.array(y_tot), 'f': np.array(f_tot)})
            current_offset_y -= 0.03 
            
    elif demo_type == "shade":
        num_lines = 150
        y_step = 0.0002
        x_min, x_max = base_x, base_x + scale
        x_tot, y_tot, f_tot = [], [], []
        
        for i in range(num_lines):
            f_current = 4.0 - (3.8) * (i / max(1, num_lines - 1))
            y_current = start_y - (i * y_step)
            seg_time = 0.5 
            steps = max(2, int(seg_time / dt))
            x_start, x_end = (x_min, x_max) if i % 2 == 0 else (x_max, x_min)
                
            y_tot.extend(np.full(steps, y_current))
            x_tot.extend(np.linspace(x_start, x_end, steps))
            f_tot.extend(np.full(steps, f_current))
            
        strokes.append({'x': np.array(x_tot), 'y': np.array(y_tot), 'f': np.array(f_tot)})
        
    return strokes

# ==============================================================================
# DRAKE SIMULATION & PLOTTING
# ==============================================================================
def build_diagram(meshcat):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    parser = Parser(plant)
    parser.package_map().Add("finger_assembly", PACKAGE_ROOT)
    
    # Load the correct SDF based on the selected mode
    sdf = SDF_PATH
    parser.AddModels(sdf)

    paper_thickness = 0.002
    paper_size = 0.20
    paper_origin = np.array([0.13331402, -0.00136361, 0.07774057]) 
    R_paper = RollPitchYaw(0, np.radians(30), 0.0)
    X_WPaper = RigidTransform(R_paper, paper_origin)
    paper_shape = Box(paper_size, paper_size, paper_thickness)
    plant.RegisterCollisionGeometry(plant.world_body(), X_WPaper, paper_shape, "paper_collision", CoulombFriction(0.6, 0.5))
    plant.RegisterVisualGeometry(plant.world_body(), X_WPaper, paper_shape, "paper_visual", [0.95, 0.95, 0.95, 1.0])
    
    plant.Finalize()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build(), plant, scene_graph


# --- MESHAT PASSED AS ARGUMENT ---
def run_simulation(demo_type="write", letter="A"):
    diagram, plant, scene_graph = build_diagram(meshcat)
    
    simulator = Simulator(diagram)
    ctx = simulator.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(ctx)
    
    pencil_link = plant.GetBodyByName("pencil")
    pencil_tip_local = [0, 0, 0.02]
    world_frame = plant.world_frame()
    
    initial_tip_pos = plant.CalcPointsPositions(
        plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame
    ).flatten()

    paper_z = initial_tip_pos[2] - 0.05
    paper_thickness = 0.002

    R_paper = RollPitchYaw(0, np.radians(30), 0.0).ToRotationMatrix()
    paper_origin = np.array([initial_tip_pos[0] - 0.02, initial_tip_pos[1], paper_z + paper_thickness/2])
    paper_normal = R_paper.multiply([0, 0, 1])
    
    surface_origin = paper_origin + paper_normal * (paper_thickness / 2.0)
    strokes = generate_strokes(demo_type, letter)

    all_x = []
    all_y = []
    for s in strokes:
        all_x.extend(s['x'])
        all_y.extend(s['y'])
    
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    center_dz = -(paper_normal[0]*(center_x - surface_origin[0]) + paper_normal[1]*(center_y - surface_origin[1])) / paper_normal[2]
    drawing_center_world = np.array([center_x, center_y, surface_origin[2] + center_dz])
    
    # Calculate finger's initial tip position (End of the distal link, not pencil tip)
    finger_tip_local = [0, 0, 0] 
    finger_tip_pos = plant.CalcPointsPositions(
        plant_ctx, pencil_link.body_frame(), finger_tip_local, world_frame
    ).flatten()

    print("\n" + "="*70)
    print("WORKSPACE RELATIVE POSITIONS (vs Finger Tip Initial Pos)")
    print("="*70)
    print(f"Finger Initial Tip Pos (World)  : [{finger_tip_pos[0]:.4f}, {finger_tip_pos[1]:.4f}, {finger_tip_pos[2]:.4f}]")
    print(f"Paper Center Position (World)   : [{paper_origin[0]:.4f}, {paper_origin[1]:.4f}, {paper_origin[2]:.4f}]")
    
    rel_paper = paper_origin - finger_tip_pos
    print(f"Paper Center Rel to Finger      : [{rel_paper[0]:.4f}, {rel_paper[1]:.4f}, {rel_paper[2]:.4f}]")
    
    rel_draw = drawing_center_world - finger_tip_pos
    print(f"Drawing Space Center Rel to Tip : [{rel_draw[0]:.4f}, {rel_draw[1]:.4f}, {rel_draw[2]:.4f}]")
    
    print(f"Drawing Bounds (Relative X)     : [{min_x - finger_tip_pos[0]:.4f}, {max_x - finger_tip_pos[0]:.4f}]")
    print(f"Drawing Bounds (Relative Y)     : [{min_y - finger_tip_pos[1]:.4f}, {max_y - finger_tip_pos[1]:.4f}]")
    print("="*70 + "\n")
    
    dt = 0.01
    curr_t = 0.0
    log = {
        't': [], 'x_act': [], 'y_act': [], 'x_des': [], 'y_des': [],
        'f_act': [], 'f_des': [], 'q_act': [], 'q_des': [], 'tensions': []
    }
    
    print(f"\n---> Open Meshcat to watch the '{demo_type}' simulation: {meshcat.web_url()} <---")
    meshcat.StartRecording()

    # DYNAMIC JOINT INDICES
    idx_splay = 0
    idx_mcp = 1
    idx_pip = 2
    idx_dip = 3

    # ==============================================================================
    # INITIALIZE THE OUTER LOOP (PID & SIMULATED PHYSICAL STATE)
    # ==============================================================================
    # Start the actual position exactly where the base kinematic target is
    q_actual = plant.GetPositions(plant_ctx).copy()
    
    # Instantiate PIDs similar to the Teensy (Max output offset = 15 degrees)
    pid_splay = PIDController(kp=1.0, ki=0.0, kd=0.001, output_limit=np.radians(15))
    pid_mcp   = PIDController(kp=1.0, ki=0.0, kd=0.001, output_limit=np.radians(15))
    pid_pip   = PIDController(kp=1.0, ki=0.0, kd=0.001, output_limit=np.radians(15))
    pid_force = PIDController(kp=0.01, ki=0.0, kd=0.0, output_limit=10.0)

    motor_ctrl = AntagonisticMotorController()

    def solve_ik(target_pos, q_guess):
        MAX_ITERATIONS = 50
        TOLERANCE = 1e-4  
        LEARNING_RATE = 0.5 
        q = q_guess.copy()
        
        for _ in range(MAX_ITERATIONS):
            plant.SetPositions(plant_ctx, q)
            current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
            err = target_pos - current_pos
            if np.linalg.norm(err) < TOLERANCE: break
                
            J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
            dq = J.T @ np.linalg.inv(J @ J.T + 1e-4*np.eye(3)) @ err
            q += dq * LEARNING_RATE
            
            q[idx_splay] = np.clip(q[idx_splay], np.radians(-10), np.radians(10)) 
            q[idx_mcp] = np.clip(q[idx_mcp], 0.0, np.radians(90))             
            q[idx_pip] = np.clip(q[idx_pip], 0.0, np.radians(90))             
            q[idx_dip] = np.clip(q[idx_pip] * DIP_COUPLING_RATIO, 0.0, np.radians(90))   
          
        return q

    def run_pid_and_physics_step(q_base_target):
        nonlocal q_actual
        
        # 1. Outer Loop: Calculate Correction based on sensor error
        correction = np.zeros_like(q_base_target)
        correction[idx_splay] = pid_splay.compute(q_base_target[idx_splay], q_actual[idx_splay], dt)
        correction[idx_mcp] = pid_mcp.compute(q_base_target[idx_mcp], q_actual[idx_mcp], dt)
        correction[idx_pip] = pid_pip.compute(q_base_target[idx_pip], q_actual[idx_pip], dt)
        
        # 2. Corrected Target (Equivalent to C++ correctedJointTarget)
        q_corrected = q_base_target + correction
        q_corrected[idx_dip] = q_corrected[idx_pip] * DIP_COUPLING_RATIO
        
        # 3. Simulate Physical Lag & Inertia
        lag_factor = 0.20
        q_actual_next = q_actual + (q_corrected - q_actual) * lag_factor
        
        # 4. Simulate Contact Constraints (Crucial for position-based force tracking)
        plant.SetPositions(plant_ctx, q_actual_next)
        next_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
        dist_to_plane = np.dot(next_pos - surface_origin, paper_normal)
        
        if dist_to_plane < 0:
            # Tip penetrated paper -> push physical joints back along Jacobian
            J_v = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
            dx = -dist_to_plane * paper_normal
            dq = J_v.T @ np.linalg.inv(J_v @ J_v.T + 1e-4*np.eye(3)) @ dx
            q_actual_next += dq
            
        q_actual = q_actual_next
        plant.SetPositions(plant_ctx, q_actual)
        
        current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
        return current_pos, q_corrected

    def interpolate_joints(q_start, q_end, duration):
        nonlocal curr_t
        steps = max(2, int(duration / dt))
        for i in range(steps):
            alpha = i / (steps - 1)
            q_base = q_start + alpha * (q_end - q_start)
            q_base[idx_dip] = q_base[idx_pip] * DIP_COUPLING_RATIO
            
            # Run the cascaded physics step
            current_pos, q_corrected = run_pid_and_physics_step(q_base)
            
            tensions = np.zeros(5) # Idle tension during interpolation
            
            log['t'].append(curr_t)
            log['y_act'].append(current_pos[1])
            log['x_act'].append(current_pos[0])
            log['f_act'].append(0.0)
            log['q_act'].append(q_actual.copy()) 
            log['y_des'].append(current_pos[1])
            log['x_des'].append(current_pos[0])
            log['f_des'].append(0.0)
            log['q_des'].append(q_base.copy())   
            log['tensions'].append(tensions)
            
            ctx.SetTime(curr_t)
            diagram.ForcedPublish(ctx)
            curr_t += dt
        return q_end

    # Set the starting target to where the plant initialized
    q_target = plant.GetPositions(plant_ctx).copy()

    for idx, stroke in enumerate(strokes):
        # LIFT & MOVE TRANSITION
        if idx >= 0:
            tx, ty = stroke['x'][0], stroke['y'][0]
            dz_slope = -(paper_normal[0]*(tx - surface_origin[0]) + paper_normal[1]*(ty - surface_origin[1])) / paper_normal[2]
            next_start_pos = np.array([tx, ty, surface_origin[2] + dz_slope])
            
            q_next = solve_ik(next_start_pos, q_target)

            q_ext1 = q_target.copy()
            q_ext1[idx_mcp] = 0.0
            q_target = interpolate_joints(q_target, q_ext1, duration=0.1)

            q_ext2 = q_target.copy()
            q_ext2[idx_pip] = np.radians(40.0)
            q_target = interpolate_joints(q_target, q_ext2, duration=0.1)

            q_move = q_target.copy()
            q_move[idx_splay] = q_next[idx_splay]
            q_target = interpolate_joints(q_target, q_move, duration=0.2)

            q_target = interpolate_joints(q_target, q_next, duration=0.15)
        
        # DRAWING STROKE
        stroke_points = [] 
        actual_normal_force = 0.0 # Initialize for the first PID calculation
        
        for i in range(len(stroke['x'])):
            tx, ty = stroke['x'][i], stroke['y'][i]
            target_force = stroke['f'][i]
            
            # 1. Target exactly on the surface (Pure Kinematics)
            dz_slope = -(paper_normal[0]*(tx - surface_origin[0]) + paper_normal[1]*(ty - surface_origin[1])) / paper_normal[2]
            target_pos = np.array([tx, ty, surface_origin[2] + dz_slope])
            q_target = solve_ik(target_pos, q_target)
            
            # 2. Run Outer Loop & Inner Loop Position Tracking
            current_pos, q_corrected = run_pid_and_physics_step(q_target)
            q_des_rel = q_corrected[idx_splay:idx_pip+1]
            q_act_rel = q_actual[idx_splay:idx_pip+1]
            tensions_fb = motor_ctrl.compute_tensions(q_des_rel, q_act_rel, dt)
            
            # 3. Closed-Loop Force Control (PID + Feedforward Jacobian)
            force_correction = pid_force.compute(target_force, actual_normal_force, dt)
            cmd_force = np.clip(target_force + force_correction, 0.0, 15.0)
            
            J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
            J_finger = J[:, idx_splay:idx_pip+1] 
            
            f_intent = -paper_normal * cmd_force 
            tau_req = J_finger.T @ f_intent 
            tensions_ff = solve_tendon_tensions(tau_req, min_tension=0.0)
            
            # 4. Combine & Execute
            tensions = tensions_fb + tensions_ff
            
            # 5. Measure resulting force (Feeds back into PID next loop)
            tau_actual = M.T @ tensions
            F_actual = np.linalg.pinv(J_finger.T) @ tau_actual
            actual_normal_force = np.dot(F_actual, -paper_normal)
            
            draw_pos = current_pos + (paper_normal * 0.0001)
            stroke_points.append(draw_pos)
            
            if len(stroke_points) > 1 and actual_normal_force > 0.1:
                intensity = np.clip(float(actual_normal_force), 0.0, 4.0)
                gray = 1.00 - (intensity * 0.25) 
                
                path = f"drawing/stroke_{idx}/seg_{i}"
                vertices = np.array([stroke_points[-2], stroke_points[-1]]).T
                
                meshcat.SetLine(path, vertices, 3.0, Rgba(gray, gray, gray, 1.0))
                meshcat.SetProperty(path, "visible", False, 0.0)
                meshcat.SetProperty(path, "visible", True, curr_t)
            
            log['t'].append(curr_t)
            log['y_act'].append(current_pos[1])
            log['x_act'].append(current_pos[0])
            log['f_act'].append(actual_normal_force)
            log['q_act'].append(q_actual.copy())
            log['y_des'].append(target_pos[1])
            log['x_des'].append(target_pos[0])
            log['f_des'].append(target_force)
            log['q_des'].append(q_target.copy())
            log['tensions'].append(tensions)

            ctx.SetTime(curr_t)
            diagram.ForcedPublish(ctx)
            curr_t += dt

    meshcat.PublishRecording()

    # Convert log dict to arrays before returning
    for k in log:
        log[k] = np.array(log[k])
        
    return log

def plot_results(log, title):
    t = log['t']
    x_act = -log['y_act']
    y_act = log['x_act']
    x_des = -log['y_des']
    y_des = log['x_des']
    f_act = log['f_act']
    f_des = log['f_des']
    q_act = log['q_act']
    q_des = log['q_des']
    tensions = log['tensions']

    fig = plt.figure(figsize=(14, 10))
    fig.canvas.manager.set_window_title(title)
    
    draw_mask = f_act > 0.01
    
    ax1 = plt.subplot(2, 2, 1)
    if len(x_act[draw_mask]) > 0:
        sc = ax1.scatter(x_act[draw_mask], y_act[draw_mask], c=f_act[draw_mask], cmap='Greys', s=20, edgecolor='none', vmin=0, vmax=np.max(f_act)+0.5)
        plt.colorbar(sc, ax=ax1, label='Normal Force (N)')
    ax1.plot(x_des[draw_mask], y_des[draw_mask], color='blue', linestyle='--', alpha=0.3, label='Desired Path')
    ax1.set_title(f"Drawing Canvas (X-Y Plane)")
    ax1.set_xlabel("World X Coordinate")
    ax1.set_ylabel("World Y Coordinate")
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(t, f_des, color='black', linestyle='--', label='Desired Force', linewidth=2)
    ax2.plot(t, f_act, color='crimson', label='Actual Force', linewidth=2)
    ax2.set_title("Kinematic Pencil Tip Normal Force")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (N)")
    ax2.grid(True, alpha=0.3)

    ax3 = plt.subplot(2, 2, 3)
    
    ax3.plot(t, np.degrees(q_des[:, 0]), color='C0', linestyle='--', label='Desired Splay')
    ax3.plot(t, np.degrees(q_des[:, 1]), color='C1', linestyle='--', label='Desired MCP')
    ax3.plot(t, np.degrees(q_des[:, 2]), color='C2', linestyle='--', label='Desired PIP')
    ax3.plot(t, np.degrees(q_act[:, 0]), color='C0', label='Actual Splay')
    ax3.plot(t, np.degrees(q_act[:, 1]), color='C1', label='Actual MCP')
    ax3.plot(t, np.degrees(q_act[:, 2]), color='C2', label='Actual PIP')
    ax3.set_title("Joint Kinematics (Actual vs Desired)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Angles (deg)")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    ax4 = plt.subplot(2, 2, 4)
    labels = ['T0 (Splay)', 'T1 (MCP Ext)', 'T2 (PIP Flex)', 'T3 (PIP Ext)', 'T4 (MCP Flex)']
    for i in range(5):
        ax4.plot(t, tensions[:, i], label=labels[i])
    ax4.set_title("Required Tendon Tensions")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Tension (N)")
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    meshcat = StartMeshcat()
    letter = input("Please enter a letter to write: ")
    log_l = run_simulation("write", letter=letter)
    plot_results(log_l, f"Writing '{letter}'")

    meshcat = StartMeshcat()
    log_s = run_simulation("shade")
    plot_results(log_s, "Variable Force Shading")
        
    input("Press Enter to close Meshcat and exit...")