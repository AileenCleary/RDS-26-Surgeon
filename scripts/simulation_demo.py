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
SDF_PATH_1D  = PACKAGE_ROOT + "/urdf/finger_sliding.sdf"
SDF_PATH_2D  = PACKAGE_ROOT + "/urdf/finger_planar.sdf"

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

def solve_tendon_tensions(tau_joint, min_tension=5.0):
    lower_bounds = [-100.0, min_tension, min_tension, min_tension, min_tension]
    upper_bounds = [100.0, 100.0, 100.0, 100.0, 100.0]
    
    res = lsq_linear(M.T, tau_joint, bounds=(lower_bounds, upper_bounds))
    return res.x

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
    start_y = 0.06
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

    elif demo_type == "image":
        try:
            img = Image.open(image_path).convert('L')
            img = img.rotate(-90, expand=True)
            w, h = img.size
            aspect_ratio = w / h
            
            res_y = 200 
            res_x = max(2, int(res_y * aspect_ratio))
            img = img.resize((res_x, res_y))
            pixel_data = np.array(img)
            
        except Exception as e:
            print(f"\n[!] Error loading image '{image_path}': {e}")
            return generate_strokes("shade", dt=dt)
            
        scale_y = 0.20
        scale_x_paper = scale_y * aspect_ratio
        
        x_min, x_max = base_x-0.02, base_x-0.02 + scale_x_paper
        y_min, y_max = scale_y/2, -scale_y/2
        
        y_steps = np.linspace(y_min, y_max, res_y)
        x_steps_forward = np.linspace(x_min, x_max, res_x)
        x_steps_backward = np.linspace(x_max, x_min, res_x)
        
        current_stroke_x, current_stroke_y, current_stroke_f = [], [], []
        
        for i in range(res_y):
            if i % 2 == 0:
                x_row = x_steps_forward
                row_pixels = pixel_data[i, :]
            else:
                x_row = x_steps_backward
                row_pixels = pixel_data[i, ::-1] 
                
            row_force = 4.0 * (1.0 - (row_pixels / 255.0))

            for j in range(res_x):
                if row_force[j] >= 0.4:
                    current_stroke_x.append(x_row[j])
                    current_stroke_y.append(y_steps[i]) 
                    current_stroke_f.append(row_force[j])
                else:
                    if len(current_stroke_x) > 0:
                        strokes.append({
                            'x': np.array(current_stroke_x), 
                            'y': np.array(current_stroke_y), 
                            'f': np.array(current_stroke_f)
                        })
                        current_stroke_x, current_stroke_y, current_stroke_f = [], [], []

        if len(current_stroke_x) > 0:
            strokes.append({
                'x': np.array(current_stroke_x), 
                'y': np.array(current_stroke_y), 
                'f': np.array(current_stroke_f)
            })
        
    return strokes

# ==============================================================================
# DRAKE SIMULATION & PLOTTING
# ==============================================================================
def build_diagram(meshcat, is_planar):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    parser = Parser(plant)
    parser.package_map().Add("finger_assembly", PACKAGE_ROOT)
    
    # Load the correct SDF based on the selected mode
    sdf = SDF_PATH_2D if is_planar else SDF_PATH_1D
    parser.AddModels(sdf)

    paper_thickness = 0.002
    paper_size = 0.20
    paper_origin = np.array([0.13331402, -0.00136361, -0.07225943]) 
    R_paper = RollPitchYaw(0, np.radians(30), 0.0)
    X_WPaper = RigidTransform(R_paper, paper_origin)
    paper_shape = Box(paper_size, paper_size, paper_thickness)
    plant.RegisterCollisionGeometry(plant.world_body(), X_WPaper, paper_shape, "paper_collision", CoulombFriction(0.6, 0.5))
    plant.RegisterVisualGeometry(plant.world_body(), X_WPaper, paper_shape, "paper_visual", [0.95, 0.95, 0.95, 1.0])
    
    plant.Finalize()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build(), plant, scene_graph


# --- MESHAT PASSED AS ARGUMENT ---
def run_simulation(demo_type="write", word="RDS", image=""):
    is_planar = (demo_type == "image")
    idx_offset = 1 if is_planar else 0 
    
    diagram, plant, scene_graph = build_diagram(meshcat, is_planar)
    
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
    paper_size = 0.20
    meshcat.SetObject("paper", Box(paper_size, paper_size, paper_thickness), rgba=Rgba(0.95, 0.95, 0.95, 1.0))

    R_paper = RollPitchYaw(0, np.radians(30), 0.0).ToRotationMatrix()
    paper_origin = np.array([initial_tip_pos[0] - 0.02, initial_tip_pos[1], paper_z + paper_thickness/2])
    meshcat.SetTransform("paper", RigidTransform(R_paper, paper_origin))
    paper_normal = R_paper.multiply([0, 0, 1])
    
    surface_origin = paper_origin + paper_normal * (paper_thickness / 2.0)
    strokes = generate_strokes(demo_type, word, image)
    
    dt = 0.01
    curr_t = 0.0
    log = {
        't': [], 'x_act': [], 'y_act': [], 'x_des': [], 'y_des': [],
        'f_act': [], 'f_des': [], 'q_act': [], 'q_des': [], 'tensions': []
    }
    
    print(f"\n---> Open Meshcat to watch the '{demo_type}' simulation: {meshcat.web_url()} <---")
    meshcat.StartRecording()

    # DYNAMIC JOINT INDICES
    idx_y = 0 if is_planar else None
    idx_x = 1 if is_planar else 0
    idx_splay = 1 + idx_offset
    idx_mcp = 2 + idx_offset
    idx_pip = 3 + idx_offset
    idx_dip = 4 + idx_offset

    # ==============================================================================
    # INITIALIZE THE OUTER LOOP (PID & SIMULATED PHYSICAL STATE)
    # ==============================================================================
    # Start the actual position exactly where the base kinematic target is
    q_actual = plant.GetPositions(plant_ctx).copy()
    
    # Instantiate PIDs similar to the Teensy (Max output offset = 15 degrees)
    pid_splay = PIDController(kp=1.0, ki=0.0, kd=0.001, output_limit=np.radians(15))
    pid_mcp   = PIDController(kp=1.0, ki=0.0, kd=0.001, output_limit=np.radians(15))
    pid_pip   = PIDController(kp=1.0, ki=0.0, kd=0.001, output_limit=np.radians(15))

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
            
            if is_planar: q[idx_y] = np.clip(q[idx_y], -0.3, 0.3)
            q[idx_x] = np.clip(q[idx_x], -0.3, 0.3)
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
        
        # 2. Corrected Target
        q_corrected = q_base_target + correction
        q_corrected[idx_dip] = q_corrected[idx_pip] * DIP_COUPLING_RATIO
        
        # 3. Simulate Physical Lag & Noise
        lag_factor = 0.20
        
        # The physical joints chase the corrected target, dragging slightly behind
        q_actual = q_actual + (q_corrected - q_actual) * lag_factor
        
        # Update Drake to render the actual physical position
        plant.SetPositions(plant_ctx, q_actual)
        return plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()

    def interpolate_joints(q_start, q_end, duration):
        nonlocal curr_t
        steps = max(2, int(duration / dt))
        for i in range(steps):
            alpha = i / (steps - 1)
            q_base = q_start + alpha * (q_end - q_start)
            q_base[idx_dip] = q_base[idx_pip] * DIP_COUPLING_RATIO
            
            # Run the PID/Physics step
            current_pos = run_pid_and_physics_step(q_base)
            
            tensions = solve_tendon_tensions(np.zeros(3))
            
            log['t'].append(curr_t)
            log['y_act'].append(current_pos[1])
            log['x_act'].append(current_pos[0])
            log['f_act'].append(0.0)
            log['q_act'].append(q_actual.copy()) # Log the ACTUAL lagging state
            log['y_des'].append(current_pos[1])
            log['x_des'].append(current_pos[0])
            log['f_des'].append(0.0)
            log['q_des'].append(q_base.copy())   # Log the PURE target state
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
            if is_planar: q_move[idx_y] = q_next[idx_y]
            q_move[idx_x] = q_next[idx_x]
            q_move[idx_splay] = q_next[idx_splay]
            q_target = interpolate_joints(q_target, q_move, duration=0.2)

            q_target = interpolate_joints(q_target, q_next, duration=0.15)
        
        # DRAWING STROKE
        stroke_points = [] 
        
        for i in range(len(stroke['x'])):
            tx, ty = stroke['x'][i], stroke['y'][i]
            dz_slope = -(paper_normal[0]*(tx - surface_origin[0]) + paper_normal[1]*(ty - surface_origin[1])) / paper_normal[2]
            target_pos = np.array([tx, ty, surface_origin[2] + dz_slope])
            
            q_target = solve_ik(target_pos, q_target)
            
            # Run the PID/Physics step
            current_pos = run_pid_and_physics_step(q_target)
            
            J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
            J_finger = J[:, idx_splay:idx_pip+1] 
            
            f_intent = -paper_normal * stroke['f'][i] 
            tau_req = J_finger.T @ f_intent 
            
            tensions = solve_tendon_tensions(tau_req)
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
            log['q_act'].append(q_actual.copy()) # Log lagging state
            log['y_des'].append(target_pos[1])
            log['x_des'].append(target_pos[0])
            log['f_des'].append(stroke['f'][i])
            log['q_des'].append(q_target.copy()) # Log pure state
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
    is_planar = q_act.shape[1] == 6
    idx_offset = 1 if is_planar else 0

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
        
    if is_planar:
        ax3.plot(t, q_des[:, 0], color='C5', linestyle='--', label='Desired Slider Y (m)')
        ax3.plot(t, q_act[:, 0], color='C5', label='Actual Slider Y (m)')
    ax3.plot(t, q_des[:, idx_offset], color='C4', linestyle='--', label='Desired Slider X (m)')
    ax3.plot(t, np.degrees(q_des[:, 1 + idx_offset]), color='C0', linestyle='--', label='Desired Splay')
    ax3.plot(t, np.degrees(q_des[:, 2 + idx_offset]), color='C1', linestyle='--', label='Desired MCP')
    ax3.plot(t, np.degrees(q_des[:, 3 + idx_offset]), color='C2', linestyle='--', label='Desired PIP')
    ax3.plot(t, np.degrees(q_des[:, 4 + idx_offset]), color='C3', linestyle='--', label='Desired DIP')
    ax3.plot(t, q_act[:, idx_offset], color='C4', label='Actual Slider X (m)')
    ax3.plot(t, np.degrees(q_act[:, 1 + idx_offset]), color='C0', label='Actual Splay')
    ax3.plot(t, np.degrees(q_act[:, 2 + idx_offset]), color='C1', label='Actual MCP')
    ax3.plot(t, np.degrees(q_act[:, 3 + idx_offset]), color='C2', label='Actual PIP')
    ax3.plot(t, np.degrees(q_act[:, 4 + idx_offset]), color='C3', label='Actual DIP')
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
    word = input("Please enter a word to write: ")
    log_w = run_simulation("write", word=word)
    plot_results(log_w, f"Writing '{word}'")

    meshcat = StartMeshcat()
    log_s = run_simulation("shade")
    plot_results(log_s, "Variable Force Shading")

    meshcat = StartMeshcat()
    img_path = input("Enter the file path of your image: ").strip()
    log_i = run_simulation("image", image=img_path)
    plot_results(log_i, f"Drawing Image: {img_path}")
        
    input("Press Enter to close Meshcat and exit...")