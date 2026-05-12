# import logging
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.optimize import lsq_linear
# from PIL import Image
# from pydrake.all import (
#     StartMeshcat, DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, Simulator,
#     RigidTransform, RollPitchYaw, JacobianWrtVariable, Box, CoulombFriction,
#     MeshcatVisualizer, Rgba
# )

# # Mute Onshape GLTF warnings
# class _FilterPTC(logging.Filter):
#     def filter(self, record):
#         return "PTC_onshape_metadata" not in record.getMessage()
# logging.getLogger("drake").addFilter(_FilterPTC())

# PACKAGE_ROOT = "../models"
# SDF_PATH     = PACKAGE_ROOT + "/urdf/finger_sliding.sdf"

# # ==============================================================================
# # TENDON KINEMATICS SETUP
# # ==============================================================================
# R_TENDON = 0.0002794
# DIP_COUPLING_RATIO = (0.00635 + R_TENDON) / (0.0090 + R_TENDON)

# D = np.array([
#     [ 1.0,  0.0,  0.0],
#     [-1.0, -1.0,  0.0],
#     [-1.0, -1.0,  1.0],
#     [ 1.0, -1.0, -1.0],
#     [ 1.0,  1.0,  0.0]
# ])

# S = np.array([
#     [0.0090+R_TENDON, 0.0, 0.0],
#     [0.0064+R_TENDON, 0.0091+R_TENDON, 0.0],
#     [0.0029+R_TENDON, 0.00635+R_TENDON, 0.0091+R_TENDON],
#     [0.0029+R_TENDON, 0.0126+R_TENDON,  0.0091+R_TENDON],
#     [0.0064+R_TENDON, 0.0124+R_TENDON,  0.0]
# ])

# M = D * S

# def solve_tendon_tensions(tau_joint, min_tension=5.0):
#     """Maps required joint torques to physical, positive tendon tensions."""
#     res = lsq_linear(M.T, tau_joint, bounds=(min_tension, 100.0))
#     return res.x

# # ==============================================================================
# # TRAJECTORY GENERATION (Stroke-Based)
# # ==============================================================================
# LETTERS = {
#     'A': [(0, 0), (0.5, 1), (1, 0), (0.75, 0.5), (0.25, 0.5)],
#     'B': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5), (0.75, 0.5), (1, 0.25), (0.75, 0), (0, 0)],
#     'C': [(1, 1), (0.25, 1), (0, 0.75), (0, 0.25), (0.25, 0), (1, 0)],
#     'D': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (1, 0.25), (0.75, 0), (0, 0)],
#     'E': [(1, 1), (0, 1), (0, 0.5), (0.75, 0.5), (0, 0.5), (0, 0), (1, 0)],
#     'F': [(0, 0), (0, 1), (1, 1), (0, 1), (0, 0.5), (0.75, 0.5)],
#     'G': [(1, 1), (0.25, 1), (0, 0.75), (0, 0.25), (0.25, 0), (1, 0), (1, 0.5), (0.5, 0.5)],
#     'H': [(0, 1), (0, 0), (0, 0.5), (1, 0.5), (1, 1), (1, 0)],
#     'I': [(0.25, 1), (0.75, 1), (0.5, 1), (0.5, 0), (0.25, 0), (0.75, 0)],
#     'J': [(0, 0.5), (0.25, 0), (0.75, 0), (1, 0.25), (1, 1)],
#     'K': [(0, 1), (0, 0), (0, 0.5), (1, 1), (0, 0.5), (1, 0)],
#     'L': [(0, 1), (0, 0), (1, 0)],
#     'M': [(0, 0), (0, 1), (0.5, 0.5), (1, 1), (1, 0)],
#     'N': [(0, 0), (0, 1), (1, 0), (1, 1)],
#     'O': [(0.5, 1), (0.2, 0.8), (0, 0.5), (0.2, 0.2), (0.5, 0), (0.8, 0.2), (1, 0.5), (0.8, 0.8), (0.5, 1)],
#     'P': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5)],
#     'Q': [(0.8, 0.2), (1, 0.5), (0.8, 0.8), (0.5, 1), (0.2, 0.8), (0, 0.5), (0.2, 0.2), (0.5, 0), (0.8, 0.2), (0.5, 0.5), (1, 0)],
#     'R': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5), (0.5, 0.5), (1, 0)],
#     'S': [(1, 1), (0.25, 1), (0, 0.75), (0.25, 0.5), (0.75, 0.5), (1, 0.25), (0.75, 0), (0, 0)],
#     'T': [(0, 1), (1, 1), (0.5, 1), (0.5, 0)],
#     'U': [(0, 1), (0, 0.25), (0.25, 0), (0.75, 0), (1, 0.25), (1, 1)],
#     'V': [(0, 1), (0.5, 0), (1, 1)],
#     'W': [(0, 1), (0.25, 0), (0.5, 0.5), (0.75, 0), (1, 1)],
#     'X': [(0, 1), (1, 0), (0.5, 0.5), (0, 0), (1, 1)],
#     'Y': [(0, 1), (0.5, 0.5), (1, 1), (0.5, 0.5), (0.5, 0)],
#     'Z': [(0, 1), (1, 1), (0, 0), (1, 0)]
# }

# def generate_strokes(demo_type="write", word="RDS", image_path="", dt=0.01):
#     """Returns a list of continuous strokes. Transitions are handled dynamically in the main loop."""
#     strokes = []
#     scale = 0.02  
#     start_y = 0.06
#     base_x = 0.105
    
#     if demo_type == "write":
#         current_offset_y = start_y  
#         for letter in word:
#             pts = LETTERS[letter]
#             x_tot, y_tot, f_tot = [], [], []
#             for i in range(len(pts) - 1):
#                 p1, p2 = np.array(pts[i]), np.array(pts[i+1])
#                 dist = np.linalg.norm(p2 - p1)
#                 steps = max(2, int((dist * 1.5) / dt))
                
#                 y_tot.extend(current_offset_y - np.linspace(p1[0], p2[0], steps) * scale)
#                 x_tot.extend(base_x + np.linspace(p1[1], p2[1], steps) * scale)
#                 f_tot.extend(np.full(steps, 3.5)) 
            
#             strokes.append({'x': np.array(x_tot), 'y': np.array(y_tot), 'f': np.array(f_tot)})
#             current_offset_y -= 0.03 
            
#     elif demo_type == "shade":
#         num_lines = 150
#         y_step = 0.0002
#         x_min, x_max = base_x, base_x + scale
#         x_tot, y_tot, f_tot = [], [], []
        
#         for i in range(num_lines):
#             f_current = 4.0 - (3.8) * (i / max(1, num_lines - 1))
#             y_current = start_y - (i * y_step)
#             seg_time = 0.5 
#             steps = max(2, int(seg_time / dt))
#             x_start, x_end = (x_min, x_max) if i % 2 == 0 else (x_max, x_min)
                
#             y_tot.extend(np.full(steps, y_current))
#             x_tot.extend(np.linspace(x_start, x_end, steps))
#             f_tot.extend(np.full(steps, f_current))
            
#         strokes.append({'x': np.array(x_tot), 'y': np.array(y_tot), 'f': np.array(f_tot)})

#     elif demo_type == "image":
#         try:
#             # 1. Load image and convert to Grayscale ('L')
#             img = Image.open(image_path).convert('L')
#             img = img.rotate(-90, expand=True)
#             w, h = img.size
#             aspect_ratio = w / h
#             res_y = 100
#             res_x = max(2, int(res_y * aspect_ratio))
#             img = img.resize((res_x, res_y))
#             pixel_data = np.array(img)
            
#         except Exception as e:
#             print(f"\n[!] Error loading image '{image_path}': {e}")
#             print("[!] Falling back to default shading block.\n")
#             return generate_strokes("shade", dt=dt)
            
#         scale_y = 0.08  # Changed from 0.02 to 0.10 (10 cm tall)
        
#         # Slant & Aspect Ratio Correction
#         scale_x_paper = scale_y * aspect_ratio
#         pitch = np.radians(20.0) 
#         scale_x_horizontal = scale_x_paper * np.cos(pitch)
        
#         # Shift base_x back slightly so the larger image doesn't exceed the finger's max reach
#         safe_base_x = 0.095
        
#         x_min, x_max = safe_base_x, safe_base_x + scale_x_horizontal
#         # Start higher up so the 10cm image stays centered on the paper
#         y_min, y_max = 0.05, 0.05 - scale_y 
        
#         y_steps = np.linspace(y_min, y_max, res_y)
#         x_steps_forward = np.linspace(x_min, x_max, res_x)
#         x_steps_backward = np.linspace(x_max, x_min, res_x)
        
#         current_stroke_x, current_stroke_y, current_stroke_f = [], [], []
        
#         # 3. Create toolpath strokes
#         for i in range(res_y):
#             if i % 2 == 0:
#                 x_row = x_steps_forward
#                 row_pixels = pixel_data[i, :]
#             else:
#                 x_row = x_steps_backward
#                 row_pixels = pixel_data[i, ::-1] 
                
#             # Map Pixel Intensity to Normal Force
#             row_force = 4.0 * (1.0 - (row_pixels / 255.0))

#             for j in range(res_x):
#                 if row_force[j] >= 0.4:
#                     # Dark pixel -> Keep drawing the current stroke
#                     current_stroke_x.append(x_row[j])
#                     current_stroke_y.append(y_steps[i]) # Fixed: using 'i' for Y steps
#                     current_stroke_f.append(row_force[j])
#                 else:
#                     # Light pixel -> Lift the pencil
#                     if len(current_stroke_x) > 0:
#                         # Save the stroke we just finished
#                         strokes.append({
#                             'x': np.array(current_stroke_x), 
#                             'y': np.array(current_stroke_y), 
#                             'f': np.array(current_stroke_f)
#                         })
#                         # Clear the active arrays for the next stroke
#                         current_stroke_x, current_stroke_y, current_stroke_f = [], [], []

#         # Catch any remaining stroke at the very end of the image
#         if len(current_stroke_x) > 0:
#             strokes.append({
#                 'x': np.array(current_stroke_x), 
#                 'y': np.array(current_stroke_y), 
#                 'f': np.array(current_stroke_f)
#             })
            
#     return strokes

# # ==============================================================================
# # DRAKE SIMULATION & PLOTTING
# # ==============================================================================
# def build_diagram(meshcat):
#     builder = DiagramBuilder()
#     plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

#     parser = Parser(plant)
#     parser.package_map().Add("finger_assembly", PACKAGE_ROOT)
#     parser.AddModels(SDF_PATH)
    
#     plant.Finalize()
#     MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
#     return builder.Build(), plant, scene_graph

# def run_simulation(demo_type="write", word="RDS", image_path=""):
#     meshcat = StartMeshcat()
#     diagram, plant, scene_graph = build_diagram(meshcat)
    
#     ctx = diagram.CreateDefaultContext()
#     plant_ctx = plant.GetMyMutableContextFromRoot(ctx)
    
#     pencil_link = plant.GetBodyByName("pencil")
#     pencil_tip_local = [0, 0, 0.02]
#     world_frame = plant.world_frame()
    
#     # Auto-Align Paper
#     initial_tip_pos = plant.CalcPointsPositions(
#         plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame
#     ).flatten()

#     paper_z = initial_tip_pos[2] - 0.05
#     paper_thickness = 0.002

#     meshcat.SetObject(
#         "paper",
#         Box(0.20, 0.20, paper_thickness),
#         rgba=Rgba(0.95, 0.95, 0.95, 1.0)
#     )

#     R_paper = RollPitchYaw(0, np.radians(30), 0.0).ToRotationMatrix()
#     paper_origin = np.array([initial_tip_pos[0] - 0.02, initial_tip_pos[1], paper_z + paper_thickness/2])
#     meshcat.SetTransform("paper", RigidTransform(R_paper, paper_origin))
#     paper_normal = R_paper.multiply([0, 0, 1])
    
#     surface_origin = paper_origin + paper_normal * (paper_thickness / 2.0)
#     strokes = generate_strokes(demo_type, word, image_path)
    
#     dt = 0.01
#     curr_t = 0.0
#     log_t, log_y, log_x, log_f, log_q, log_tensions = [], [], [], [], [], []
    
#     print(f"\n---> Open Meshcat to watch the '{demo_type}' simulation: {meshcat.web_url()} <---")
#     meshcat.StartRecording()

#     def solve_ik(target_pos, q_guess):
#         MAX_ITERATIONS = 50
#         TOLERANCE = 1e-4  
#         LEARNING_RATE = 0.5 
#         q = q_guess.copy()
        
#         for _ in range(MAX_ITERATIONS):
#             plant.SetPositions(plant_ctx, q)
#             current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
#             err = target_pos - current_pos
#             if np.linalg.norm(err) < TOLERANCE: break
                
#             J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
#             dq = J.T @ np.linalg.inv(J @ J.T + 1e-4*np.eye(3)) @ err
#             q += dq * LEARNING_RATE
            
#             q[1] = np.clip(q[1], np.radians(-10), np.radians(10)) 
#             q[2] = np.clip(q[2], 0.0, np.radians(90))             
#             q[3] = np.clip(q[3], 0.0, np.radians(90))             
#             q[4] = np.clip(q[3] * DIP_COUPLING_RATIO, 0.0, np.radians(90))             
#         return q

#     def interpolate_joints(q_start, q_end, duration):
#         nonlocal curr_t
#         steps = max(2, int(duration / dt))
#         q = q_start.copy()
#         for i in range(steps):
#             alpha = i / (steps - 1)
#             q = q_start + alpha * (q_end - q_start)
#             q[4] = q[3] * DIP_COUPLING_RATIO
            
#             plant.SetPositions(plant_ctx, q)
#             current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
            
#             tensions = solve_tendon_tensions(np.zeros(3)) # Min tensions in the air
            
#             log_t.append(curr_t)
#             log_y.append(current_pos[1])
#             log_x.append(current_pos[0])
#             log_f.append(0.0) # 0 force when hovering
#             log_q.append(q.copy())
#             log_tensions.append(tensions)
            
#             ctx.SetTime(curr_t)
#             diagram.ForcedPublish(ctx)
#             curr_t += dt
#         return q

#     q_current = plant.GetPositions(plant_ctx).copy()

#     for idx, stroke in enumerate(strokes):
#         # ==========================================
#         # JOINT-SPACE TRANSITION (Lift & Move)
#         # ==========================================
#         if idx > 0:
#             tx, ty = stroke['x'][0], stroke['y'][0]
#             dz_slope = -(paper_normal[0]*(tx - surface_origin[0]) + paper_normal[1]*(ty - surface_origin[1])) / paper_normal[2]
#             next_start_pos = np.array([tx, ty, surface_origin[2] + dz_slope])
            
#             # Silently find where we need to end up
#             q_next = solve_ik(next_start_pos, q_current)

#             # 1. Extend MCP fully
#             q_ext1 = q_current.copy()
#             q_ext1[2] = 0.0
#             q_current = interpolate_joints(q_current, q_ext1, duration=0.1)

#             # 2. Extend PIP (not fully)
#             q_ext2 = q_current.copy()
#             q_ext2[3] = np.radians(40.0)
#             q_current = interpolate_joints(q_current, q_ext2, duration=0.1)

#             # 3. Move Slider & Splay to new position
#             q_move = q_current.copy()
#             q_move[0] = q_next[0]
#             q_move[1] = q_next[1]
#             q_current = interpolate_joints(q_current, q_move, duration=0.2)

#             # 4. Drop to exactly the next start position
#             q_current = interpolate_joints(q_current, q_next, duration=0.15)
        
#         # ==========================================
#         # DRAWING STROKE (Strict Surface IK)
#         # ==========================================
#         stroke_points = []

#         for i in range(len(stroke['x'])):
#             # Track surface height EXACTLY
#             tx, ty = stroke['x'][i], stroke['y'][i]
#             dz_slope = -(paper_normal[0]*(tx - surface_origin[0]) + paper_normal[1]*(ty - surface_origin[1])) / paper_normal[2]
#             target_pos = np.array([tx, ty, surface_origin[2] + dz_slope])
            
#             q_current = solve_ik(target_pos, q_current)
#             plant.SetPositions(plant_ctx, q_current)
            
#             current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()

#             # --- DRAW IN MESHCAT ---
#             # Lift the drawing 0.1 mm above the paper to avoid Z-fighting glitches
#             # --- DRAW IN MESHCAT ---
#             draw_pos = current_pos + (paper_normal * 0.0001)
#             stroke_points.append(draw_pos)
            
#             if len(stroke_points) > 1:
#                 intensity = np.clip(float(actual_normal_force), 0.0, 4.0)
#                 gray = 1.00 - (intensity * 0.25) 
                
#                 path = f"drawing/stroke_{idx}/seg_{i}"
#                 vertices = np.array([stroke_points[-2], stroke_points[-1]]).T
                
#                 # Draw the line segment
#                 meshcat.SetLine(path, vertices, 3.0, Rgba(gray, gray, gray, 1.0))
                
#                 # Animate visibility: Hide at t=0, Show at t=curr_t
#                 meshcat.SetProperty(path, "visible", False, 0.0)
#                 meshcat.SetProperty(path, "visible", True, curr_t)
            
#             J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
#             J_finger = J[:, 1:4] 
            
#             # Kinematic Force Extraction
#             f_intent = -paper_normal * stroke['f'][i] 
#             tau_req = J_finger.T @ f_intent 
            
#             tensions = solve_tendon_tensions(tau_req)
#             tau_actual = M.T @ tensions
#             F_actual = np.linalg.pinv(J_finger.T) @ tau_actual
#             actual_normal_force = np.dot(F_actual, -paper_normal) # Project force onto negative normal
            
#             log_t.append(curr_t)
#             log_y.append(current_pos[1])
#             log_x.append(current_pos[0])
#             log_f.append(actual_normal_force)
#             log_q.append(q_current.copy())
#             log_tensions.append(tensions)

#             ctx.SetTime(curr_t)
#             diagram.ForcedPublish(ctx)
#             curr_t += dt

#     meshcat.PublishRecording()

#     return np.array(log_t), np.array(log_y), np.array(log_x), np.array(log_f), np.array(log_q), np.array(log_tensions)

# def plot_results(t, x, y, f, q, tensions, title):
#     fig = plt.figure(figsize=(14, 10))
#     fig.canvas.manager.set_window_title(title)
    
#     # 1. 2D Drawing Result with Shading
#     draw_mask = f > 0.01
#     ax1 = plt.subplot(2, 2, 1)
#     sc = ax1.scatter(x[draw_mask], y[draw_mask], c=f[draw_mask], cmap='Greys', s=20, edgecolor='none', vmin=0, vmax=np.max(f)+0.5)
#     plt.colorbar(sc, ax=ax1, label='Normal Force (N)')
#     ax1.set_title(f"Drawing Canvas (X-Y Plane)")
#     ax1.set_xlabel("Slider / X Coordinate")
#     ax1.set_ylabel("Finger / Y Coordinate")
#     ax1.axis('equal')
#     ax1.grid(True, alpha=0.3)

#     # 2. Pencil Normal Force Over Time
#     ax2 = plt.subplot(2, 2, 2)
#     ax2.plot(t, f, color='crimson', linewidth=2)
#     ax2.set_title("Kinematic Pencil Tip Normal Force")
#     ax2.set_xlabel("Time (s)")
#     ax2.set_ylabel("Force (N)")
#     ax2.grid(True, alpha=0.3)

#     # 3. Joint Kinematics
#     ax3 = plt.subplot(2, 2, 3)
#     ax3.plot(t, q[:, 0]*100, label='Slider (cm)', linestyle='--')
#     ax3.plot(t, np.degrees(q[:, 1]), label='Splay (deg)')
#     ax3.plot(t, np.degrees(q[:, 2]), label='MCP (deg)')
#     ax3.plot(t, np.degrees(q[:, 3]), label='PIP (deg)')
#     ax3.plot(t, np.degrees(q[:, 4]), label='DIP (deg)')
#     ax3.set_title("Joint Kinematics")
#     ax3.set_xlabel("Time (s)")
#     ax3.legend()
#     ax3.grid(True, alpha=0.3)

#     # 4. Tendon Tensions
#     ax4 = plt.subplot(2, 2, 4)
#     labels = ['T0 (Splay)', 'T1 (MCP Ext)', 'T2 (PIP Flex)', 'T3 (PIP Ext)', 'T4 (MCP Flex)']
#     for i in range(5):
#         ax4.plot(t, tensions[:, i], label=labels[i])
#     ax4.set_title("Required Tendon Tensions")
#     ax4.set_xlabel("Time (s)")
#     ax4.set_ylabel("Tension (N)")
#     ax4.legend()
#     ax4.grid(True, alpha=0.3)

#     plt.tight_layout()
#     plt.show()

# if __name__ == "__main__":
#     # word = input("Please enter a word to write: ")
#     # t_w, y_w, x_w, f_w, q_w, ten_w = run_simulation("write", word)
#     # plot_results(t_w, -y_w, x_w, f_w, q_w, ten_w, f"Writing '{word}'")
    
#     image_path = "apple.jpeg"
#     t_i, y_i, x_i, f_i, q_i, ten_i = run_simulation("image", image_path=image_path)
#     plot_results(t_i, -y_i, x_i, f_i, q_i, ten_i, f"Drawing Image: {image_path}")
    
#     input("Press Enter to close Meshcat and exit...")

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

PACKAGE_ROOT = "../models"
SDF_PATH_1D  = PACKAGE_ROOT + "/urdf/finger_sliding.sdf"
SDF_PATH_2D  = PACKAGE_ROOT + "/urdf/finger_planar.sdf" # <--- NEW SDF PATH

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
    res = lsq_linear(M.T, tau_joint, bounds=(min_tension, 100.0))
    return res.x

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
    
    plant.Finalize()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build(), plant, scene_graph

def run_simulation(demo_type="write", word="RDS", image=""):
    is_planar = (demo_type == "image")
    idx_offset = 1 if is_planar else 0 # 1 if we have a Y slider, 0 if just X
    
    meshcat = StartMeshcat()
    diagram, plant, scene_graph = build_diagram(meshcat, is_planar)
    
    ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(ctx)
    
    pencil_link = plant.GetBodyByName("pencil")
    pencil_tip_local = [0, 0, 0.02]
    world_frame = plant.world_frame()
    
    initial_tip_pos = plant.CalcPointsPositions(
        plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame
    ).flatten()

    paper_z = initial_tip_pos[2] - 0.05
    paper_thickness = 0.002

    # Use a bigger piece of paper for drawings!
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
    log_t, log_y, log_x, log_f, log_q, log_tensions = [], [], [], [], [], []
    
    print(f"\n---> Open Meshcat to watch the '{demo_type}' simulation: {meshcat.web_url()} <---")
    meshcat.StartRecording()

    # DYNAMIC JOINT INDICES
    idx_y = 0 if is_planar else None
    idx_x = 1 if is_planar else 0
    idx_splay = 1 + idx_offset
    idx_mcp = 2 + idx_offset
    idx_pip = 3 + idx_offset
    idx_dip = 4 + idx_offset

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

    def interpolate_joints(q_start, q_end, duration):
        nonlocal curr_t
        steps = max(2, int(duration / dt))
        q = q_start.copy()
        for i in range(steps):
            alpha = i / (steps - 1)
            q = q_start + alpha * (q_end - q_start)
            q[idx_dip] = q[idx_pip] * DIP_COUPLING_RATIO
            
            plant.SetPositions(plant_ctx, q)
            current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
            
            tensions = solve_tendon_tensions(np.zeros(3))
            
            log_t.append(curr_t)
            log_y.append(current_pos[1])
            log_x.append(current_pos[0])
            log_f.append(0.0)
            log_q.append(q.copy())
            log_tensions.append(tensions)
            
            ctx.SetTime(curr_t)
            diagram.ForcedPublish(ctx)
            curr_t += dt
        return q

    q_current = plant.GetPositions(plant_ctx).copy()

    for idx, stroke in enumerate(strokes):
        # LIFT & MOVE TRANSITION
        if idx > 0:
            tx, ty = stroke['x'][0], stroke['y'][0]
            dz_slope = -(paper_normal[0]*(tx - surface_origin[0]) + paper_normal[1]*(ty - surface_origin[1])) / paper_normal[2]
            next_start_pos = np.array([tx, ty, surface_origin[2] + dz_slope])
            
            q_next = solve_ik(next_start_pos, q_current)

            q_ext1 = q_current.copy()
            q_ext1[idx_mcp] = 0.0
            q_current = interpolate_joints(q_current, q_ext1, duration=0.1)

            q_ext2 = q_current.copy()
            q_ext2[idx_pip] = np.radians(40.0)
            q_current = interpolate_joints(q_current, q_ext2, duration=0.1)

            q_move = q_current.copy()
            if is_planar: q_move[idx_y] = q_next[idx_y]
            q_move[idx_x] = q_next[idx_x]
            q_move[idx_splay] = q_next[idx_splay]
            q_current = interpolate_joints(q_current, q_move, duration=0.2)

            q_current = interpolate_joints(q_current, q_next, duration=0.15)
        
        # DRAWING STROKE
        stroke_points = [] 
        
        for i in range(len(stroke['x'])):
            tx, ty = stroke['x'][i], stroke['y'][i]
            dz_slope = -(paper_normal[0]*(tx - surface_origin[0]) + paper_normal[1]*(ty - surface_origin[1])) / paper_normal[2]
            target_pos = np.array([tx, ty, surface_origin[2] + dz_slope])
            
            q_current = solve_ik(target_pos, q_current)
            plant.SetPositions(plant_ctx, q_current)
            
            current_pos = plant.CalcPointsPositions(plant_ctx, pencil_link.body_frame(), pencil_tip_local, world_frame).flatten()
            J = plant.CalcJacobianTranslationalVelocity(plant_ctx, JacobianWrtVariable.kV, pencil_link.body_frame(), pencil_tip_local, world_frame, world_frame)
            
            # Select only Splay, MCP, and PIP joints for force math
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
            
            log_t.append(curr_t)
            log_y.append(current_pos[1])
            log_x.append(current_pos[0])
            log_f.append(actual_normal_force)
            log_q.append(q_current.copy())
            log_tensions.append(tensions)

            ctx.SetTime(curr_t)
            diagram.ForcedPublish(ctx)
            curr_t += dt

    meshcat.PublishRecording()

    return np.array(log_t), np.array(log_y), np.array(log_x), np.array(log_f), np.array(log_q), np.array(log_tensions)

def plot_results(t, x, y, f, q, tensions, title):
    is_planar = q.shape[1] == 6
    idx_offset = 1 if is_planar else 0

    fig = plt.figure(figsize=(14, 10))
    fig.canvas.manager.set_window_title(title)
    
    draw_mask = f > 0.01
    
    ax1 = plt.subplot(2, 2, 1)
    sc = ax1.scatter(x[draw_mask], y[draw_mask], c=f[draw_mask], cmap='Greys', s=20, edgecolor='none', vmin=0, vmax=np.max(f)+0.5)
    plt.colorbar(sc, ax=ax1, label='Normal Force (N)')
    ax1.set_title(f"Drawing Canvas (X-Y Plane)")
    ax1.set_xlabel("World X Coordinate")
    ax1.set_ylabel("World Y Coordinate")
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(t, f, color='crimson', linewidth=2)
    ax2.set_title("Kinematic Pencil Tip Normal Force")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (N)")
    ax2.grid(True, alpha=0.3)

    ax3 = plt.subplot(2, 2, 3)
    if is_planar:
        ax3.plot(t, q[:, 1]*100, label='Slider X (cm)', linestyle='--')
        ax3.plot(t, q[:, 0]*100, label='Slider Y (cm)', linestyle='--')
    else:
        ax3.plot(t, q[:, 0]*100, label='Slider X (cm)', linestyle='--')
        
    ax3.plot(t, np.degrees(q[:, 1 + idx_offset]), label='Splay (deg)')
    ax3.plot(t, np.degrees(q[:, 2 + idx_offset]), label='MCP (deg)')
    ax3.plot(t, np.degrees(q[:, 3 + idx_offset]), label='PIP (deg)')
    ax3.plot(t, np.degrees(q[:, 4 + idx_offset]), label='DIP (deg)')
    ax3.set_title("Joint Kinematics")
    ax3.set_xlabel("Time (s)")
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
    word = input("Please enter a word to write: ")
    t_w, y_w, x_w, f_w, q_w, ten_w = run_simulation("write", word=word)
    plot_results(t_w, -y_w, x_w, f_w, q_w, ten_w, f"Writing '{word}'")
    
    t_s, y_s, x_s, f_s, q_s, ten_s = run_simulation("shade")
    plot_results(t_s, -y_s, x_s, f_s, q_s, ten_s, "Variable Force Shading")
    
    img_path = input("Enter the file path of your image: ").strip()
    t_i, y_i, x_i, f_i, q_i, ten_i = run_simulation("image", image=img_path)
    plot_results(t_i, -y_i, x_i, f_i, q_i, ten_i, f"Drawing Image: {img_path}")
        
    input("Press Enter to close Meshcat and exit...")