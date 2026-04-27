
import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph,
    Parser, Simulator, MeshcatVisualizer, StartMeshcat,
    ConstantVectorSource, LogVectorOutput,
    CoulombFriction, Box, RigidTransform,
)
import logging

from tendon_controller import FingerTendonController
from static_solver import solve_tendon_forces, forces_to_lslack

class _FilterPTC(logging.Filter):
    def filter(self, record):
        return "PTC_onshape_metadata" not in record.getMessage()
logging.getLogger("drake").addFilter(_FilterPTC())

PACKAGE_ROOT = "/home/zach/472/finger_drake_sim/models"
SDF_PATH     = PACKAGE_ROOT + "/urdf/finger.sdf"

TENDON_STIFFNESS = 5000.0
SIM_TIME         = 8.0
REALTIME_RATE    = 1.0


SPLAY_KP = 5.0   
SPLAY_KD = 0.1    
SPLAY_KI = 2.0    
SPLAY_F0 = 8.0

Q0 = {
    "mcp_splay":   np.radians(-10.0),
    "mcp_flexion": 0.0,
    "pip":         0.0,
    "dip":         0.0,
}


def build_diagram(meshcat):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    parser = Parser(plant)
    parser.package_map().Add("finger_assembly", PACKAGE_ROOT)
    parser.AddModels(SDF_PATH)
    plant.set_adjacent_bodies_collision_filters(True)

    plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransform([0, 0, -0.05]),
        Box(1.0, 1.0, 0.05), "ground_col", CoulombFriction(0.9, 0.8))
    plant.RegisterVisualGeometry(
        plant.world_body(), RigidTransform([0, 0, -0.05]),
        Box(1.0, 1.0, 0.05), "ground_vis", [0.8, 0.8, 0.8, 1.0])

    plant.RegisterVisualGeometry(
        plant.world_body(), RigidTransform([-0.01, 0, 0.15]),
        Box(0.02, 0.20, 0.40), "wall_vis", [1.0, 1.0, 1.0, 1.0])
    plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransform([-0.01, 0, 0.15]),
        Box(0.02, 0.20, 0.40), "wall_col", CoulombFriction(0.9, 0.8))

    plant.Finalize()

    dummy_cmds = np.zeros(6)
    ctrl = builder.AddSystem(
        FingerTendonController(plant, tendon_stiffness=TENDON_STIFFNESS))
    ctrl.set_name("tendon_ctrl")

    motor_src = builder.AddSystem(ConstantVectorSource(dummy_cmds))

    builder.Connect(plant.get_state_output_port(),
                    ctrl.GetInputPort("plant_state"))
    builder.Connect(motor_src.get_output_port(),
                    ctrl.GetInputPort("motor_commands"))
    builder.Connect(ctrl.GetOutputPort("generalized_forces"),
                    plant.get_applied_generalized_force_input_port())

    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    return diagram, plant, ctrl, logger


def compute_splay_lslacks(ctrl, theta_splay, tau_pd):
    r     = ctrl._tendons[4]["pulley_r"]
    k     = ctrl._k
    t_ref = ctrl._tendons[4]["theta_ref"]

    delta_F = np.clip(tau_pd / (2 * r), -SPLAY_F0 * 0.9, SPLAY_F0 * 0.9)

    L_s1 = (ctrl._tendons[4]["L_ref"]
            - ctrl._tendons[4]["sign"] * r_s * (theta_splay - t_ref))
    L_s2 = (ctrl._tendons[5]["L_ref"]
            - ctrl._tendons[5]["sign"] * r_s * (theta_splay - t_ref))

    ls1 = max(0.001, L_s1 - (SPLAY_F0 + delta_F) / k)
    ls2 = max(0.001, L_s2 - (SPLAY_F0 - delta_F) / k)
    return ls1, ls2


def run_simulation():
    meshcat = StartMeshcat()
    print(f"Meshcat: {meshcat.web_url()}")

    diagram, plant, ctrl, logger = build_diagram(meshcat)

    simulator = Simulator(diagram)
    ctx = simulator.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(ctx)

    for name, angle in Q0.items():
        plant.GetJointByName(name).set_angle(plant_ctx, angle)

    q0_vec = plant.GetPositions(plant_ctx)
    tau_g0 = plant.CalcGravityGeneralizedForces(plant_ctx)
    print(f"\n重力力矩 (q=0): {np.round(tau_g0, 4)}")
    print(f"splay 重力力矩 tau_g[0] = {tau_g0[0]:.6f} Nm") 
    F0 = solve_tendon_forces(tau_g0, ctrl._tendons)
    L0 = forces_to_lslack(F0, ctrl._tendons, q0_vec, ctrl._k)

    print(f"初始张力: {[f'{f:.3f}N' for f in F0]}")
    print(f"初始 L_slack: {[f'{l*1000:.2f}mm' for l in L0]}")
    ctrl.set_motor_commands(L0)

    meshcat.StartRecording()
    simulator.set_target_realtime_rate(REALTIME_RATE)
    simulator.Initialize()

    splay_integral = 0.0
    dt_log = 0.001
    t_log, q_log, tension_log = [], [], []

    for t in np.arange(0, SIM_TIME, dt_log):
        simulator.AdvanceTo(t + dt_log)

        pc = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
        

        q  = plant.GetPositions(pc)
        qv = plant.GetVelocities(pc)

        
        # freq = 2.0 * np.pi / 4.0
        
        # target_splay=0.0
        # target_mcp = 0.5 * 1.0 * (1.0 - np.cos(freq * t))
        # target_pip = 0.5 * 1.0 * (1.0 - np.cos(freq * t))
    
        t_pts = [0.0, 1.5, 2.0, 4.0, 4.5, 6.0, 8.0]
        
        s_max = np.radians(10.0)
        
        # Splay 轨迹：平滑右扫 -> 停顿 -> 平滑左扫 -> 停顿 -> 平滑右扫 -> 停顿到结束
        splay_pts = [-s_max, s_max, s_max, -s_max, -s_max, s_max, s_max]
        
        # 弯曲轨迹： 保持平展 -> 平展 -> 缓慢弯曲 -> 弯曲 -> 保持弯曲 -> 弯曲到结束
        flex_pts  = [0.0,    0.0,   0.0,   0.8,    0.8,    0.8,   0.8]
        
        target_splay = np.interp(t, t_pts, splay_pts)
        target_mcp   = 0.5 * np.interp(t, t_pts, flex_pts)
        target_pip   = 0.6 * np.interp(t, t_pts, flex_pts)
  

        # 2. 关节空间 PD 控制器 
        Kp = 0.1    
        Kd = 0.005  
        
        tau_pd = np.zeros_like(q)
        tau_pd[1] = Kp * (target_mcp - q[1]) - Kd * qv[1]
        tau_pd[2] = Kp * (target_pip - q[2]) - Kd * qv[2]
        
        tau_g = plant.CalcGravityGeneralizedForces(pc)
        
        # 3. 输入给 solver：重力 - PD力矩
        input_to_solver = tau_g - tau_pd
        F_sol = solve_tendon_forces(input_to_solver, ctrl._tendons)
        L_slacks = forces_to_lslack(F_sol, ctrl._tendons, q, ctrl._k)

        # Splay：solver 结果作为前馈，PID 作为反馈修正
        theta_splay     = q[0]
        v_splay         = qv[0]
        splay_err       = target_splay - theta_splay
        splay_integral += splay_err * dt_log
        splay_integral  = np.clip(splay_integral, -0.05, 0.05)
        tau_pd_corr = np.clip(SPLAY_KP * splay_err
                              - SPLAY_KD * v_splay
                              + SPLAY_KI * splay_integral, -0.3, 0.3)

        # 叠加 PID 修正
        r_s     = ctrl._tendons[4]["pulley_r"]
        t_ref   = ctrl._tendons[4]["theta_ref"]
        delta_F = tau_pd_corr / (2 * r_s)

        F_s1 = max(0.5, SPLAY_F0 + delta_F)
        F_s2 = max(0.5, SPLAY_F0 - delta_F)

        L_s1 = (ctrl._tendons[4]["L_ref"]
                - ctrl._tendons[4]["sign"] * r_s * (theta_splay - t_ref))
        L_s2 = (ctrl._tendons[5]["L_ref"]
                 - ctrl._tendons[5]["sign"] * r_s * (theta_splay - t_ref))

        L_slacks[4] = max(0.001, L_s1 - F_s1 / ctrl._k)
        L_slacks[5] = max(0.001, L_s2 - F_s2 / ctrl._k)

        ctrl.set_motor_commands(L_slacks)

        # Logging
        ctrl._plant.SetPositions(ctrl._plant_ctx, q)
        tensions = {}
        for i, t_def in enumerate(ctrl._tendons):
            theta = ctrl._plant.GetJointByName(
                t_def["joint"]).get_angle(ctrl._plant_ctx)
            L  = (t_def["L_ref"]
                   - t_def["sign"] * t_def["pulley_r"]
                  * (theta - t_def["theta_ref"]))
            tensions[t_def["name"]] = max(0.0, ctrl._k * (L - L_slacks[i]))

        t_log.append(t)
        q_log.append(q.copy())
        tension_log.append([tensions[n] for n in
                            ["mcp_flexor", "mcp_extensor", "pip_flexor",
                             "pip_extensor", "splay_1", "splay_2"]])

        print(f"t={t:.3f}s  "
              f"splay={np.degrees(q[0]):+5.1f}  "
              f"mcp={np.degrees(q[1]):+5.1f}  "
              f"pip={np.degrees(q[2]):+5.1f}  "
              f"dip={np.degrees(q[3]):+5.1f}")

    meshcat.PublishRecording()

    t_log       = np.array(t_log)
    q_log       = np.array(q_log)
    tension_log = np.array(tension_log)
    tendon_names = ["mcp_flexor", "mcp_extensor", "pip_flexor",
                    "pip_extensor", "splay_1", "splay_2"]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    ax = axes[0]
    for i, label in enumerate(["splay", "mcp_flexion", "pip", "dip"]):
        ax.plot(t_log, np.degrees(q_log[:, i]), label=label)
    ax.set(xlabel="Time (s)", ylabel="Angle (deg)", title="Joint Angles")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = axes[1]
    for i, name in enumerate(tendon_names):
        ax.plot(t_log, tension_log[:, i], label=name)
    ax.set(xlabel="Time (s)", ylabel="Tension (N)", title="Tendon Tensions")
    ax.legend()
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("tendon_sim_results.png", dpi=150)
    plt.show(block=False)
    plt.pause(60)


if __name__ == "__main__":
    run_simulation()
