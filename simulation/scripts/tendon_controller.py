"""
tendon_controller.py  —  Surgeon Team, ME 472 Robot Design Studio 2026

Tendon controller for the 4-DOF robotic finger.
Uses Drake kinematics (EvalBodyPoseInWorld + CalcJacobianTranslationalVelocity)
to correctly handle multi-DOF coupling between joints.

Active tendons (5 + 1 extra splay):
  T0  MCP flexor   (yellow)  — terminate on proximal, pulley on mcp_link
  T1  MCP extensor (purple)  — terminate on proximal, pulley on mcp_link
  T2  PIP flexor   (green)   — terminate on middle,   pulley on proximal
  T3  PIP extensor (blue)    — terminate on middle,   pulley on proximal
  T4a Splay 1      (black)   — terminate on mcp_link, pulley on base
  T4b Splay 2      (black)   — terminate on mcp_link, pulley on base (same motor)

DIP-PIP coupling: enforced as q_dip = ALPHA * q_pip (rigid constraint),
simulated by setting DIP angle each step outside this controller.

Coordinate conversion (OnShape sub-assembly local → Drake link frame):
  Proximal (finger dir +Z): x_link = y_ons/1e3,  y_link = -x_ons/1e3, z_link = z_ons/1e3
  Middle   (finger dir -X): x_link = -z_ons/1e3, y_link = -y_ons/1e3, z_link = -x_ons/1e3
  MCP Asm  (finger dir +Y): x_link = x_ons/1e3,  y_link = -z_ons/1e3, z_link = y_ons/1e3
"""

import numpy as np
from pydrake.all import LeafSystem, BasicVector, JacobianWrtVariable


# ─────────────────────────────────────────────────────────────────────────────
# DIP-PIP coupling ratio (DIP = ALPHA * PIP)
# Adjust based on actual mechanical coupling geometry
ALPHA_DIP_PIP = 2/3
# ─────────────────────────────────────────────────────────────────────────────



class FingerTendonController(LeafSystem):
    """
    Tendon force model for the Surgeon finger.

    Ports:
      Input  "plant_state"    — full plant state [q; v]
      Input  "motor_commands" — L_slack per tendon (6 values)
                                set to 0 to use natural slack (passive)
      Output "generalized_forces" — joint torques

    Physics:
      1. For each tendon, compute world-frame terminate point A(q)
         using Drake FK (EvalBodyPoseInWorld).
      2. Compute world-frame pulley position P(q) — the idler pulley
         can also move if it is fixed to a moving link.
      3. Approximate tangent point T = P + r * (A-P)/|A-P|.
      4. Tendon length L = |A - T|.
      5. Tension F = k * max(0, L - L_slack).
      6. Joint torques via Jacobian:  τ = -F * (unit_v · J)
         where J = CalcJacobianTranslationalVelocity of terminate point.
    """

    def __init__(self, plant, tendon_stiffness: float = 1000.0):
        """
        Args:
            plant: MultibodyPlant (must be Finalize()'d)
            tendon_stiffness: k [N/m], spring constant for all tendons
        """
        super().__init__()
        self._plant = plant
        self._plant_ctx = plant.CreateDefaultContext()
        self._k = tendon_stiffness
        self._motor_cmds_override = None   # None = 用 input port；set_motor_commands() 设置后走 override

        # ── Body references ───────────────────────────────────────────────
        self._body = {
            name: plant.GetBodyByName(name)
            for name in ["base", "mcp_link", "proximal", "middle", "distal"]
        }

        # ── Tendon geometry ───────────────────────────────────────────────
        # All positions in meters, expressed in the body's Drake link frame.
        # Pulley positions are in the body frame of the link the pulley is
        # FIXED TO (not the link the terminate is on).
        #
        # Coordinate conversions applied from OnShape measurements:
        #
        #   Proximal (finger +Z in OnShape):
        #     x_link = y_ons/1e3,  y_link = -x_ons/1e3, z_link = z_ons/1e3
        #
        #   Middle (finger -X in OnShape):
        #     x_link = -z_ons/1e3, y_link = -y_ons/1e3, z_link = -x_ons/1e3
        #
        #   MCP Assembly (finger +Y in OnShape):
        #     x_link = x_ons/1e3,  y_link = -z_ons/1e3, z_link = y_ons/1e3
        #
        # Pulley world position at q=0 verified against link world positions:
        #   mcp_link:  z=0.026 m
        #   proximal:  z=0.050 m
        #   middle:    z=0.094 m

        self._tendons = [
            {
                "name":        "mcp_flexor",
                "joint":       "mcp_flexion",   # ← 新增
                "sign":        +1,              # ← 新增：关节角增大时此腱变紧
                "pulley_r":    0.0126,
                "L_slack":     None,
                # 其余字段保留不动
                "term_body":   "proximal",
                "term_local":  np.array([ 0.0069, -0.0086,  0.013905]),
                "pulley_body": "mcp_link",
                "pulley_local":np.array([ 0.0069,  0.0,     0.024   ]),
            },
            {
                "name":        "mcp_extensor",
                "joint":       "mcp_flexion",
                "sign":        -1,              # ← 关节角减小时此腱变紧
                "pulley_r":    0.0091,
                "L_slack":     None,
                "term_body":   "proximal",
                "term_local":  np.array([-0.0069, -0.0086,  0.013905]),
                "pulley_body": "mcp_link",
                "pulley_local":np.array([-0.0069,  0.0,     0.024   ]),
            },
            {
                "name":        "pip_flexor",
                "joint":       "pip",
                "sign":        +1,
                "pulley_r":    0.0091,
                "L_slack":     None,
                "term_body":   "middle",
                "term_local":  np.array([-0.0034, -0.008838,  0.00884]),
                "pulley_body": "proximal",
                "pulley_local":np.array([-0.0034,  0.0,       0.044  ]),
            },
            {
                "name":        "pip_extensor",
                "joint":       "pip",
                "sign":        -1,
                "pulley_r":    0.0091,
                "L_slack":     None,
                "term_body":   "middle",
                "term_local":  np.array([ 0.0034, -0.008838,  0.00884]),
                "pulley_body": "proximal",
                "pulley_local":np.array([ 0.0034,  0.0,       0.044  ]),
            },
            {
                "name":        "splay_1",
                "joint":       "mcp_splay",
                "sign":        +1,
                "pulley_r":    0.00925,
                "L_slack":     None,
                "term_body":   "mcp_link",
                "term_local":  np.array([-0.002084, -0.0115, -0.011818]),
                "pulley_body": "base",
                "pulley_local":np.array([ 0.0,       0.0,     0.026   ]),
            },
            {
                "name":        "splay_2",
                "joint":       "mcp_splay",
                "sign":        -1,
                "pulley_r":    0.00925,
                "L_slack":     None,
                "term_body":   "mcp_link",
                "term_local":  np.array([ 0.002084, -0.0115, -0.011818]),
                "pulley_body": "base",
                "pulley_local":np.array([ 0.0,       0.0,     0.026   ]),
            },
        ]

        N = len(self._tendons)

        # ── Drake ports ───────────────────────────────────────────────────
        self._state_port = self.DeclareVectorInputPort(
            "plant_state", plant.num_multibody_states())
        self._cmd_port = self.DeclareVectorInputPort(
            "motor_commands", N)

        self.DeclareVectorOutputPort(
            "generalized_forces", plant.num_velocities(),
            self.CalcTorques)

        # Compute natural slack lengths at q=0
        self._init_slack_lengths()

    # ── Internal helpers ──────────────────────────────────────────────────

    def _init_slack_lengths(self):
        """记录 q=0 时的参考角度，设置预张力 slack。"""
        print("Initializing tendon slack lengths (q=0):")
        for t in self._tendons:
            θ_ref = self._plant.GetJointByName(t["joint"]).get_angle(self._plant_ctx)
            t["theta_ref"] = θ_ref
            # L_ref = 一个名义长度（比如 pulley 周长的一部分，随便设个合理值）
            # 预张力：L_slack = L_ref * 0.9，即 10% 预张力
            # 这里直接用 pulley 半径 * π/2 作为参考长度（quarter wrap）
            L_ref = t["pulley_r"] * np.pi / 2
            t["L_ref"] = L_ref
            t["L_slack"] = 0.90 * L_ref
            print(f"  {t['name']:16s}: r={t['pulley_r']*1e3:.1f}mm  "
                f"L_ref={L_ref*1e3:.2f}mm  L_slack={t['L_slack']*1e3:.2f}mm")

    def _update_plant_ctx(self, context):
        """Copy current state into the internal plant context."""
        state = self._state_port.Eval(context)
        n_q = self._plant.num_positions()
        self._plant.SetPositions( self._plant_ctx, state[:n_q])
        self._plant.SetVelocities(self._plant_ctx, state[n_q:])

    def _point_world(self, body_name: str, local_pos: np.ndarray) -> np.ndarray:
        """Return world-frame position of a point fixed in a body."""
        X_WB = self._plant.EvalBodyPoseInWorld(
            self._plant_ctx, self._body[body_name])
        return X_WB.multiply(local_pos)

    @staticmethod
    def _tangent_point(A: np.ndarray, P: np.ndarray, r: float) -> np.ndarray:
        """
        Approximate tangent point on pulley of radius r centred at P,
        for tendon going from pulley to terminate point A.
        Returns the point on the pulley surface along the P→A direction.
        """
        v = A - P
        d = np.linalg.norm(v)
        if d < 1e-9:
            return P.copy()
        return P + r * (v / d)

    # ── Drake output callback ─────────────────────────────────────────────

    def CalcTorques(self, context, output):
        self._update_plant_ctx(context)
        motor_cmds = (self._motor_cmds_override
                      if self._motor_cmds_override is not None
                      else self._cmd_port.Eval(context))
        torques = np.zeros(self._plant.num_velocities())

        # 1. 计算腱张力产生的力矩
        for i, t in enumerate(self._tendons):
            θ = self._plant.GetJointByName(t["joint"]).get_angle(self._plant_ctx)
            L = t["L_ref"] - t["sign"] * t["pulley_r"] * (θ - t["theta_ref"])
            Ls = motor_cmds[i] if motor_cmds[i] > 1e-6 else t["L_slack"]
            F = max(0.0, self._k * (L - Ls))

            if F > 1e-9:
                joint = self._plant.GetJointByName(t["joint"])
                v_idx = joint.velocity_start()
                torques[v_idx] += t["sign"] * t["pulley_r"] * F

        # 2. 增加 DIP-PIP 的虚拟弹簧阻尼耦合力矩 (加入前馈重力补偿消除下垂)
        alpha = ALPHA_DIP_PIP
        # 保持较低刚度，防止在 dt=0.001 时出现高频震荡爆炸
        K_couple = 0.5  
        D_couple = 0.001

        pip_q_idx = self._plant.GetJointByName("pip").position_start()
        dip_q_idx = self._plant.GetJointByName("dip").position_start()
        pip_v_idx = self._plant.GetJointByName("pip").velocity_start()
        dip_v_idx = self._plant.GetJointByName("dip").velocity_start()

        state = self._state_port.Eval(context)
        nq = self._plant.num_positions()
        
        q_pip, q_dip = state[pip_q_idx], state[dip_q_idx]
        v_pip, v_dip = state[nq + pip_v_idx], state[nq + dip_v_idx]

        # ---- 新增：获取重力导致的关节力矩 ----
        tau_g = self._plant.CalcGravityGeneralizedForces(self._plant_ctx)
        
        # 虚拟连杆张力 = 弹簧力 - DIP 自身的重力拉力
        # (-tau_g[dip_v_idx] 相当于一个无形的力提前托住了 DIP，弹簧初始形变为 0)
        tau_linkage = K_couple * (alpha * q_pip - q_dip) + D_couple * (alpha * v_pip - v_dip) - tau_g[dip_v_idx]

        # 作用力施加到 DIP，同时！反作用力按比例压回给 PIP (这完美模拟了连杆的负载传导)
        torques[dip_v_idx] += tau_linkage
        torques[pip_v_idx] -= tau_linkage * alpha

        output.SetFromVector(torques)

    # ── Utility for logging / plotting ────────────────────────────────────

    def set_motor_commands(self, cmds: np.ndarray):
        """
        外部实时设置 motor commands，绕过 ConstantVectorSource input port。
        cmds: L_slack per tendon (metres)，顺序同 self._tendons。
        """
        self._motor_cmds_override = cmds.copy()

    def get_tensions(self, drake_context) -> dict:
        self._update_plant_ctx(drake_context)
        motor_cmds = (self._motor_cmds_override
                      if self._motor_cmds_override is not None
                      else self._cmd_port.Eval(drake_context))
        out = {}
        for i, t in enumerate(self._tendons):
            θ = self._plant.GetJointByName(t["joint"]).get_angle(self._plant_ctx)
            L = t["L_ref"] - t["sign"] * t["pulley_r"] * (θ - t["theta_ref"])
            Ls = motor_cmds[i] if motor_cmds[i] > 1e-6 else t["L_slack"]
            out[t["name"]] = max(0.0, self._k * (L - Ls))
        return out
