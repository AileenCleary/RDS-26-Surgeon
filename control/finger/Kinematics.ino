#include <math.h>
#include "Globals.h"

const float L_SPLAY = 24.0f;
const float L_MCP = 44.0f;
const float L_PIP = 39.0f;
const float L_DIP = 22.0f;
const float L_PENCIL = 38.97f;

const float R_TENDON = 0.2794f;
const float R_PULLEY_SPLAY = 9.0f + R_TENDON;
const float R_PULLEY_SPLAY_MCP = 6.4f + R_TENDON;
const float R_PULLEY_SPLAY_PIP = 2.9f + R_TENDON;
const float R_PULLEY_MCP_FLEX   = 12.4f + R_TENDON;
const float R_PULLEY_MCP_EXT   = 9.1f + R_TENDON;
const float R_PULLEY_MCP_PIP_FLEX   = 6.35f + R_TENDON;
const float R_PULLEY_MCP_PIP_EXT   = 12.6f + R_TENDON;
const float R_PULLEY_PIP   = 9.1f + R_TENDON;
const float R_PULLEY_PIP_DIP   = 6.35f + R_TENDON;
const float R_PULLEY_DIP   = 9.0f + R_TENDON;

const float R_MOTOR[5] = { 4.0f + R_TENDON, 4.0f + R_TENDON, 4.0f + R_TENDON, 4.0f + R_TENDON, 4.0f + R_TENDON };
const float MOTOR_DIR[5] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }; 
const float DIP_COUPLING_RATIO = R_PULLEY_PIP_DIP / R_PULLEY_DIP;

const float D[5][3] = {
  {1.0f, 0.0f, 0.0f},
  {-1.0f, -1.0f,  0.0f},
  {-1.0f, -1.0, 1.0f},
  {1.0f, -1.0f, -1.0f},
  {1.0f, 1.0f, 0.0f}
};

const float S[5][3] = {
  {  R_PULLEY_SPLAY,  0.0f,            0.0f }, 
  {  R_PULLEY_SPLAY_MCP,  R_PULLEY_MCP_EXT,  0.0f }, 
  {  R_PULLEY_SPLAY_PIP,  R_PULLEY_MCP_PIP_FLEX,  R_PULLEY_PIP }, 
  {  R_PULLEY_SPLAY_PIP,  R_PULLEY_MCP_PIP_EXT,  R_PULLEY_PIP }, 
  {  R_PULLEY_SPLAY_MCP,  R_PULLEY_MCP_FLEX,  0.0f } 
};

void estimateJointAnglesFromMotors(float* joints_out) {
  float t[5] = {0};
  for (int i=0; i<5; i++) {
    if (odrive_data[i].received_feedback) {
      float m_turns = odrive_data[i].last_feedback.Pos_Estimate - motor_zero_offsets[i];
      t[i] = ((m_turns * DEGREES_PER_TURN * DEG_TO_RAD) * R_MOTOR[i]) / MOTOR_DIR[i];
    }
  }

  static float prev_q1 = 0.0f;
  static float prev_q2 = 0.0f;

  float q0 = t[0] / (D[0][0] * S[0][0]);

  float q1_ext  = (t[1] - (D[1][0] * S[1][0] * q0)) / (D[1][1] * S[1][1]);
  float q1_flex = (t[4] - (D[4][0] * S[4][0] * q0)) / (D[4][1] * S[4][1]);
  
  float q1 = 0.0f;
  if (currentJointTarget[1] < (prev_q1 * RAD_TO_DEG)) { 
    q1 = q1_flex; 
  } else { 
    q1 = q1_ext;  
  }

  float q2_ext  = (t[3] - (D[3][0] * S[3][0] * q0) - (D[3][1] * S[3][1] * q1)) / (D[3][2] * S[3][2]);
  float q2_flex = (t[2] - (D[2][0] * S[2][0] * q0) - (D[2][1] * S[2][1] * q1)) / (D[2][2] * S[2][2]);
  
  float q2 = 0.0f;
  if (currentJointTarget[2] < (prev_q2 * RAD_TO_DEG)) {
    q2 = q2_flex;
  } else {
    q2 = q2_ext;
  }
  
  prev_q1 = q1;
  prev_q2 = q2;
  
  joints_out[0] = q0 * RAD_TO_DEG;
  joints_out[1] = q1 * RAD_TO_DEG;
  joints_out[2] = q2 * RAD_TO_DEG;
  joints_out[3] = joints_out[2] * DIP_COUPLING_RATIO;
}

void getForwardKinematics(float q_splay_deg, float q_mcp_deg, float q_pip_deg, float* tip_out) {
  float q_dip_deg = q_pip_deg * DIP_COUPLING_RATIO;
  
  float a1 = q_mcp_deg * DEG_TO_RAD;
  float a2 = (q_mcp_deg + q_pip_deg) * DEG_TO_RAD;
  float a3 = (q_mcp_deg + q_pip_deg + q_dip_deg) * DEG_TO_RAD;
  float splay_rad = q_splay_deg * DEG_TO_RAD;
  
  float x_planar = L_SPLAY + L_MCP * cos(a1) + L_PIP * cos(a2) + L_DIP * cos(a3);
  float z_planar = L_MCP * sin(a1) + L_PIP * sin(a2) + L_DIP * sin(a3);
  
  tip_out[0] = L_SPLAY + L_MCP + L_PIP + L_DIP - (x_planar * cos(splay_rad));
  tip_out[1] = x_planar * sin(splay_rad);
  tip_out[2] = z_planar;
}

void calculateJointAngles(float* target, float* joints_out) {
  const int MAX_ITERATIONS = 200;
  const float LEARNING_RATE = 0.01f;
  const float TOLERANCE = 0.5f; 
  const float DELTA = 1.0f;

  float current[3], p_splay[3], p_mcp[3], p_pip[3];

  for (int i = 0; i < MAX_ITERATIONS; i++) {
    getForwardKinematics(joints_out[0], joints_out[1], joints_out[2], current);
    
    float err_x = target[0] - current[0];
    float err_y = target[1] - current[1];
    float err_z = target[2] - current[2];
    
    float dist = sqrt(err_x*err_x + err_y*err_y + err_z*err_z);
    if (dist < TOLERANCE) break;

    getForwardKinematics(joints_out[0] + DELTA, joints_out[1], joints_out[2], p_splay);
    getForwardKinematics(joints_out[0], joints_out[1] + DELTA, joints_out[2], p_mcp);
    getForwardKinematics(joints_out[0], joints_out[1], joints_out[2] + DELTA, p_pip);

    float d_splay = ((p_splay[0] - current[0])/DELTA)*err_x + 
                    ((p_splay[1] - current[1])/DELTA)*err_y + 
                    ((p_splay[2] - current[2])/DELTA)*err_z;
                    
    float d_mcp   = ((p_mcp[0] - current[0])/DELTA)*err_x + 
                    ((p_mcp[1] - current[1])/DELTA)*err_y + 
                    ((p_mcp[2] - current[2])/DELTA)*err_z;
                    
    float d_pip   = ((p_pip[0] - current[0])/DELTA)*err_x + 
                    ((p_pip[1] - current[1])/DELTA)*err_y + 
                    ((p_pip[2] - current[2])/DELTA)*err_z;

    joints_out[0] += LEARNING_RATE * d_splay;
    joints_out[1] += LEARNING_RATE * d_mcp;
    joints_out[2] += LEARNING_RATE * d_pip;

    joints_out[0] = constrain(joints_out[0], -10.0f, 10.0f);
    joints_out[1] = constrain(joints_out[1], -90.0f, 0.0f);
    joints_out[2] = constrain(joints_out[2], -90.0f, 0.0f);
  }
  
  joints_out[3] = joints_out[2] * DIP_COUPLING_RATIO;
}

void calculateMotorAngles(float* joints, float* motorAngles_out) {
  float q_rad[3] = { 
    (float)(joints[0] * DEG_TO_RAD), 
    (float)(joints[1] * DEG_TO_RAD), 
    (float)(joints[2] * DEG_TO_RAD)
  };
  
  for (int m = 0; m < 5; m++) {
    float tendon_displacement = 0.0f;
    for (int j = 0; j < 3; j++) {
      tendon_displacement += D[m][j] * S[m][j] * q_rad[j];
    }
    float motor_rad = (tendon_displacement / R_MOTOR[m]) * MOTOR_DIR[m];
    motorAngles_out[m] = motor_rad * RAD_TO_DEG;
  }
}

// Computes the 3x3 Geometric Jacobian
void calculateJacobian(float J_out[3][3]) {
  float tip_base[3], tip_delta[3];
  float delta_rad = 0.001f; 
  float delta_deg = delta_rad * RAD_TO_DEG;
  
  float q_deg[4];
  estimateJointAnglesFromMotors(q_deg);
  getForwardKinematics(q_deg[0], q_deg[1], q_deg[2], tip_base);

  for (int j = 0; j < 3; j++) {
    float q_temp[3] = { q_deg[0], q_deg[1], q_deg[2] };
    q_temp[j] += delta_deg; 
    getForwardKinematics(q_temp[0], q_temp[1], q_temp[2], tip_delta);
    
    // J = dx / dq. (Convert mm to meters)
    J_out[0][j] = ((tip_delta[0] - tip_base[0]) / 1000.0f) / delta_rad;
    J_out[1][j] = ((tip_delta[1] - tip_base[1]) / 1000.0f) / delta_rad;
    J_out[2][j] = ((tip_delta[2] - tip_base[2]) / 1000.0f) / delta_rad;
  }
}

// Maps 3D Joint Torques (Nm) to 5x Motor Torques (Nm) via Exact Decoupling
void mapJointTorquesToMotorTorques(float* tau_joint, float* tau_motor_out) {
  float T_pre = 2.0f; // 2 Newtons of baseline tendon pretension
  float T[5] = {0.0f, T_pre, T_pre, T_pre, T_pre}; 

  float R_mj[5][3];
  for (int m = 0; m < 5; m++) {
    for (int j = 0; j < 3; j++) {
      // Virtual work moment arm mapping: tau = -J^T * T
      R_mj[m][j] = -D[m][j] * (S[m][j] / 1000.0f);
    }
  }

  // 1. PIP Joint (Crossed by M2, M3)
  if (tau_joint[2] < 0.0f) { // Requires Flexion
    T[3] = T_pre;
    T[2] = (tau_joint[2] - R_mj[3][2]*T[3]) / R_mj[2][2];
  } else { // Requires Extension
    T[2] = T_pre;
    T[3] = (tau_joint[2] - R_mj[2][2]*T[2]) / R_mj[3][2];
  }

  // 2. MCP Joint (Crossed by M1, M4. Disturbed by M2, M3)
  float tau_mcp_disturb = R_mj[2][1]*T[2] + R_mj[3][1]*T[3];
  float tau_mcp_req = tau_joint[1] - tau_mcp_disturb;

  if (tau_mcp_req < 0.0f) { // Requires Flexion
    T[1] = T_pre;
    T[4] = (tau_mcp_req - R_mj[1][1]*T[1]) / R_mj[4][1];
  } else { // Requires Extension
    T[4] = T_pre;
    T[1] = (tau_mcp_req - R_mj[4][1]*T[4]) / R_mj[1][1];
  }

  // 3. Splay Joint (Crossed by M0. Disturbed by M1, M2, M3, M4)
  float tau_splay_disturb = R_mj[1][0]*T[1] + R_mj[2][0]*T[2] + R_mj[3][0]*T[3] + R_mj[4][0]*T[4];
  float tau_splay_req = tau_joint[0] - tau_splay_disturb;
  T[0] = tau_splay_req / R_mj[0][0];

  // Convert Tensions to Motor Torques
  const float GEAR_RATIO = 25.62f;
  const float EFFICIENCY = 0.85f;
  
  for(int m = 0; m < 5; m++) {
    // Soft constraint to avoid tendon snapping (Max ~60 N of physical tension)
    if (m > 0) T[m] = constrain(T[m], T_pre, 60.0f);
    else T[m] = constrain(T[m], -60.0f, 60.0f); // M0 is a belt, can support negative tension
    
    tau_motor_out[m] = -T[m] * (R_MOTOR[m] / 1000.0f) / (GEAR_RATIO * EFFICIENCY) * MOTOR_DIR[m];
  }
}

// Solves (J^T)^-1 to estimate the current tip force using the known motor torques
float getEstimatedTipForceScalar() {
  float q_deg[4];
  estimateJointAnglesFromMotors(q_deg);

  float T_tendon[5];
  for (int m = 0; m < 5; m++) {
    float tau_shaft = commanded_torque[m] * 25.62f * 0.85f;
    T_tendon[m] = tau_shaft / (R_MOTOR[m] / 1000.0f) * MOTOR_DIR[m];
  }

  float tau_joint[3] = {0.0f, 0.0f, 0.0f};
  for (int j = 0; j < 3; j++) {
    for (int m = 0; m < 5; m++) {
      tau_joint[j] += -D[m][j] * (S[m][j] / 1000.0f) * T_tendon[m];
    }
  }

  float J[3][3];
  calculateJacobian(J);

  // Simplified extraction: Assuming the primary contact force opposes the Z-axis
  // Fz = tau_PIP / Jz_PIP (Approximation for scalar output)
  if (abs(J[2][2]) > 0.0001f) {
    return abs(tau_joint[2] / J[2][2]);
  }
  return 0.0f;
}

// --- ADD THIS TO YOUR KINEMATICS SECTION ---
// Calculate tip of the pencil (offset perpendicularly from the DIP link)
void getPencilForwardKinematics(float q_splay_deg, float q_mcp_deg, float q_pip_deg, float* tip_out) {
  float q_dip_deg = q_pip_deg * DIP_COUPLING_RATIO;
  
  float a1 = q_mcp_deg * DEG_TO_RAD;
  float a2 = (q_mcp_deg + q_pip_deg) * DEG_TO_RAD;
  float a3 = (q_mcp_deg + q_pip_deg + q_dip_deg) * DEG_TO_RAD;
  float a4 = (q_mcp_deg + q_pip_deg + q_dip_deg - 90) * DEG_TO_RAD;
  float splay_rad = q_splay_deg * DEG_TO_RAD;
  
  float x_planar = L_SPLAY + L_MCP * cos(a1) + L_PIP * cos(a2) + L_DIP * cos(a3) + L_PENCIL * cos(a4);
  float z_planar = L_MCP * sin(a1) + L_PIP * sin(a2) + L_DIP * sin(a3) + L_PENCIL * sin(a4);
  
  tip_out[0] = L_SPLAY + L_MCP + L_PIP + L_DIP - (x_planar * cos(splay_rad));
  tip_out[1] = x_planar * sin(splay_rad);
  tip_out[2] = z_planar;
}

void calculateJointAnglesFromPencil(float* target, float* joints_out) {
  const int MAX_ITERATIONS = 200;
  const float LEARNING_RATE = 0.01f;
  const float TOLERANCE = 0.5f; 
  const float DELTA = 1.0f;

  float current[3], p_splay[3], p_mcp[3], p_pip[3];

  for (int i = 0; i < MAX_ITERATIONS; i++) {
    getPencilForwardKinematics(joints_out[0], joints_out[1], joints_out[2], current);
    
    float err_x = target[0] - current[0];
    float err_y = target[1] - current[1];
    float err_z = target[2] - current[2];
    
    float dist = sqrt(err_x*err_x + err_y*err_y + err_z*err_z);
    if (dist < TOLERANCE) break;

    getPencilForwardKinematics(joints_out[0] + DELTA, joints_out[1], joints_out[2], p_splay);
    getPencilForwardKinematics(joints_out[0], joints_out[1] + DELTA, joints_out[2], p_mcp);
    getPencilForwardKinematics(joints_out[0], joints_out[1], joints_out[2] + DELTA, p_pip);

    float d_splay = ((p_splay[0] - current[0])/DELTA)*err_x + 
                    ((p_splay[1] - current[1])/DELTA)*err_y + 
                    ((p_splay[2] - current[2])/DELTA)*err_z;
                    
    float d_mcp   = ((p_mcp[0] - current[0])/DELTA)*err_x + 
                    ((p_mcp[1] - current[1])/DELTA)*err_y + 
                    ((p_mcp[2] - current[2])/DELTA)*err_z;
                    
    float d_pip   = ((p_pip[0] - current[0])/DELTA)*err_x + 
                    ((p_pip[1] - current[1])/DELTA)*err_y + 
                    ((p_pip[2] - current[2])/DELTA)*err_z;

    joints_out[0] += LEARNING_RATE * d_splay;
    joints_out[1] += LEARNING_RATE * d_mcp;
    joints_out[2] += LEARNING_RATE * d_pip;

    joints_out[0] = constrain(joints_out[0], -10.0f, 10.0f);
    joints_out[1] = constrain(joints_out[1], -90.0f, 0.0f);
    joints_out[2] = constrain(joints_out[2], -90.0f, 0.0f);
  }
  
  joints_out[3] = joints_out[2] * DIP_COUPLING_RATIO;
}

void getFeedforwardMotorTorques(float Fz_N, float* tau_motor_out) {
  float J[3][3];
  calculateJacobian(J);

  // If the finger pushes DOWN into the table (-Z), the table pushes UP (+Z) on the finger.
  // We use this reaction force to map to joint torques: tau = J^T * F_env
  float tau_joint[3];
  for(int j=0; j<3; j++) {
      tau_joint[j] = -J[2][j] * Fz_N; 
  }

  // Exact Decoupling for Tensions (with 0 baseline, so it ONLY adds pulling force, no slack)
  float T[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float R_mj[5][3];
  for (int m = 0; m < 5; m++) {
      for (int j = 0; j < 3; j++) {
          R_mj[m][j] = -D[m][j] * (S[m][j] / 1000.0f);
      }
  }

  // 1. PIP Joint
  if (tau_joint[2] < 0.0f) { T[2] = tau_joint[2] / R_mj[2][2]; T[3] = 0.0f; }
  else { T[3] = tau_joint[2] / R_mj[3][2]; T[2] = 0.0f; }

  // 2. MCP Joint
  float tau_mcp_disturb = R_mj[2][1]*T[2] + R_mj[3][1]*T[3];
  float tau_mcp_req = tau_joint[1] - tau_mcp_disturb;
  if (tau_mcp_req < 0.0f) { T[4] = tau_mcp_req / R_mj[4][1]; T[1] = 0.0f; }
  else { T[1] = tau_mcp_req / R_mj[1][1]; T[4] = 0.0f; }

  // 3. Splay Joint
  float tau_splay_disturb = R_mj[1][0]*T[1] + R_mj[2][0]*T[2] + R_mj[3][0]*T[3] + R_mj[4][0]*T[4];
  T[0] = (tau_joint[0] - tau_splay_disturb) / R_mj[0][0];

  // Convert Tensions to Motor Torques
  const float GEAR_RATIO = 25.62f;
  const float EFFICIENCY = 0.85f;
  for(int m = 0; m < 5; m++) {
      if (m > 0) T[m] = constrain(T[m], 0.0f, 200.0f); // Positive tension only
      else T[m] = constrain(T[m], -200.0f, 200.0f); // M0 is a belt, can pull both ways

      tau_motor_out[m] = -T[m] * (R_MOTOR[m] / 1000.0f) / (GEAR_RATIO * EFFICIENCY) * MOTOR_DIR[m];
  }
}

int getLetterPoints(char letter, float out_pts[20][2]) {
  letter = toupper(letter); // Ensure uppercase
  
  switch(letter) {
    case 'A': { float pts[5][2] = {{0, 0}, {0.5, 1}, {1, 0}, {0.75, 0.5}, {0.25, 0.5}};
                memcpy(out_pts, pts, sizeof(pts)); return 5; }
    case 'B': { float pts[10][2] = {{0, 0}, {0, 1}, {0.75, 1}, {1, 0.75}, {0.75, 0.5}, {0, 0.5}, {0.75, 0.5}, {1, 0.25}, {0.75, 0}, {0, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 10; }
    case 'C': { float pts[6][2] = {{1, 1}, {0.25, 1}, {0, 0.75}, {0, 0.25}, {0.25, 0}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 6; }
    case 'D': { float pts[7][2] = {{0, 0}, {0, 1}, {0.75, 1}, {1, 0.75}, {1, 0.25}, {0.75, 0}, {0, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 7; }
    case 'E': { float pts[7][2] = {{1, 1}, {0, 1}, {0, 0.5}, {0.75, 0.5}, {0, 0.5}, {0, 0}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 7; }
    case 'F': { float pts[6][2] = {{0, 0}, {0, 1}, {1, 1}, {0, 1}, {0, 0.5}, {0.75, 0.5}};
                memcpy(out_pts, pts, sizeof(pts)); return 6; }
    case 'G': { float pts[8][2] = {{1, 1}, {0.25, 1}, {0, 0.75}, {0, 0.25}, {0.25, 0}, {1, 0}, {1, 0.5}, {0.5, 0.5}};
                memcpy(out_pts, pts, sizeof(pts)); return 8; }
    case 'H': { float pts[6][2] = {{0, 1}, {0, 0}, {0, 0.5}, {1, 0.5}, {1, 1}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 6; }
    case 'I': { float pts[6][2] = {{0.25, 1}, {0.75, 1}, {0.5, 1}, {0.5, 0}, {0.25, 0}, {0.75, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 6; }
    case 'J': { float pts[5][2] = {{0, 0.5}, {0.25, 0}, {0.75, 0}, {1, 0.25}, {1, 1}};
                memcpy(out_pts, pts, sizeof(pts)); return 5; }
    case 'K': { float pts[6][2] = {{0, 1}, {0, 0}, {0, 0.5}, {1, 1}, {0, 0.5}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 6; }
    case 'L': { float pts[3][2] = {{0, 1}, {0, 0}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 3; }
    case 'M': { float pts[5][2] = {{0, 0}, {0, 1}, {0.5, 0.5}, {1, 1}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 5; }
    case 'N': { float pts[4][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}};
                memcpy(out_pts, pts, sizeof(pts)); return 4; }
    case 'O': { float pts[9][2] = {{0.5, 1}, {0.2, 0.8}, {0, 0.5}, {0.2, 0.2}, {0.5, 0}, {0.8, 0.2}, {1, 0.5}, {0.8, 0.8}, {0.5, 1}};
                memcpy(out_pts, pts, sizeof(pts)); return 9; }
    case 'P': { float pts[6][2] = {{0, 0}, {0, 1}, {0.75, 1}, {1, 0.75}, {0.75, 0.5}, {0, 0.5}};
                memcpy(out_pts, pts, sizeof(pts)); return 6; }
    case 'Q': { float pts[11][2] = {{0.8, 0.2}, {1, 0.5}, {0.8, 0.8}, {0.5, 1}, {0.2, 0.8}, {0, 0.5}, {0.2, 0.2}, {0.5, 0}, {0.8, 0.2}, {0.5, 0.5}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 11; }
    case 'R': { float pts[8][2] = {{0, 0}, {0, 1}, {0.75, 1}, {1, 0.75}, {0.75, 0.5}, {0, 0.5}, {0.5, 0.5}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 8; }
    case 'S': { float pts[8][2] = {{1, 1}, {0.25, 1}, {0, 0.75}, {0.25, 0.5}, {0.75, 0.5}, {1, 0.25}, {0.75, 0}, {0, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 8; }
    case 'T': { float pts[4][2] = {{0, 1}, {1, 1}, {0.5, 1}, {0.5, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 4; }
    case 'U': { float pts[6][2] = {{0, 1}, {0, 0.25}, {0.25, 0}, {0.75, 0}, {1, 0.25}, {1, 1}};
                memcpy(out_pts, pts, sizeof(pts)); return 6; }
    case 'V': { float pts[3][2] = {{0, 1}, {0.5, 0}, {1, 1}};
                memcpy(out_pts, pts, sizeof(pts)); return 3; }
    case 'W': { float pts[5][2] = {{0, 1}, {0.25, 0}, {0.5, 0.5}, {0.75, 0}, {1, 1}};
                memcpy(out_pts, pts, sizeof(pts)); return 5; }
    case 'X': { float pts[5][2] = {{0, 1}, {1, 0}, {0.5, 0.5}, {0, 0}, {1, 1}};
                memcpy(out_pts, pts, sizeof(pts)); return 5; }
    case 'Y': { float pts[5][2] = {{0, 1}, {0.5, 0.5}, {1, 1}, {0.5, 0.5}, {0.5, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 5; }
    case 'Z': { float pts[4][2] = {{0, 1}, {1, 1}, {0, 0}, {1, 0}};
                memcpy(out_pts, pts, sizeof(pts)); return 4; }
  }
  
  return 0; // Invalid character
}