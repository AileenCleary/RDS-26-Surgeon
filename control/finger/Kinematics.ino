#include <math.h>

// Hardware constants and configuration
// Finger Link Lengths (mm)
const float L_SPLAY = 24.0f;
const float L_MCP = 44.0f;
const float L_PIP = 39.0f;
const float L_DIP = 22.0f;

// Joint Pulley Radii (mm)
const float R_PULLEY_SPLAY = 9.0f;
const float R_PULLEY_SPLAY_MCP = 6.4f;
const float R_PULLEY_SPLAY_PIP = 2.9f;
const float R_PULLEY_MCP_FLEX   = 12.4f;
const float R_PULLEY_MCP_EXT   = 9.1f;
const float R_PULLEY_MCP_PIP_FLEX   = 6.35f;
const float R_PULLEY_MCP_PIP_EXT   = 12.6f;
const float R_PULLEY_PIP   = 9.1f;
const float R_PULLEY_PIP_DIP   = 6.35f;
const float R_PULLEY_DIP   = 9.0f;

// Motor Radii (mm) and direction
const float R_MOTOR[5] = { 4.0f, 4.0f, 4.0f, 4.0f, 4.0f };
const float MOTOR_DIR[5] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }; // SPLAY, MCP_EXT, PIP_FLEX, PIP_EXT, MCP_FLEX

// DIP Coupling Ratio
const float DIP_COUPLING_RATIO = R_PULLEY_PIP_DIP / R_PULLEY_DIP;

// Direction matrix
const float D[5][3] = {
  {1.0f, 0.0f, 0.0f},
  {-1.0f, -1.0f,  0.0f},
  {-1.0f, -1.0, 1.0f},
  {1.0f, -1.0f, -1.0f},
  {1.0f, 1.0f, 0.0f}
};

// Structure matrix
const float S[5][3] = {
  // Splay              // MCP           // PIP
  {  R_PULLEY_SPLAY,  0.0f,            0.0f }, // Motor/Tendon 0 (SPLAY)
  {  R_PULLEY_SPLAY_MCP,  R_PULLEY_MCP_EXT,  0.0f }, // Motor/Tendon 1 (MCP Extension)
  {  R_PULLEY_SPLAY_PIP,  R_PULLEY_MCP_PIP_FLEX,  R_PULLEY_PIP }, // Motor/Tendon 2 (PIP Flexion)
  {  R_PULLEY_SPLAY_PIP,  R_PULLEY_MCP_PIP_EXT,  R_PULLEY_PIP }, // Motor/Tendon 3 (PIP Extension)
  {  R_PULLEY_SPLAY_MCP,  R_PULLEY_MCP_FLEX,  0.0f } // Motor/Tendon 4 (MCP Flexion)
};

// Forward Kinematics
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

// Inverse Kinematics
void calculateJointAngles(float* target, float* joints_out) {
  joints_out[0] = 0.0f; 
  joints_out[1] = 0.0f; 
  joints_out[2] = 0.0f; 
  joints_out[3] = 0.0f; 
  
  const int MAX_ITERATIONS = 50;
  const float LEARNING_RATE = 0.001f;
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

// Tendon Kinematics
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