#include "Globals.h"

bool useForceSensor = false; 

float currentForceTarget = 0.0f;

// These gains output Newtons (N) of required tip force based on Newtons of error
float force_Kp = 10.0f;   
float force_Ki = 0.0f;
float force_Kd = 0.1f;

float force_integral = 0.0f;
float force_prev_error = 0.0f;

void updateForceControl(float dt) {
  if (dt <= 0.0f) dt = 0.02f;

  // Run force control if explicitly called, or if it's enabled during drawing/tip mode
  if (currentMode != MODE_CONTROL_FORCE && currentMode != MODE_CONTROL_TIP && currentMode != MODE_TRAJ_STREAMING_TIP) {
    force_integral = 0.0f; // Reset integral windup
    return;
  }

  // 1. Calculate Force Error
  float actual_force = getForce(); // Or getEstimatedTipForceScalar() if using kinematic estimation
  if (abs(actual_force) < 0.1f) actual_force = 0.0f;

  float error = currentForceTarget - actual_force;

  force_integral += error * dt;
  force_integral = constrain(force_integral, -20.0f, 20.0f); // Anti-windup

  float derivative = (error - force_prev_error) / dt;
  force_prev_error = error;

  // The magnitude of how much we need to adjust the joints to fix the force error
  float admittance_mag = (force_Kp * error) + (force_Ki * force_integral) + (force_Kd * derivative);
  admittance_mag = constrain(admittance_mag, -40.0f, 40.0f); // Max degree shift per second

  // 2. Calculate what direction the Force Sensor is facing (Unit Normal Vector)
  float joints[NUM_ENC];
  estimateJointAnglesFromMotors(joints);
  float splay_rad = joints[0] * DEG_TO_RAD;
  // Absolute angle of the DIP link in the sagittal plane
  float a3_rad = (joints[1] + joints[2] + joints[3]) * DEG_TO_RAD; 

  // Assuming the sensor is mounted perpendicularly to the DIP link pointing "down"
  float nx = sin(a3_rad) * cos(splay_rad);
  float ny = sin(a3_rad) * sin(splay_rad);
  float nz = -cos(a3_rad);

  // 3. Get the current geometric leverage of the finger
  float J[3][3];
  calculateJacobian(J);

  // 4. Jacobian Transpose (J^T) Mapping: 
  // Maps the 3D direction vector (nx, ny, nz) into proportional joint targets
  float dq_splay = J[0][0] * nx + J[1][0] * ny + J[2][0] * nz;
  float dq_mcp   = J[0][1] * nx + J[1][1] * ny + J[2][1] * nz;
  float dq_pip   = J[0][2] * nx + J[1][2] * ny + J[2][2] * nz;

  // 5. Apply the mapped target changes
  // currentJointTarget[0] += dq_splay * admittance_mag * dt;
  // currentJointTarget[1] += dq_mcp * admittance_mag * dt;
  // currentJointTarget[2] += dq_pip * admittance_mag * dt;
  currentJointTarget[1] -= admittance_mag * dt;
  currentJointTarget[2] -= 
  

  

  // Hard safety limits
  currentJointTarget[0] = constrain(currentJointTarget[0], -15.0f, 15.0f);
  currentJointTarget[1] = constrain(currentJointTarget[1], -180.0f, 0.0f);
  currentJointTarget[2] = constrain(currentJointTarget[2], -180.0f, 0.0f);
  currentJointTarget[3] = currentJointTarget[2] * DIP_COUPLING_RATIO;
}