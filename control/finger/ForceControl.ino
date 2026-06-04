#include "Globals.h"

bool useForceSensor = false; 

float currentForceTarget = 0.0f;

// These gains output Newtons (N) of required tip force based on Newtons of error
float force_Kp = 10.0f;   
float force_Ki = 0.0f;
float force_Kd = 0.1f;

float force_integral = 0.0f;
float force_prev_error = 0.0f;

// void updateForceControl(float dt) {
//   if (dt <= 0.0f) dt = 0.02f;

//   if (currentMode == MODE_IDLE) {
//     for (int i = 0; i < 5; i++) commanded_torque[i] = 0.0;
//     return;
//   }
  
//   if (currentMode == MODE_CONTROL_TORQUE) {
//     return;
//   }

//   float actual_force = getForce();
//   float error = currentForceTarget - actual_force;

//   force_integral += error * dt;
//   force_integral = constrain(force_integral, -2.0f, 2.0f); // Anti-windup

//   float derivative = (error - force_prev_error) / dt;
//   force_prev_error = error;

//   // Output: Commanded Force in Newtons + Feedforward Target
//   float torque_pid = (force_Kp * error) + (force_Ki * force_integral) + (force_Kd * derivative);
//   torque_pid = constrain(torque_pid, 0.0f, 5.0f);

//   // 3. Decouple into Motor Torques
//   if (currentForceTarget > 0.0) {
//     if (actual_force > 0.0) {
//       commanded_torque[0] = 0.0;
//       commanded_torque[1] = 0.0;
//       commanded_torque[3] = 0.0;
//       commanded_torque[2] = -torque_pid;
//       commanded_torque[4] = -torque_pid;
//     } else {
//       commanded_torque[0] = 0.0;
//       commanded_torque[1] = 0.0;
//       commanded_torque[3] = 0.0;
//       commanded_torque[2] = -0.004;
//       commanded_torque[4] = -0.004;
//     }
//   }
// }

void updateForceControl(float dt) {
  if (dt <= 0.0f) dt = 0.02f;

  // Only run if we are explicitly in Force Control mode
  if (currentMode != MODE_CONTROL_FORCE) {
    force_integral = 0.0f; // Reset integral windup for next time
    return;
  }

  float actual_force = getForce();
  
  // Optional deadband so it doesn't jitter when hunting for 0 N
  if (abs(actual_force) < 0.1f) actual_force = 0.0f;

  float error = currentForceTarget - actual_force;

  // PI Admittance Controller
  force_integral += error * dt;
  force_integral = constrain(force_integral, -20.0f, 20.0f); // Anti-windup

  float derivative = (error - force_prev_error) / dt;
  force_prev_error = error;

  // Calculate desired joint velocity (Degrees per Second)
  float joint_velocity = (force_Kp * error) + (force_Ki * force_integral) + (force_Kd * derivative);

  // Rate Limiter: Prevent the finger from moving too fast while searching for the table
  joint_velocity = constrain(joint_velocity, -40.0f, 40.0f); // Max 40 degrees/sec

  // Shift the kinematic position targets.
  // Flexion is negative. To push harder (positive error), we subtract velocity.
  currentJointTarget[1] -= (joint_velocity * dt); // Adjust MCP
  currentJointTarget[2] -= (joint_velocity * dt); // Adjust PIP

  // Hard Limits: Prevent the finger from trying to push entirely through the table
  currentJointTarget[1] = constrain(currentJointTarget[1], -180.0f, 0.0f);
  currentJointTarget[2] = constrain(currentJointTarget[2], -180.0f, 0.0f);

  // CRITICAL: We do NOT overwrite commanded_torque[] here anymore.
  // We let updateMotion() naturally calculate the correct antagonist torques 
  // to reach these gently shifting joint targets.
}