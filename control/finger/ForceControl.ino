#include "Globals.h"

bool useForceSensor = false; 

float currentForceTarget = 0.0f;

// These gains output Newtons (N) of required tip force based on Newtons of error
float force_Kp = 1.0f;   
float force_Ki = 0.5f;
float force_Kd = 0.01f;

float force_integral = 0.0f;
float force_prev_error = 0.0f;

void updateForceControl(float dt) {
  if (currentMode != MODE_CONTROL_FORCE) return;
  if (dt <= 0.0f) dt = 0.02f;

  float actual_force = useForceSensor ? getForce() : getEstimatedTipForceScalar();
  float error = currentForceTarget - actual_force;

  force_integral += error * dt;
  force_integral = constrain(force_integral, -20.0f, 20.0f); // Anti-windup

  float derivative = (error - force_prev_error) / dt;
  force_prev_error = error;

  // Output: Commanded Force in Newtons + Feedforward Target
  float F_pid = (force_Kp * error) + (force_Ki * force_integral) + (force_Kd * derivative);
  F_pid += currentForceTarget; 

  // Constrain total commanded force to 30 N for hardware safety
  F_pid = constrain(F_pid, -30.0f, 30.0f);

  // Apply the force vector. Assuming contact is against the Z-axis (pushing down = -Z direction)
  float F_vec[3] = {0.0f, 0.0f, -F_pid}; 

  // 1. Calculate Jacobian
  float J[3][3];
  calculateJacobian(J);

  // 2. Jacobian Transpose: tau_joint = J^T * F_vec
  float tau_joint[3];
  tau_joint[0] = J[0][0]*F_vec[0] + J[1][0]*F_vec[1] + J[2][0]*F_vec[2];
  tau_joint[1] = J[0][1]*F_vec[0] + J[1][1]*F_vec[1] + J[2][1]*F_vec[2];
  tau_joint[2] = J[0][2]*F_vec[0] + J[1][2]*F_vec[1] + J[2][2]*F_vec[2];

  // 3. Decouple into Motor Torques
  float tau_motor[5];
  mapJointTorquesToMotorTorques(tau_joint, tau_motor);

  // 4. Actuate
  for(int i = 0; i < 5; i++) {
    setMotorTorque(i, tau_motor[i]);
  }
}