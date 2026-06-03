#include "Globals.h"

bool useForceSensor = false; 

float currentForceTarget = 0.0f;

// These gains output Newtons (N) of required tip force based on Newtons of error
float force_Kp = 1.0f;   
float force_Ki = 0.0f;
float force_Kd = 0.0f;

float force_integral = 0.0f;
float force_prev_error = 0.0f;

void updateForceControl(float dt) {
  if (dt <= 0.0f) dt = 0.02f;

  if (currentMode == MODE_IDLE) {
    for (int i = 0; i < 5; i++) commanded_torque[i] = 0.0;
    return;
  }
  
  if (currentMode == MODE_CONTROL_TORQUE) {
    return;
  }

  float actual_force = getForce();
  float error = currentForceTarget - actual_force;

  force_integral += error * dt;
  force_integral = constrain(force_integral, -2.0f, 2.0f); // Anti-windup

  float derivative = (error - force_prev_error) / dt;
  force_prev_error = error;

  // Output: Commanded Force in Newtons + Feedforward Target
  float torque_pid = (force_Kp * error) + (force_Ki * force_integral) + (force_Kd * derivative);
  torque_pid = constrain(torque_pid, 0.0f, 5.0f);

  // 3. Decouple into Motor Torques
  if (currentForceTarget > 0.0) {
    if (actual_force > 0.0) {
      commanded_torque[0] = 0.0;
      commanded_torque[1] = 0.0;
      commanded_torque[3] = 0.0;
      commanded_torque[2] = -torque_pid;
      commanded_torque[4] = -torque_pid;
    } else {
      commanded_torque[0] = 0.0;
      commanded_torque[1] = 0.0;
      commanded_torque[3] = 0.0;
      commanded_torque[2] = -0.004;
      commanded_torque[4] = -0.004;
    }
  }
}