#include "Globals.h"

bool useForceSensor = false; 

float currentForceTarget = 0.0f;

// These gains output Newtons (N) of required tip force based on Newtons of error
float force_Kp = 2.0f;   
float force_Ki = 0.1f;
float force_Kd = 0.1f;

float force_integral = 0.0f;
float force_prev_error = 0.0f;

void updateForceControl(float dt) {
  if (dt <= 0.0f) dt = 0.02f;

  if (currentMode == MODE_IDLE) {
    for (int i = 0; i < 5; i++) commanded_torque[i] = 0.0;
    return;
  }

  // 1. Measure actual state
  float actual_force = getForce(); 
  float error = currentForceTarget - actual_force;
  force_integral += error * dt;
  force_integral = constrain(force_integral, -200.0, 200.0);

  float derivative = (error - force_prev_error) / dt;
  force_prev_error = error;
  float force_correction = (force_Kp * error) + (force_Ki * force_integral) + (force_Kd * derivative);
  
  float cmd_force = constrain(currentForceTarget + force_correction, 0.0f, 50.0f);
  
  // 4. Calculate exact Jacobian decoupling for the corrected force
  float ff_torques[5];
  getFeedforwardMotorTorques(cmd_force, ff_torques);
  
  // 5. Inject pulling torque directly into the active flexors
  for (int i = 0; i < NUM_MOTORS; i++) {
    commanded_torque[i] += ff_torques[i]; 
  }

  // Serial.printf(
  //   "target=%.2f "
  //   "actual=%.2f "
  //   "error=%.2f "
  //   "cmd_force=%.2f\n",
  //   currentForceTarget,
  //   actual_force,
  //   error,
  //   cmd_force
  // );
  // Serial.printf(
  //   "T=[%.1f %.1f %.1f %.1f %.1f]\n",
  //   T[0], T[1], T[2], T[3], T[4]
  // );
  // Serial.printf(
  //   "tau_m=[%.4f %.4f %.4f %.4f %.4f]\n",
  //   commanded_torque[0], commanded_torque[1], commanded_torque[2], commanded_torque[3], commanded_torque[4]
  // );
  // Serial.printf(
  //   "tau_j=[%.3f %.3f %.3f]\n",
  //   tau_joint[0],
  //   tau_joint[1],
  //   tau_joint[2]
  // );
//   Serial.printf(
//     "Jz=[%.4f %.4f %.4f]\n",
//     J[2][0],
//     J[2][1],
//     J[2][2]
// );
}