#include <math.h>
#include "Globals.h"

ControlMode currentMode = MODE_IDLE;
unsigned long motionStartTime = 0;

int sineAxis = 0;
float sineAmp = 0.0f;
float sineFreq = 0.0f;
float sineOffset = 0.0f;

float currentTipTarget[3] = {0.0f, 0.0f, 0.0f};
float currentJointTarget[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float currentMotorTarget[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Inner Loop: Calculate basic motor torque
float computeSingleMotorTorque(int motor_id, float target_turns, float kp, float kd, float dt) {
  float actual = odrive_data[motor_id].last_feedback.Pos_Estimate - motor_zero_offsets[motor_id];
  float error = target_turns - actual;
  float derivative = (error - motor_prev_error[motor_id]) / dt;
  motor_prev_error[motor_id] = error;
  
  return (kp * error) + (kd * derivative);
}

void computeAntagonisticTorque(int ext_id, int flex_id, int joint_idx, float current_joint_pos, float dt) {
  float joint_error = currentJointTarget[joint_idx] - current_joint_pos;
  
  bool is_flexing = (joint_error < -0.5f);
  bool is_extending = (joint_error > 0.5f);

  // Default to soft (holding state)
  float ext_Kp = motor_Kp_soft; float ext_Kd = motor_Kd_soft;
  float flex_Kp = motor_Kp_soft; float flex_Kd = motor_Kd_soft;

  // Assign Stiff gains to the pulling motor
  if (is_flexing) {
    flex_Kp = motor_Kp_strong; flex_Kd = motor_Kd_strong; // Flexor pulls
  } else if (is_extending) {
    ext_Kp = motor_Kp_strong; ext_Kd = motor_Kd_strong; // Extensor pulls
  }

  // Both motors actively track their kinematic targets, just with different stiffness!
  float ext_torque = computeSingleMotorTorque(ext_id, currentMotorTarget[ext_id], ext_Kp, ext_Kd, dt);
  float flex_torque = computeSingleMotorTorque(flex_id, currentMotorTarget[flex_id], flex_Kp, flex_Kd, dt);

  // Apply baseline pretension (Assumes negative torque pulls tendon in. Change to + if reversed).
  setMotorTorque(ext_id, ext_torque - motor_pretension);
  setMotorTorque(flex_id, flex_torque - motor_pretension);
}

void updateMotion(float dt) {
  if (dt <= 0.0f) dt = 0.02f;

  if (currentMode == MODE_IDLE) {
    for (int i = 0; i < 5; i++) setMotorTorque(i, 0.0f);
    return;
  }

  // ---------------------------------------------------------
  // 1. GENERATE CONTINUOUS SETPOINTS (SINE)
  // ---------------------------------------------------------
  if (currentMode == MODE_SINE_TIP || currentMode == MODE_SINE_JOINT || currentMode == MODE_SINE_MOTOR) {
    float t = (millis() - motionStartTime) / 1000.0f;
    float sineValue = sineOffset + sineAmp * sin(2.0f * PI * sineFreq * t);

    if (currentMode == MODE_SINE_TIP) currentTipTarget[sineAxis] = sineValue;
    else if (currentMode == MODE_SINE_JOINT) currentJointTarget[sineAxis] = sineValue;
    else if (currentMode == MODE_SINE_MOTOR) currentMotorTarget[sineAxis] = sineValue;
  }

  // ---------------------------------------------------------
  // 2. KINEMATIC CASCADE & OUTER LOOP (Joint PID)
  // ---------------------------------------------------------
  if (currentMode == MODE_CONTROL_TIP || currentMode == MODE_SINE_TIP || currentMode == MODE_TRAJ_STREAMING_TIP) {
    calculateJointAngles(currentTipTarget, currentJointTarget);
  }

  // Prepare the targets we will actually send to the motor IK
  // This allows us to modify them using the Joint Sensors without permanently altering the base target.
  float correctedJointTarget[4] = { currentJointTarget[0], currentJointTarget[1], currentJointTarget[2], currentJointTarget[3] };

  // Apply Outer Loop Joint PID if MA782 sensors are enabled
  if (feedbackEnabled && currentMode != MODE_CONTROL_MOTOR && currentMode != MODE_SINE_MOTOR && currentMode != MODE_TRAJ_STREAMING_MOTOR) {
    float* actualJoints = getJointAngles(); 
    correctedJointTarget[0] += jointPIDs[0].compute(currentJointTarget[0], actualJoints[0], dt);
    correctedJointTarget[1] += jointPIDs[1].compute(currentJointTarget[1], actualJoints[1], dt);
    correctedJointTarget[2] += jointPIDs[2].compute(currentJointTarget[2], actualJoints[2], dt);
    correctedJointTarget[3] = correctedJointTarget[2] * DIP_COUPLING_RATIO;
  }

  // Convert Joint targets to Motor targets
  if (currentMode != MODE_CONTROL_MOTOR && currentMode != MODE_SINE_MOTOR && currentMode != MODE_TRAJ_STREAMING_MOTOR) {
    float motorTargetsDeg[5];
    calculateMotorAngles(correctedJointTarget, motorTargetsDeg);
    for (int i = 0; i < 5; i++) {
      currentMotorTarget[i] = motorTargetsDeg[i] / DEGREES_PER_TURN;
    }
  }

  // ---------------------------------------------------------
  // 3. INNER LOOP (Motor Torque Output)
  // ---------------------------------------------------------
  if (currentMode == MODE_CONTROL_MOTOR || currentMode == MODE_SINE_MOTOR || currentMode == MODE_TRAJ_STREAMING_MOTOR) {
    for (int i = 0; i < 5; i++) {
      float torque = computeSingleMotorTorque(i, currentMotorTarget[i], motor_Kp_strong, motor_Kd_strong, dt);
      setMotorTorque(i, torque);
    }
    return;
  }

  // Normal Joint Operation (Antagonistic tracking)
  float currentJoints[4];
  estimateJointAnglesFromMotors(currentJoints);

  // M0: Splay (1-to-1 linkage, always strong)
  float splay_torque = computeSingleMotorTorque(0, currentMotorTarget[0], motor_Kp_strong, motor_Kd_strong, dt);
  setMotorTorque(0, splay_torque); // Splay typically doesn't need pretension

  // M1/M4: MCP Joint
  computeAntagonisticTorque(1, 4, 1, currentJoints[1], dt);

  // M3/M2: PIP Joint
  computeAntagonisticTorque(3, 2, 2, currentJoints[2], dt);
}