// MotionControl.ino
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

void updateMotion(float dt) {
  if (currentMode == MODE_IDLE) return; // Completely idle, don't spam CAN bus

  // ---------------------------------------------------------
  // 1. DYNAMIC SETPOINT GENERATION (e.g., Sine Waves)
  // ---------------------------------------------------------
  if (currentMode == MODE_SINE_TIP || currentMode == MODE_SINE_JOINT || currentMode == MODE_SINE_MOTOR) {
    float t = (millis() - motionStartTime) / 1000.0f;
    float sineValue = sineOffset + sineAmp * sin(2.0f * PI * sineFreq * t);

    if (currentMode == MODE_SINE_TIP) currentTipTarget[sineAxis] = sineValue;
    else if (currentMode == MODE_SINE_JOINT) currentJointTarget[sineAxis] = sineValue;
    else if (currentMode == MODE_SINE_MOTOR) currentMotorTarget[sineAxis] = sineValue;
  }

  // ---------------------------------------------------------
  // 2. KINEMATIC CASCADE (Tip -> Joint)
  // ---------------------------------------------------------
  if (currentMode == MODE_CONTROL_TIP || currentMode == MODE_SINE_TIP || currentMode == MODE_TRAJ_STREAMING_TIP) {
    calculateJointAngles(currentTipTarget, currentJointTarget);
  }

  // ---------------------------------------------------------
  // 3. FEEDBACK CONTROL (Joint Level PID)
  // ---------------------------------------------------------
  // We only run PID if the user is controlling Tip or Joints. 
  // Direct motor control bypasses joint feedback.
  if (currentMode == MODE_CONTROL_TIP || currentMode == MODE_SINE_TIP || currentMode == MODE_TRAJ_STREAMING_TIP || currentMode == MODE_CONTROL_JOINT || currentMode == MODE_SINE_JOINT || currentMode == MODE_TRAJ_STREAMING_JOINT) {
    
    // Default commands if PID is OFF
    float commandJoints[4] = {
      currentJointTarget[0], 
      currentJointTarget[1], 
      currentJointTarget[2], 
      currentJointTarget[3]
    };

    if (feedbackEnabled) {
      float* actualJoints = getJointAngles(); 
      
      // Calculate PID corrections
      // commandJoints[0] += jointPIDs[0].compute(currentJointTarget[0], actualJoints[0], dt);
      commandJoints[1] += jointPIDs[1].compute(currentJointTarget[1], actualJoints[1], dt);
      commandJoints[2] += jointPIDs[2].compute(currentJointTarget[2], actualJoints[2], dt);
      Serial.print(currentJointTarget[2]);
      Serial.print(",");
      Serial.println(actualJoints[2]);
      
      // Maintain PIP/DIP physical coupling after correction
      commandJoints[3] = commandJoints[2] * DIP_COUPLING_RATIO;
    }

    // Convert final corrected joint angles to motor turns
    calculateMotorAngles(commandJoints, currentMotorTarget);
  }

  // ---------------------------------------------------------
  // 4. HARDWARE EXECUTION
  // ---------------------------------------------------------
  moveMotors(currentMotorTarget);
}