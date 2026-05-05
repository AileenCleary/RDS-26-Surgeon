#include <math.h>

#include "Globals.h"

// Define the global variables
ControlMode currentMode = MODE_IDLE;
unsigned long motionStartTime = 0;

int sineAxis = 0;
float sineAmp = 0.0f;
float sineFreq = 0.0f;
float sineOffset = 0.0f;

float currentTipTarget[3] = {0.0f, 0.0f, 0.0f};
float currentJointTarget[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float currentMotorTarget[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Updates the motors based on the current mode. Called continuously in loop().
void updateMotion(float dt) {
  if (currentMode == MODE_IDLE || currentMode == MODE_TRAJ_STREAMING) return;

  float t = (millis() - motionStartTime) / 1000.0f;
  float sineValue = sineOffset + sineAmp * sin(2.0f * PI * sineFreq * t);

  // 1. Determine base target based on mode
  if (currentMode == MODE_SINE_TIP) {
    currentTipTarget[sineAxis] = sineValue;
    calculateJointAngles(currentTipTarget, currentJointTarget);
  } else if (currentMode == MODE_SINE_JOINT) {
    currentJointTarget[sineAxis] = sineValue;
    if (sineAxis == 2) currentJointTarget[3] = currentJointTarget[2] * DIP_COUPLING_RATIO;
  } else if (currentMode == MODE_SINE_MOTOR) {
    currentMotorTarget[sineAxis] = sineValue;
    moveMotors(currentMotorTarget);
    return; // Skip joint PID if controlling motors directly
  }

  // 2. Apply Feedback Control (Outer-Loop Position Correction)
  float commandJoints[4] = {
    currentJointTarget[0], 
    currentJointTarget[1], 
    currentJointTarget[2], 
    currentJointTarget[3]
  };

  if (feedbackEnabled) {
    // Assuming getJointAngles() is defined in MA782.ino and returns a float*
    float* actualJoints = getJointAngles(); 
    
    // Add the calculated PID offset to the target position
    commandJoints[0] += jointPIDs[0].compute(currentJointTarget[0], actualJoints[0], dt);
    commandJoints[1] += jointPIDs[1].compute(currentJointTarget[1], actualJoints[1], dt);
    commandJoints[2] += jointPIDs[2].compute(currentJointTarget[2], actualJoints[2], dt);
    
    // Re-couple the DIP joint to the newly corrected PIP target
    commandJoints[3] = commandJoints[2] * DIP_COUPLING_RATIO;
  }

  // 3. Send to hardware
  float motorsOut[5];
  calculateMotorAngles(commandJoints, motorsOut);
  moveMotors(motorsOut);
}