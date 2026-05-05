#pragma once

const int NUM_ENC = 4;

// Control State Machine
enum ControlMode {
  MODE_IDLE,
  MODE_SINE_TIP,
  MODE_SINE_JOINT,
  MODE_SINE_MOTOR,
  MODE_TRAJ_STREAMING
};

extern ControlMode currentMode;

// Motion Parameters
extern unsigned long motionStartTime;

// Sine Wave Configuration
extern int sineAxis; // 0=X/Splay/M0, 1=Y/MCP/M1, etc.
extern float sineAmp;
extern float sineFreq;
extern float sineOffset;

// Global Arrays for current target states
extern float currentTipTarget[3];
extern float currentJointTarget[4];
extern float currentMotorTarget[5];

// PID Control
struct PIDController {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prevError;
  float outputLimit; // Prevents wild corrections (e.g., max 15 degrees of over-pull)

  void reset() {
    integral = 0.0f;
    prevError = 0.0f;
  }

  float compute(float target, float actual, float dt) {
    if (dt <= 0.0f) return 0.0f;
    
    float error = target - actual;
    integral += error * dt;
    
    // Anti-windup for the integral term
    if (integral > outputLimit) integral = outputLimit;
    if (integral < -outputLimit) integral = -outputLimit;

    float derivative = (error - prevError) / dt;
    prevError = error;
    
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // Clamp the final correction output
    if (output > outputLimit) output = outputLimit;
    if (output < -outputLimit) output = -outputLimit;
    
    return output;
  }
};

extern bool feedbackEnabled;
extern PIDController jointPIDs[3];