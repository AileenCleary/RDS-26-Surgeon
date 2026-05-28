#include "Globals.h"

// ==============================================================================
// OUTER LOOP: JOINT PIDs (Uses MA782 Encoders)
// ==============================================================================
bool feedbackEnabled = false; // Default off until sensors are working
const float MAX_CORRECTION_DEG = 15.0f; // Max offset the PID can apply

PIDController jointPIDs[3] = {
  {1.0, 0.0, 0.0, 0.0, 0.0, MAX_CORRECTION_DEG}, // Splay
  {1.0, 0.0, 0.0, 0.0, 0.0, MAX_CORRECTION_DEG}, // MCP
  {1.0, 0.0, 0.0, 0.0, 0.0, MAX_CORRECTION_DEG}  // PIP
};

// ==============================================================================
// INNER LOOP: MOTOR TORQUE PIDs (Uses ODrive Encoders)
// ==============================================================================
// These remain constant to manage the antagonistic tension
float motor_Kp_strong = 0.02f;
float motor_Kd_strong = 0.0005f;

// Antagonist (Yielding) Gains: Soft enough to not fight, strong enough to prevent slack
float motor_Kp_soft = 0.005f;
float motor_Kd_soft = 0.0002f;

// Steady-state baseline tension
float motor_pretension = 0.0006f; 

float motor_prev_error[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void resetPIDs() {
  for (int i = 0; i < 3; i++) jointPIDs[i].reset();
  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_prev_error[i] = 0.0f;
  }
}