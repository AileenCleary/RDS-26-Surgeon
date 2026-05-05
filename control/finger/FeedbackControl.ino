#include "Globals.h"

bool feedbackEnabled = false;

// Default PID K Values. 
// Start with Kp very low! Tendon systems can resonate if this is too high.
const float DEFAULT_KP = 0.3f;  
const float DEFAULT_KI = 0.01f; 
const float DEFAULT_KD = 0.02f; 
const float MAX_CORRECTION_DEG = 15.0f; // Never let the PID adjust a joint by more than 15 degrees

// Instantiate controllers for Splay (0), MCP (1), and PIP (2)
PIDController jointPIDs[3] = {
  {DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 0, 0, MAX_CORRECTION_DEG},
  {DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 0, 0, MAX_CORRECTION_DEG},
  {DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 0, 0, MAX_CORRECTION_DEG}
};

void resetPIDs() {
  for (int i = 0; i < 3; i++) {
    jointPIDs[i].reset();
  }
}