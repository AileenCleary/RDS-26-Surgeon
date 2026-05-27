#include "Globals.h"

void runTestDuration(unsigned long ms) {
  unsigned long start = millis();
  unsigned long last_time = millis();

  while (millis() - start < ms) {
    pumpODriveCAN();
    unsigned long now = millis();
    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      updateMotion(dt);
    }
  }
}

void resetToZero() {
  Serial.println("Returning to Zero...");
  currentMode = MODE_CONTROL_JOINT;
  currentJointTarget[0] = 0.0f; currentJointTarget[1] = 0.0f; currentJointTarget[2] = 0.0f;
  runTestDuration(2000);
}

// ==============================================================================
// SENSOR TESTS
// ==============================================================================
void testForceSensor() {
  Serial.println("\n--- TESTING FORCE SENSOR (10 Samples) ---");
  for (int i = 0; i < 10; i++) {
    float force = getForce();
    Serial.printf("Force: %.3f N\n", force);
    delay(200);
  }
}

void testJointSensors() {
  Serial.println("\n--- TESTING MA782 ENCODERS (10 Samples) ---");
  for (int i = 0; i < 10; i++) {
    float* joints = getJointAngles();
    Serial.printf("Splay:%.2f, MCP:%.2f, PIP:%.2f, DIP:%.2f\n", joints[0], joints[1], joints[2], joints[3]);
    delay(200);
  }
}

// ==============================================================================
// ENCODER CALIBRATION (Sweep tests)
// ==============================================================================
void runCalibrationSweep(int jointIdx, float end_deg) {
  Serial.println("START_DATA");
  bool prev_feedback = feedbackEnabled;
  feedbackEnabled = false; // MUST be open-loop for calibration
  currentMode = MODE_CONTROL_JOINT;
  
  unsigned long start = millis();
  unsigned long last_time = millis();
  
  while (millis() - start < 10000) {
    pumpODriveCAN();
    unsigned long now = millis();
    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      
      float t = (millis() - start) / 10000.0f;
      currentJointTarget[jointIdx] = end_deg * t;
      
      updateMotion(dt);
      
      float estimatedJointAngles[NUM_ENC];
      estimateJointAnglesFromMotors(estimatedJointAngles);
      Serial.printf("%.3f,%.2f,%u\n", t, estimatedJointAngles[jointIdx], raw_w[jointIdx]);
    }
  }
  Serial.println("END_DATA");
  feedbackEnabled = prev_feedback;
  resetToZero();
}

void calibrateEncoderSplay() { Serial.println("--- CALIBRATING SPLAY ---"); runCalibrationSweep(0, 20.0f); }
void calibrateEncoderMCP() { Serial.println("--- CALIBRATING MCP ---"); runCalibrationSweep(1, -60.0f); }
void calibrateEncoderPIP() { Serial.println("--- CALIBRATING PIP ---"); runCalibrationSweep(2, -90.0f); }
void calibrateEncoderDIP() { Serial.println("--- CALIBRATING DIP ---"); runCalibrationSweep(3, -60.0f); }

// ==============================================================================
// HARDWARE TESTS
// ==============================================================================
void testSingleMotor(int motorIndex) {
  Serial.printf("\n--- TESTING MOTOR %d ---\n", motorIndex);
  currentMode = MODE_CONTROL_MOTOR;
  currentMotorTarget[motorIndex] = 2.0f; runTestDuration(2000);
  currentMotorTarget[motorIndex] = -2.0f; runTestDuration(2000);
  currentMotorTarget[motorIndex] = 0.0f; runTestDuration(2000);
}

void testAllMotorsTogether() {
  Serial.println("\n--- TESTING ALL MOTORS ---");
  currentMode = MODE_CONTROL_MOTOR;
  for(int i=0; i<5; i++) currentMotorTarget[i] = 1.0f;
  runTestDuration(2000);
  for(int i=0; i<5; i++) currentMotorTarget[i] = 0.0f;
  runTestDuration(2000);
}

void testJointSplay() {
  Serial.println("TEST SPLAY");
  currentMode = MODE_CONTROL_JOINT;
  currentJointTarget[0] = 10.0f;
  runTestDuration(3000);
  resetToZero();
}

void testJointMCP() {
  Serial.println("TEST MCP");
  currentMode = MODE_CONTROL_JOINT;
  currentJointTarget[1] = -30.0f;
  runTestDuration(3000);
  resetToZero();
}

void testJointPIP() {
  Serial.println("TEST PIP");
  currentMode = MODE_CONTROL_JOINT;
  currentJointTarget[2] = -45.0f;
  runTestDuration(3000);
  resetToZero();
}

void testDemo() {
  Serial.println("\n--- TESTING DEMO ---");
  currentMode = MODE_CONTROL_JOINT;
  for (int i = 0; i < 3; i++) {
    currentJointTarget[1] = -30.0f; currentJointTarget[2] = -45.0f;
    runTestDuration(2000);
    currentJointTarget[1] = 0.0f; currentJointTarget[2] = 0.0f;
    runTestDuration(2000);
  }
}