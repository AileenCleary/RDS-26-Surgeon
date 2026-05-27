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

// ------------------------------------------------------------------
// ⭐ TEST 1: Maximum Fingertip Force
// ------------------------------------------------------------------
void testMaxForce(bool isFlexed) {
  Serial.println("\n--- TEST: MAX FINGERTIP FORCE ---");
  Serial.println("Ramping force until mechanical limit or 30N max.");
  Serial.println("Time(s), Target_Force(N), Actual_Force(N), MCP_Deg, PIP_Deg");

  if (isFlexed) {
    currentMode = MODE_CONTROL_JOINT;
    currentJointTarget[1] = -45.0f; 
    currentJointTarget[2] = -45.0f;
    runTestDuration(2000); 
  }
  
  currentMode = MODE_CONTROL_FORCE;
  currentForceTarget = 0.0f;
  
  unsigned long start = millis();
  unsigned long last_time = millis();
  unsigned long last_print = millis();
  
  // Ramp up for 15 seconds
  while (millis() - start < 15000) {
    pumpODriveCAN();
    unsigned long now = millis();
    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      
      // Ramp target by 2 N/sec (Max 30N)
      currentForceTarget += 2.0f * dt; 
      if (currentForceTarget > 20.0f) currentForceTarget = 20.0f;
      
      updateForceControl(dt);
      updateMotion(dt);
      
      if (now - last_print >= 50) { // 20Hz logging
        last_print = now;
        Serial.printf("%.3f, %.2f, %.2f, %.2f, %.2f\n", 
          (now-start)/1000.0f, currentForceTarget, getForce(), currentJointTarget[1], currentJointTarget[2]);
      }
    }
  }
  resetToZero();
}

// ------------------------------------------------------------------
// ⭐ TEST 2: Step Force Control Test
// ------------------------------------------------------------------
void testStepForce(float lowN, float highN) {
  Serial.printf("\n--- TEST: STEP FORCE (%.1fN <-> %.1fN) ---\n", lowN, highN);
  Serial.println("Time(s), Target_Force(N), Actual_Force(N)");
  
  currentMode = MODE_CONTROL_FORCE;
  currentForceTarget = lowN;
  
  unsigned long start = millis();
  unsigned long last_time = millis();
  unsigned long last_step = millis();
  unsigned long settling_t = 0;
  float peak = 0;
  float overshoot = 0.0f;
  float settling_time = 0.0f;
  float ss_error = 0.0f;
  
  // Run for 10 seconds total
  while (millis() - start < 10000) {
    pumpODriveCAN();
    unsigned long now = millis();
    
    // Toggle every 1000 ms
    if (now - last_step >= 1000) {
      last_step = now;
      currentForceTarget = (currentForceTarget == lowN) ? highN : lowN;
    }

    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      float target = currentForceTarget;
      float actual = getEstimatedTipForceScalar();
      float error = abs(target - actual);
      
      updateForceControl(dt);
      updateMotion(dt);

      if (actual > peak) {
        peak = actual;
      }
      if (settling_t == 0 and error < 0.6) {
        settling_t = now - start;
        settling_time = (float) settling_t / 1000.0f;
      }
      if (peak > target) {
        overshoot = peak - target;
      } else {
        overshoot = 0.0f;
      }
      ss_error = error;

      Serial.printf("%.3f, %.2f, %.2f\n", (now-start)/1000.0f, currentForceTarget, actual);
    }
  }
  resetToZero();
  Serial.printf("Settling Time: %.2fs | Overshoot: %.2f | Steady-State Error: %.2f\n", settling_time, overshoot, ss_error);
}

// ------------------------------------------------------------------
// ⭐ TEST 3: Step Position Control Test
// ------------------------------------------------------------------
void testStepPosition() {
  Serial.println("\n--- TEST: STEP POSITION CONTROL ---");
  Serial.println("Time(s), Target_Z_Tip(cm), Actual_Z_Tip_Est(cm)");
  
  currentMode = MODE_CONTROL_TIP;
  
  // Baseline starting position (Assume fully extended tip is roughly at some Z/X coordinate based on your FK)
  // Example uses Z oscillation. Update axes based on your specific forward kinematics frame.
  
  // Set targets relative to current base (convert cm to mm)
  float base_x = 1.899f;
  float lowZ = -5.0f;
  float highZ = -25.0f;
  
  currentTipTarget[0] = base_x;
  currentTipTarget[1] = 0.0f; // Splay
  currentTipTarget[2] = lowZ;
  
  unsigned long start = millis();
  unsigned long last_time = millis();
  unsigned long last_step = millis();
  unsigned long settling_t = 0;
  float peak = 0;
  float overshoot = 0.0f;
  float settling_time = 0.0f;
  float ss_error = 0.0f;
  
  while (millis() - start < 10000) {
    pumpODriveCAN();
    unsigned long now = millis();
    
    if (now - last_step >= 1000) {
      last_step = now;
      currentTipTarget[2] = (currentTipTarget[2] == lowZ) ? highZ : lowZ;
    }

    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      updateMotion(dt);
      
        // Estimate actual tip pos from motor IK
      float joints[4]; estimateJointAnglesFromMotors(joints);
      float actual_tip[3]; getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);
      float target = currentTipTarget[2];
      float actual = actual_tip[2];
      float error = abs(target - actual);

      if (actual > peak) {
        peak = actual;
      }
      if (settling_t == 0 and error < 5.0) {
        settling_t = now - start;
        settling_time = (float) settling_t / 1000.0f;
        ss_error = error;
      }
      if (peak > target) {
        overshoot = peak - target;
      } else {
        overshoot = 0.0f;
      }

      Serial.printf("%.3f, %.2f, %.2f\n", (now-start)/1000.0f, currentTipTarget[2], actual_tip[2]); 
    }
  }
  resetToZero();
  Serial.printf("Settling Time: %.2fs | Overshoot: %.2f | Steady-State Error: %.2f\n", settling_time, overshoot, ss_error);
}

// ------------------------------------------------------------------
// ⭐ TEST 4: Trajectory Tracking
// ------------------------------------------------------------------
void testTrajectory() {
  Serial.println("\n--- TEST: TRAJECTORY ---");
  Serial.println("Time(s), Target_X, Target_Z, Actual_X, Actual_Z");
  
  currentMode = MODE_CONTROL_TIP;
  float f = 0.5f; // Frequency (Hz)
  float base_x = 7.0; // Nominal center X (cm)
  float base_z = -40.0f; // Nominal center Z (cm)
  float duration = 2.0f;
  
  unsigned long start = millis();
  unsigned long last_time = millis();
  unsigned long last_print = millis();
  Serial.println("START_DATA");
  while (millis() - start < (duration * 1000)) {
    pumpODriveCAN();
    unsigned long now = millis();
    
    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      
      float t = (now - start) / 1000.0f;
      
      float target_x_cm = 15.0f * sin(2.0f * PI * f * t);
      float target_z_cm = 15.0f * sin(2.0f * PI * (2.0f * f) * t + (3.0f * PI / 4.0f));
      
      currentTipTarget[0] = (base_x + target_x_cm) * 10.0f; // to mm
      currentTipTarget[1] = 0.0f; // No splay
      currentTipTarget[2] = (base_z + target_z_cm) * 10.0f; // to mm
      
      updateMotion(dt);
      
      if (now - last_print >= 50) { 
        last_print = now;
        float joints[4]; estimateJointAnglesFromMotors(joints);
        float actual_tip[3]; getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);
        Serial.printf("%.3f, %.2f, %.2f, %.2f, %.2f\n", 
          t, currentTipTarget[0]/10.0f, currentTipTarget[2]/10.0f, actual_tip[0]/10.0f, actual_tip[2]/10.0f);
      }
    }
  }
  Serial.println("END_DATA");

  resetToZero();
}

// ------------------------------------------------------------------
// ⭐ TEST 5: Fingertip Impedance Test
// ------------------------------------------------------------------
void testImpedance() {
  Serial.println("\n--- TEST: IMPEDANCE CHARACTERIZATION ---");
  
  // Set to force control, targeting 0N (perfectly compliant)
  currentMode = MODE_CONTROL_FORCE;
  
  float frequencies[] = {0.5, 1.0, 2.0, 5.0};
  Serial.println("START_DATA");
  for (float f : frequencies) {
    float amp = 1.0; // 1 Newton oscillation
    unsigned long start = millis();
    unsigned long last_time = millis();
  
    while (millis() - start < 20000) { // 20 seconds to interact with it
      pumpODriveCAN();
      unsigned long now = millis();
      
      if (now - last_time >= 20) {
        float dt = (now - last_time) / 1000.0f;
        last_time = now;

        float t = (millis() - start) / 1000.0;
        currentForceTarget = amp * sin(2.0 * PI * f * t);
        
        updateForceControl(dt);
        updateMotion(dt);
        
        float joints[4]; estimateJointAnglesFromMotors(joints);
        float tip[3]; getForwardKinematics(joints[0], joints[1], joints[2], tip);
        Serial.printf("%.3f, %.3f, %.3f\n", t, currentForceTarget, tip[2]);
      }
    }
    resetToZero();
  };
  Serial.println("END_DATA");
}