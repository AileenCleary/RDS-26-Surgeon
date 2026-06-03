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

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }
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

    Serial.printf(
      "RAW S:%u M:%u P:%u D:%u | DEG S:%.2f M:%.2f P:%.2f D:%.2f\n",
      raw_w[0], raw_w[1], raw_w[2], raw_w[3],
      joints[0], joints[1], joints[2], joints[3]
    );

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
    getJointAngles();
    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      
      float t = (millis() - start) / 10000.0f;
      if (jointIdx == 3) {
        currentJointTarget[2] = end_deg * t / DIP_COUPLING_RATIO;
      }
      currentJointTarget[jointIdx] = end_deg * t;
      
      updateMotion(dt);

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }
      
      float estimatedJointAngles[NUM_ENC];
      estimateJointAnglesFromMotors(estimatedJointAngles);
      Serial.printf("%.3f,%.2f,%u\n", t, estimatedJointAngles[jointIdx], raw_w[jointIdx]);
    }
  }
  Serial.println("END_DATA");
  feedbackEnabled = prev_feedback;
  resetToZero();
}

void calibrateEncoderSplay() { Serial.println("--- CALIBRATING SPLAY ---"); runCalibrationSweep(0, -20.0f); }
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
  currentJointTarget[0] = -10.0f;
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
  
  currentMode = MODE_CONTROL_JOINT;
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
      
      // Ramp target by 2 N/sec (Max 20N)
      currentForceTarget += 2.0f * dt; 
      if (currentForceTarget > 20.0f) currentForceTarget = 20.0f;
      
      updateMotion(dt);
      updateForceControl(dt);

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }

      float actual_force = getForce();
      
      if (now - last_print >= 50) { // 20Hz logging
        last_print = now;
        Serial.printf("%.3f, %.2f, %.2f\n", 
          (now-start)/1000.0f, currentForceTarget, actual_force);
      }
    }
  }
  currentForceTarget = 0.0f;
  resetToZero();
}

// ------------------------------------------------------------------
// ⭐ TEST 2: Step Force Control Test
// ------------------------------------------------------------------
void testStepForce(float lowN, float highN) {
  Serial.printf("\n--- TEST: STEP FORCE (%.1fN <-> %.1fN) ---\n", lowN, highN);
  Serial.println("Time(s), Target_Force(N), Actual_Force(N)");
  
  currentMode = MODE_CONTROL_JOINT;
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
      float actual = getForce();
      float error = abs(target - actual);
      
      updateMotion(dt);
      updateForceControl(dt);

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }

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
  Serial.println("Time(s), Target_Z_Tip, Actual_Z_Tip");
  
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

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }
      
        // Estimate actual tip pos from motor IK
      // float joints[4]; estimateJointAnglesFromMotors(joints);
      // float actual_tip[3]; getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);
      float* joints = getJointAngles();

      float actual_tip[3];
      getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);
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
  
  feedbackEnabled = true;
  resetPIDs();


  currentMode = MODE_CONTROL_TIP;
  float f = 0.05f; // Frequency (Hz)
  float base_x = 75.03; // Nominal center X 
  float base_z = -70.09f; // Nominal center Z 
  float duration = 20.0f;

  // --- 1. MOVE TO START POSITION ---
  Serial.println("Moving to start position...");
  // Calculate targets at t = 0
  float target_x_start = 15.0f * sin(0.0f);
  float target_z_start = 15.0f * sin(3.0f * PI / 4.0f);
  
  currentTipTarget[0] = base_x + target_x_start;
  currentTipTarget[1] = 0.0f; // No splay
  currentTipTarget[2] = (base_z + target_z_start); 
  
  // Give the finger 1.5 seconds to smoothly move to the starting coordinates
  runTestDuration(1500);

  unsigned long start = millis();
  unsigned long last_time = millis();
  Serial.println("START_DATA");
  while (millis() - start < (duration * 1000)) {
    pumpODriveCAN();
    unsigned long now = millis();
    
    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      
      float t = (now - start) / 1000.0f;
      
      float target_x = 15.0f * sin(2.0f * PI * f * t);
      float target_z = 15.0f * sin(2.0f * PI * (2.0f * f) * t + (3.0f * PI / 4.0f));
      
      currentTipTarget[0] = base_x + target_x;
      currentTipTarget[1] = 0.0f; // No splay
      currentTipTarget[2] = (base_z + target_z); 
      
      updateMotion(dt);

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }
      
      float* joints = getJointAngles();

      float actual_tip[3];
      getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);
      //getForwardKinematics4(joints[0], joints[1], joints[2], joints[3], actual_tip);
      // Serial.printf("%.3f, %.2f, %.2f, %.2f, %.2f\n", 
      //   t, currentTipTarget[0], currentTipTarget[2], actual_tip[0], actual_tip[2]);
      Serial.printf(
        "%.3f, TX %.2f, TZ %.2f, AX %.2f, AZ %.2f, "
        "TJ S %.2f M %.2f P %.2f D %.2f, "
        "AJ S %.2f M %.2f P %.2f D %.2f\n",
        t,
        currentTipTarget[0],
        currentTipTarget[2],
        actual_tip[0],
        actual_tip[2],
        currentJointTarget[0],
        currentJointTarget[1],
        currentJointTarget[2],
        currentJointTarget[3],
        joints[0],
        joints[1],
        joints[2],
        joints[3]
      );
    }
  }
  Serial.println("END_DATA");

  resetToZero();
  Serial.println("START_DATA");

  // Pick several static points from the same trajectory.
  // These are points on the original trajectory at t = 0, 5, 10, 15, 20 seconds.
  // float test_times[] = {0.0f, 5.0f, 10.0f, 15.0f, 20.0f};
  // int num_points = 5;

  // for (int i = 0; i < num_points; i++) {
  //   float t = test_times[i];

  //   float target_x = 15.0f * sin(2.0f * PI * f * t);
  //   float target_z = 15.0f * sin(2.0f * PI * (2.0f * f) * t + (3.0f * PI / 4.0f));

  //   currentTipTarget[0] = base_x + target_x;
  //   currentTipTarget[1] = 0.0f;
  //   currentTipTarget[2] = base_z + target_z;

  //   // Hold this point for 3 seconds so the finger can settle
  //   unsigned long point_start = millis();
  //   unsigned long last_time = millis();

  //   while (millis() - point_start < 3000) {
  //     pumpODriveCAN();

  //     unsigned long now = millis();
  //     if (now - last_time >= 20) {
  //       float dt = (now - last_time) / 1000.0f;
  //       last_time = now;

  //       updateMotion(dt);
  //     }
  //   }

  //   // After settling, read actual joint angles and compute actual fingertip position
  //   float* joints = getJointAngles();

  //   float actual_tip[3];
  //   getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);

  //   Serial.printf("%.3f, %.2f, %.2f, %.2f, %.2f\n",
  //     t,
  //     currentTipTarget[0],
  //     currentTipTarget[2],
  //     actual_tip[0],
  //     actual_tip[2]
  //   );

  //   delay(300);
  // }

  // Serial.println("END_DATA");

  // resetToZero();
}

// ------------------------------------------------------------------
// ⭐ TEST 6: Sinusoidal Position Control Test (Frequency Response)
// ------------------------------------------------------------------
void testSinePosition() {
  Serial.println("\n--- TEST: SINE POSITION CONTROL (0.1 to 100 Hz) ---");
  Serial.println("START_DATA");
  
  currentMode = MODE_CONTROL_TIP;
  
  float base_x = 1.899f;
  float base_z = 0.0f; 
  
  // Chirp Parameters
  float f0 = 0.1f;        // Start frequency (Hz)
  float f1 = 100.0f;      // End frequency (Hz)
  float T = 60.0f;        // Sweep duration (Seconds)
  
  // Logarithmic sweep rate constant: k = ln(f1/f0) / T
  float k = log(f1 / f0) / T;
  
  unsigned long start = millis();
  unsigned long last_time = millis();
  
  while (millis() - start < (T * 1000)) {
    pumpODriveCAN();
    unsigned long now = millis();
    
    // FAST CONTROL LOOP: 2ms (500 Hz) to allow tracking up to 100 Hz
    if (now - last_time >= 2) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;
      
      float t = (now - start) / 1000.0f;
      
      // Calculate instantaneous phase for logarithmic chirp
      // Phase integral of f0 * exp(k*t) = f0 * (exp(k*t) - 1) / k
      float phase = 2.0f * PI * f0 * (exp(k * t) - 1.0f) / k;
      
      // Target: (-15.0 + 10.0 * sin(phase)) mm
      float target_z_offset_mm = -15.0f + 10.0f * sin(phase);
      
      currentTipTarget[0] = base_x;
      currentTipTarget[1] = 0.0f; 
      currentTipTarget[2] = base_z + target_z_offset_mm;
      
      updateMotion(dt);

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }
      
      // Estimate actual tip pos from motor IK
      float joints[4]; estimateJointAnglesFromMotors(joints);
      float actual_tip[3]; getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);
      
      // Log: Time, Expected Z, Actual Z
      Serial.printf("%.4f, %.3f, %.3f\n", t, currentTipTarget[2], actual_tip[2]); 
    }
  }
  
  resetToZero();
  Serial.println("END_DATA");
}

void testTipPulseToZero() {
  Serial.println("\n--- TEST: TIP PULSE TO ZERO ---");
  Serial.println("Alternating every 1 second between TIP(60,0,-80) and JOINT ZERO for 10 seconds.");

  feedbackEnabled = true;
  resetPIDs();
  enableAllMotors();

  unsigned long test_start = millis();
  unsigned long last_time = millis();

  int last_phase = -1;

  while (millis() - test_start < 10000) {
    pumpODriveCAN();

    unsigned long now = millis();
    float elapsed_s = (now - test_start) / 1000.0f;

    // phase changes every 1 second: 0,1,2,3...
    int phase = (int)elapsed_s;

    if (phase != last_phase) {
      last_phase = phase;

      if (phase % 2 == 0) {
        // Even seconds: move to tip target
        currentMode = MODE_CONTROL_TIP;
        currentTipTarget[0] = 60.0f;
        currentTipTarget[1] = 0.0f;
        currentTipTarget[2] = -80.0f;

        Serial.printf("t=%.2f: Target = TIP 60, 0, -80\n", elapsed_s);
      } else {
        // Odd seconds: return to joint zero
        currentMode = MODE_CONTROL_JOINT;
        currentJointTarget[0] = 0.0f;
        currentJointTarget[1] = 0.0f;
        currentJointTarget[2] = 0.0f;
        currentJointTarget[3] = 0.0f;

        Serial.printf("t=%.2f: Target = JOINT ZERO\n", elapsed_s);
      }
    }

    if (now - last_time >= 20) {
      float dt = (now - last_time) / 1000.0f;
      last_time = now;

      updateMotion(dt);

      for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorTorque(i, commanded_torque[i]);
      }

      float* joints = getJointAngles();

      float actual_tip[3];
      getForwardKinematics(joints[0], joints[1], joints[2], actual_tip);

      Serial.printf(
        "DATA %.3f, mode %d, target_tip %.2f %.2f %.2f, actual_tip %.2f %.2f %.2f, joints %.2f %.2f %.2f %.2f\n",
        elapsed_s,
        currentMode,
        currentTipTarget[0],
        currentTipTarget[1],
        currentTipTarget[2],
        actual_tip[0],
        actual_tip[1],
        actual_tip[2],
        joints[0],
        joints[1],
        joints[2],
        joints[3]
      );
    }
  }

  Serial.println("Finished pulse test. Returning to zero...");
  resetToZero();
}