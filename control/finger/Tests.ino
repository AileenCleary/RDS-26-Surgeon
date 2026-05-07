#include "Globals.h"

// ==============================================================================
// HELPERS
// ==============================================================================
void resetToZero() {
  float zeroMotors[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  moveMotors(zeroMotors);
  Serial.println("Hardware returned to Zero (Fully Extended).");
}

void moveJointsSafely(float splay, float mcp, float pip) {
  float joints[4] = {splay, mcp, pip, pip * DIP_COUPLING_RATIO};
  float motors[5];
  calculateMotorAngles(joints, motors);
  moveMotors(motors);
}

// ==============================================================================
// MOTOR TESTS
// ==============================================================================
void testSingleMotor(int motorIndex) {
  switch(motorIndex) {
    case 0:
      Serial.printf("\n--- TESTING SPLAY MOTOR ---\n");
      Serial.println("Moving +15 degrees...");
      moveSplay(15.0f);
      delay(1000);
      Serial.println("Moving -15 degrees...");
      moveSplay(-15.0f);
      delay(1000);
      break;
    case 1:
      Serial.printf("\n--- TESTING MCP FLEX MOTOR ---\n");
      Serial.println("Moving +15 degrees...");
      moveMCPFlex(15.0f);
      delay(1000);
      moveMCPFlex(-15.0f);
      delay(1000);
      break;
    case 2:
      Serial.printf("\n--- TESTING MCP EXT MOTOR ---\n");
      Serial.println("Moving +15 degrees...");
      moveMCPExt(15.0f); delay(1000);
      moveMCPExt(-15.0f); delay(1000);
      break;
    case 3:
      Serial.printf("\n--- TESTING PIP FLEX MOTOR ---\n");
      Serial.println("Moving +15 degrees...");
      movePIPFlex(15.0f); delay(1000);
      movePIPFlex(-15.0f); delay(1000);
      break;
    case 4:
      Serial.printf("\n--- TESTING PIP EXT MOTOR ---\n");
      Serial.println("Moving +15 degrees...");
      movePIPExt(15.0f); delay(1000);
      movePIPExt(-15.0f); delay(1000);
      break;
    default:
      Serial.println("Invalid motor index. Use 0-4.");
      return;
  }
  resetToZero();
}

void testAllMotorsTogether() {
  Serial.println("\n--- TESTING ALL 5 MOTORS TOGETHER ---\n");
  
  Serial.println("Moving all motors to +15 degrees...");
  float testArrayPos[5] = {15.0f, 15.0f, 15.0f, 15.0f, 15.0f};
  moveMotors(testArrayPos);
  delay(1000);
  
  Serial.println("Moving all motors to -15 degrees...");
  float testArrayNeg[5] = {-15.0f, -15.0f, -15.0f, -15.0f, -15.0f};
  moveMotors(testArrayNeg);
  delay(1000);
  
  resetToZero();
}

// ==============================================================================
// JOINT TESTS
// ==============================================================================
void testJointSplay() {
  Serial.println("\n--- TESTING SPLAY JOINT ---\n");
  Serial.println("Splaying +10 degrees...");
  moveJointsSafely(10.0f, 0.0f, 0.0f); delay(1500);
  
  Serial.println("Splaying -10 degrees...");
  moveJointsSafely(-10.0f, 0.0f, 0.0f); delay(1500);
  resetToZero();
}

void testJointMCP() {
  Serial.println("\n--- TESTING MCP JOINT ---\n");
  Serial.println("Flexing MCP to -45 degrees...");
  moveJointsSafely(0.0f, -45.0f, 0.0f); delay(1500);
  
  Serial.println("Flexing MCP to -90 degrees...");
  moveJointsSafely(0.0f, -90.0f, 0.0f); delay(1500);
  resetToZero();
}

void testJointPIP() {
  Serial.println("\n--- TESTING PIP/DIP COUPLED JOINTS ---\n");
  Serial.println("Flexing PIP to -45 degrees (DIP will follow automatically)...");
  moveJointsSafely(0.0f, 0.0f, -45.0f); delay(1500);
  
  Serial.println("Flexing PIP to -90 degrees...");
  moveJointsSafely(0.0f, 0.0f, -90.0f); delay(1500);
  resetToZero();
}

// ==============================================================================
// KINEMATICS TESTS
// ==============================================================================
void runJointTest(float splay, float mcp, float pip, float dip) {
  float joints[4] = {splay, mcp, pip, dip};
  float tip[3];
  float motors[5];
  
  Serial.printf("\n--- Testing Joint Angles (%.1f, %.1f, %.1f, %.1f) ---\n", splay, mcp, pip, dip);
  getForwardKinematics(joints[0], joints[1], joints[2], tip);
  Serial.printf("Resulting Tip Position: X: %.2f, Y: %.2f, Z: %.2f\n", tip[0], tip[1], tip[2]);
  
  calculateMotorAngles(joints, motors);
  Serial.printf("Motor Angles: M0:%.1f, M1:%.1f, M2:%.1f, M3:%.1f, M4:%.1f\n", 
                 motors[0], motors[1], motors[2], motors[3], motors[4]);
}

void testKinematics() {
  Serial.println("================= KINEMATICS TESTS =================");
  
  Serial.println("\n--- Testing IK for Tip Position (0, 0, 0) ---");
  float targetTip[3] = {0.0f, 0.0f, 0.0f};
  float calcJoints[4];
  float calcMotors[5];
  
  calculateJointAngles(targetTip, calcJoints);
  Serial.printf("Calculated Joints: Splay:%.2f, MCP:%.2f, PIP:%.2f, DIP:%.2f\n", 
                 calcJoints[0], calcJoints[1], calcJoints[2], calcJoints[3]);
                 
  calculateMotorAngles(calcJoints, calcMotors);
  Serial.printf("Calculated Motors: M0:%.1f, M1:%.1f, M2:%.1f, M3:%.1f, M4:%.1f\n", 
                 calcMotors[0], calcMotors[1], calcMotors[2], calcMotors[3], calcMotors[4]);

  // Static Math Tests
  runJointTest(0.0f,   0.0f,   0.0f,   0.0f);
  runJointTest(10.0f,  0.0f,   0.0f,   0.0f);
  runJointTest(-10.0f, 0.0f,   0.0f,   0.0f);
  runJointTest(0.0f,   -90.0f, -90.0f, -62.8f);
  runJointTest(10.0f,  -90.0f, -90.0f, -62.8f);
  runJointTest(-10.0f, -90.0f, -90.0f, -62.8f);
  
  Serial.println("\n====================== DONE ======================");
}

// ==============================================================================
// SENSOR TESTS (Prints 10 samples to the Serial Monitor)
// ==============================================================================
void testForceSensor() {
  Serial.println("\n--- TESTING FORCE SENSOR (10 Samples) ---\n");
  for (int i = 0; i < 10; i++) {
    // float force = getForce();
    printForce();
    delay(200);
  }
  Serial.println("Force Sensor Test Complete.");
}

void testJointSensors() {
  Serial.println("\n--- TESTING MA782 ENCODERS (10 Samples) ---\n");
  for (int i = 0; i < 10; i++) {
    handleCommandMA782(); // Ensure registers are updated before reading
    // float* thetaList = getJointAngles();
    printJointAngles();
    delay(200);
  }
  Serial.println("Encoder Test Complete.");
}

// void testDemo() {
//   Serial.println("\n--- TESTING DEMO (SPLAY CLOSED-LOOP STEP) (10 s) ---");

//   // 1. Enable global feedback and clear any old PID memory
//   feedbackEnabled = true;
//   resetPIDs(); 

//   unsigned long startTime = millis();
//   unsigned long stepTimer = millis();
//   unsigned long lastLoopTime = micros();

//   float target_splay_deg = 10.0f; // Start with +10 deg step

//   // Run the test for exactly 10 seconds
//   while (millis() - startTime < 10000) {
//     unsigned long currentMillis = millis();

//     // 2. Step the target every 2 seconds
//     if (currentMillis - stepTimer >= 2000) {
//       target_splay_deg = -target_splay_deg; // Toggle between +10 and -10
//       stepTimer = currentMillis;
//     }

//     // 3. Read the actual joint angle from the MA782 sensors
//     handleCommandMA782(); 
//     float* thetaList = getJointAngles(); 
//     float current_splay_deg = thetaList[0]; // Index 0 is Splay

//     // 4. Calculate dt
//     float dt = (micros() - lastLoopTime) / 1000000.0f;
//     if (dt <= 0.0001f) dt = 0.001f; // Prevent divide by zero
//     lastLoopTime = micros();

//     // 5. Calculate PID Correction using jointPIDs[0]
//     float error = target_splay_deg - current_splay_deg;
    
//     // Note: Adjust the struct variable names below if they differ in your Globals.h
//     jointPIDs[0].integral += error * dt;
//     float d_error = (error - jointPIDs[0].prevError) / dt;
    
//     float pid_correction_deg = (jointPIDs[0].Kp * error) + 
//                                (jointPIDs[0].Ki * jointPIDs[0].integral) + 
//                                (jointPIDs[0].Kd * d_error);
                               
//     // Apply safety clamp so the PID doesn't rip the tendon if a sensor fails
//     if (pid_correction_deg > jointPIDs[0].outputLimit) pid_correction_deg = jointPIDs[0].outputLimit;
//     if (pid_correction_deg < -jointPIDs[0].outputLimit) pid_correction_deg = -jointPIDs[0].outputLimit;
    
//     jointPIDs[0].prevError = error;

//     // 6. Apply correction to the target and feed it to the Kinematics engine
//     float commanded_splay = target_splay_deg + pid_correction_deg;
    
//     // Use the joint control command as requested
//     moveJointsSafely(commanded_splay, 0.0f, 0.0f);
//     pumpODriveCAN(); 

//     // 7. Print directly to Serial Plotter format
//     Serial.print("Target:");
//     Serial.print(target_splay_deg);
//     Serial.print(","); // Comma separates the variables
//     Serial.print("Actual:");
//     Serial.println(current_splay_deg);

//     delay(10); // Loop at roughly 100Hz
//   }

//   Serial.println("Demo Test Complete. Returning to Zero.");
//   feedbackEnabled = false; // Turn off feedback for safety
//   resetToZero();
// }

void testDemo() {
  Serial.println("\n--- TESTING DEMO (SPLAY CLOSED-LOOP STEP) (10 s) ---");
  for (int i = 0; i < 10; i++) {
    moveSplay(15.0f);
    delay(1000);
    moveSplay(-15.0f);
    delay(1000);
  }
}

void testLinearitySplay() {
  Serial.println("\n--- TESTING SPLAY LINEARITY (BCT SWEEP CHECK) ---");
  
  // 1. Temporarily disable feedback so the motor moves perfectly linearly.
  // This allows us to see the sensor's raw, uncorrected geometric distortion.
  bool previousFeedbackState = feedbackEnabled;
  feedbackEnabled = false; 

  float start_deg = -20.0f;
  float end_deg = 20.0f;
  float sweep_time_sec = 10.0f; // 10 seconds for a slow, high-resolution sweep
  
  // 2. Move to the starting position and let it settle
  Serial.println("Moving to start position (-20 deg)...");
  moveJointsSafely(start_deg, 0.0f, 0.0f);
  
  // Wait 2 seconds, giving you time to open the Serial Plotter
  Serial.println("Starting Sweep in 2 seconds... Open Serial Plotter NOW!");
  delay(2000); 

  unsigned long startTime = millis();
  
  // 3. Execute the linear sweep
  while (true) {
    float t = (millis() - startTime) / 1000.0f;
    if (t > sweep_time_sec) break;
    
    // Calculate the perfectly linear expected angle
    float current_expected_deg = start_deg + ((end_deg - start_deg) * (t / sweep_time_sec));
    
    // Command the motor (Open-loop via Kinematics)
    moveJointsSafely(current_expected_deg, 0.0f, 0.0f);
    pumpODriveCAN(); 
    
    // Read the actual MA782 sensor
    handleCommandMA782(); 
    float* thetaList = getJointAngles(); 
    float sensor_deg = thetaList[0]; // Splay is index 0
    
    // 4. Print directly to Serial Plotter format
    Serial.print("Expected_Linear:");
    Serial.print(current_expected_deg);
    Serial.print(",");
    Serial.print("Sensor_Actual:");
    Serial.println(sensor_deg);
    
    delay(20); // 50 Hz update rate for a smooth graph
  }
  
  Serial.println("Linearity Test Complete. Returning to Zero.");
  resetToZero();
  
  // Restore the PID to whatever state the user had it in previously
  feedbackEnabled = previousFeedbackState; 
}

void testLinearityMCP() {
  Serial.println("\n--- TESTING MCP LINEARITY (BCT SWEEP CHECK) ---");
  
  bool previousFeedbackState = feedbackEnabled;
  feedbackEnabled = false; 

  float start_deg = 0.0f;
  float end_deg = 60.0f; // Typical MCP range
  float sweep_time_sec = 10.0f; 
  
  Serial.println("Moving to start position (0 deg)...");
  moveJointsSafely(0.0f, start_deg, 0.0f);
  
  Serial.println("Starting Sweep in 2 seconds... Open Serial Plotter NOW!");
  delay(2000); 

  unsigned long startTime = millis();
  
  while (true) {
    float t = (millis() - startTime) / 1000.0f;
    if (t > sweep_time_sec) break;
    
    float current_expected_deg = start_deg + ((end_deg - start_deg) * (t / sweep_time_sec));
    
    moveJointsSafely(0.0f, current_expected_deg, 0.0f);
    pumpODriveCAN(); 
    
    handleCommandMA782(); 
    float* thetaList = getJointAngles(); 
    float sensor_deg = thetaList[1]; // MCP is index 1
    
    Serial.print("Expected_Linear:");
    Serial.print(current_expected_deg);
    Serial.print(",");
    Serial.print("Sensor_Actual:");
    Serial.println(sensor_deg);
    
    delay(20); 
  }
  
  Serial.println("Linearity Test Complete. Returning to Zero.");
  resetToZero();
  feedbackEnabled = previousFeedbackState; 
}

void testLinearityPIP() {
  Serial.println("\n--- TESTING PIP LINEARITY (BCT SWEEP CHECK) ---");
  
  bool previousFeedbackState = feedbackEnabled;
  feedbackEnabled = false; 

  float start_deg = 0.0f;
  float end_deg = 90.0f; // Typical PIP range
  float sweep_time_sec = 10.0f; 
  
  Serial.println("Moving to start position (0 deg)...");
  moveJointsSafely(0.0f, 0.0f, start_deg);
  
  Serial.println("Starting Sweep in 2 seconds... Open Serial Plotter NOW!");
  delay(2000); 

  unsigned long startTime = millis();
  
  while (true) {
    float t = (millis() - startTime) / 1000.0f;
    if (t > sweep_time_sec) break;
    
    float current_expected_deg = start_deg + ((end_deg - start_deg) * (t / sweep_time_sec));
    
    moveJointsSafely(0.0f, 0.0f, current_expected_deg);
    pumpODriveCAN(); 
    
    handleCommandMA782(); 
    float* thetaList = getJointAngles(); 
    float sensor_deg = thetaList[2]; // PIP is index 2
    
    Serial.print("Expected_Linear:");
    Serial.print(current_expected_deg);
    Serial.print(",");
    Serial.print("Sensor_Actual:");
    Serial.println(sensor_deg);
    
    delay(20); 
  }
  
  Serial.println("Linearity Test Complete. Returning to Zero.");
  resetToZero();
  feedbackEnabled = previousFeedbackState; 
}

void testLinearityDIP() {
  Serial.println("\n--- TESTING DIP LINEARITY (BCT SWEEP CHECK) ---");
  
  bool previousFeedbackState = feedbackEnabled;
  feedbackEnabled = false; 

  // To test the DIP, we command the PIP to sweep
  float pip_start_deg = 0.0f;
  float pip_end_deg = 90.0f; 
  float sweep_time_sec = 10.0f; 
  
  Serial.println("Moving to start position (0 deg)...");
  moveJointsSafely(0.0f, 0.0f, pip_start_deg);
  
  Serial.println("Starting Sweep in 2 seconds... Open Serial Plotter NOW!");
  delay(2000); 

  unsigned long startTime = millis();
  
  while (true) {
    float t = (millis() - startTime) / 1000.0f;
    if (t > sweep_time_sec) break;
    
    float current_expected_pip_deg = pip_start_deg + ((pip_end_deg - pip_start_deg) * (t / sweep_time_sec));
    
    // The expected DIP angle is proportional to the expected PIP angle
    float current_expected_dip_deg = current_expected_pip_deg * DIP_COUPLING_RATIO;
    
    moveJointsSafely(0.0f, 0.0f, current_expected_pip_deg);
    pumpODriveCAN(); 
    
    handleCommandMA782(); 
    float* thetaList = getJointAngles(); 
    float sensor_deg = thetaList[3]; // DIP is index 3
    
    Serial.print("Expected_Linear:");
    Serial.print(current_expected_dip_deg);
    Serial.print(",");
    Serial.print("Sensor_Actual:");
    Serial.println(sensor_deg);
    
    delay(20); 
  }
  
  Serial.println("Linearity Test Complete. Returning to Zero.");
  resetToZero();
  feedbackEnabled = previousFeedbackState; 
}