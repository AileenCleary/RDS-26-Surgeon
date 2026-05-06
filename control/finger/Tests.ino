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

void testDemo() {
  Serial.println("\n--- TESTING DEMO (SPLAY STEP) (10 s) ---\n");
  for (int i = 0; i < 10; i++) {
    odrv0.setPosition(2.0, 0.0f, 0.0f);
    delay(1000);
    odrv0.setPosition(-2.0, 0.0f, 0.0f);
    delay(1000);
  }
  Serial.println("Demo Test Complete.");
}