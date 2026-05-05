#pragma once

#include "../Kinematics.ino"

// Settings
const uint32_t  UART_BAUDRATE = 115200;

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

void setup() {
  Serial.begin(UART_BAUDRATE);
  while (!Serial) {}

  Serial.println("================= KINEMATICS TESTS =================");

  // Test 1: Inverse Kinematics for Tip (0, 0, 0)
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

  // Tests 2-7: Forward and Tendon Kinematics from given Joint Angles
  runJointTest(0.0f,   0.0f,   0.0f,   0.0f);
  runJointTest(10.0f,  0.0f,   0.0f,   0.0f);
  runJointTest(-10.0f, 0.0f,   0.0f,   0.0f);
  
  // Note: -62.8 is very close to the theoretical -63.5 derived from your DIP_COUPLING_RATIO
  runJointTest(0.0f,   -90.0f, -90.0f, -62.8f);
  runJointTest(10.0f,  -90.0f, -90.0f, -62.8f);
  runJointTest(-10.0f, -90.0f, -90.0f, -62.8f);
  
  Serial.println("\n====================== DONE ======================");
}

void loop() {
}