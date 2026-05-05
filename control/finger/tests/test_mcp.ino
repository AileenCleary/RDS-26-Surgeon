#pragma once

#include "../ODrive.ino"
#include "../Kinematics.ino"

// Settings
const uint32_t UART_BAUDRATE = 115200;
const uint32_t PERIOD_MS = 200;

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

void testJointMCP() {
  Serial.println("\n--- TESTING MCP JOINT ---\n");
  
  // Note: MCP only flexes in the negative direction based on your constraints
  Serial.println("Flexing MCP to -45 degrees...");
  moveJointsSafely(0.0f, -45.0f, 0.0f);
  delay(1500);
  
  Serial.println("Flexing MCP to -90 degrees...");
  moveJointsSafely(0.0f, -90.0f, 0.0f);
  delay(1500);
  
  resetToZero();
}

void setup() {
  Serial.begin(UART_BAUDRATE);
  while (!Serial) {}

  setupODrive();
  testJointMCP();
}

void loop() {
}