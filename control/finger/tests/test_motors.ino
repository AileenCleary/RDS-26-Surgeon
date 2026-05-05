#pragma once

#include "../ODrive.ino"

// Settings
const uint32_t UART_BAUDRATE = 115200;
const uint32_t PERIOD_MS = 200;

void resetToZero() {
  float zeroMotors[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  moveMotors(zeroMotors);
  Serial.println("Hardware returned to Zero (Fully Extended).");
}

void testSplayMotor() {
  Serial.printf("\n--- TESTING SPLAY MOTOR ---\n");
  Serial.println("Moving +15 degrees...");
  
  moveSplay(15.0f);
  delay(1000); // Wait 1 second for visual verification
  
  moveSplay(-15.0f);
  delay(1000);
  
  resetToZero();
}

void testMCPFlEXMotor() {
  Serial.printf("\n--- TESTING MCP FlEX MOTOR ---\n");
  Serial.println("Moving +15 degrees...");
  
  moveMCPFlex(15.0f);
  delay(1000); // Wait 1 second for visual verification
  
  moveMCPFlex(-15.0f);
  delay(1000);
  
  resetToZero();
}

void testMCPEXTMotor() {
  Serial.printf("\n--- TESTING MCP EXT MOTOR ---\n");
  Serial.println("Moving +15 degrees...");
  
  moveMCPExt(15.0f);
  delay(1000); // Wait 1 second for visual verification
  
  moveMCPExt(-15.0f);
  delay(1000);
  
  resetToZero();
}

void testPIPFLEXMotor() {
  Serial.printf("\n--- TESTING PIP FLEX MOTOR ---\n");
  Serial.println("Moving +15 degrees...");
  
  movePIPFlex(15.0f);
  delay(1000); // Wait 1 second for visual verification
  
  movePIPFlex(-15.0f);
  delay(1000);
  
  resetToZero();
}

void testPIPEXTMotor() {
  Serial.printf("\n--- TESTING PIP EXT MOTOR ---\n");
  Serial.println("Moving +15 degrees...");
  
  movePIPExt(15.0f);
  delay(1000); // Wait 1 second for visual verification
  
  movePIPExt(-15.0f);
  delay(1000);
  
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

void setup() {
  Serial.begin(UART_BAUDRATE);
  while (!Serial) {}

  setupODrive();
  testSplayMotor();
  testMCPFlEXMotor();
  testMCPEXTMotor();
  testPIPFLEXMotor();
  testPIPEXTMotor();
}

void loop() {
}