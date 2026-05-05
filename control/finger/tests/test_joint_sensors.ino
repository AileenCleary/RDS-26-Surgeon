#pragma once

#include "../MA782.ino"

// Settings
const uint32_t  UART_BAUDRATE = 115200;
const uint32_t PERIOD_MS = 200;

void setup() {
  Serial.begin(UART_BAUDRATE);
  while (!Serial) {}

  setupMA782();
}

void loop() {
  handleCommandMA782();
  
  float* thetaList = getJointAngles();
  printJointAngles();
  delay(PERIOD_MS);
}