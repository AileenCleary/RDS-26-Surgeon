#pragma once

#include "../A101.ino"

// Settings
const uint32_t  UART_BAUDRATE = 115200;
const uint32_t PERIOD_MS = 200;

void setup() {
  Serial.begin(UART_BAUDRATE);
  while (!Serial) {}
}

void loop() {
  float force = getForce();
  printForce();
  delay(PERIOD_MS);
}