#include "Globals.h"

const uint32_t UART_BAUDRATE = 115200;
unsigned long lastMotionTime = 0;
bool streamTelemetry = false;
unsigned long streamStartTime = millis();

void setup() {
  Serial.begin(UART_BAUDRATE);
  while (!Serial) {}

  setupMA782();
  setupODrive();
  
  currentMode = MODE_IDLE;
  lastMotionTime = millis();
  Serial.println("System Ready. Waiting for commands...");
}

void loop() {
  pumpODriveCAN();
  handleCommand();
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastMotionTime >= 20) {
    float dt = (currentMillis - lastMotionTime) / 1000.0f;
    lastMotionTime = currentMillis;
    
    updateMotion(dt);
  }
}