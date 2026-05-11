#include "Globals.h"

// Settings
const uint32_t UART_BAUDRATE = 115200;
const uint32_t SENSOR_READ_PERIOD_MICROS = 1; // 1MHz sensor polling

unsigned long lastSensorReadTime = 0;
unsigned long lastMotionTime = 0;

void setup() {
  Serial.begin(UART_BAUDRATE);
  while (!Serial) {}

  setupMA782();
  setupODrive();
  
  // Set initial default target to extended finger
  currentMode = MODE_IDLE;
  lastMotionTime = micros();
  Serial.println("System Ready. Waiting for commands...");
}

void loop() {
  pumpODriveCAN();
  
  // 1. Process incoming commands from the user/PC
  handleCommand();
  
  // 2. Generate smooth continuous motion if a SINE mode is active
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMotionTime) / 1000000.0f; // Convert microseconds to seconds
  lastMotionTime = currentMicros;

  updateMotion(dt);
  
  // 3. Non-blocking sensor reads
  if (currentMicros - lastSensorReadTime >= SENSOR_READ_PERIOD_MICROS) {
    lastSensorReadTime = currentMicros;
    getJointAngles();
    getForce();
  }

  if (streamTelemetry) {
    static unsigned long lastStreamTime = 0;
    if (millis() - lastStreamTime >= 20) { // 50Hz update rate
      lastStreamTime = millis();
      float t = (millis() - streamStartTime) / 1000.0f;
      
      // Output: Time, Target Angle, Actual Angle (Using PIP/Axis 2 as an example)
      Serial.printf("%.3f,%.2f,%.2f\n", t, currentJointTarget[2], jointDegs[2]);
    }
  }
}