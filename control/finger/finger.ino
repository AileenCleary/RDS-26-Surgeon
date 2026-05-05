#include "Globals.h"

// Settings
const uint32_t UART_BAUDRATE = 115200;
const uint32_t SENSOR_READ_PERIOD_MS = 20; // 50Hz sensor polling

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
  // 1. Process incoming commands from the user/PC
  handleCommand();
  
  // 2. Generate smooth continuous motion if a SINE mode is active
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMotionTime) / 1000000.0f; // Convert microseconds to seconds
  lastMotionTime = currentMicros;

  updateMotion(dt);
  
  // 3. Non-blocking sensor reads
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime >= SENSOR_READ_PERIOD_MS) {
    lastSensorReadTime = currentTime;
    
    // Read and store sensor data (assuming these are defined in your sensor files)
    handleCommandMA782();
    // float force = getForce();
  }
}