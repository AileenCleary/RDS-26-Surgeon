#include <SPI.h>
#include <math.h>
#include "Globals.h"

// ==============================================================================
// CONFIGURATION
// ==============================================================================
// CS Pins: SPLAY(0), MCP(1), PIP(2), DIP(3)
const int CS_PINS[NUM_ENC] = {10, 25, 36, 37}; 
const char* ENC_NAMES[NUM_ENC] = {"SPLAY", "MCP", "PIP", "DIP"};

// MA782 SPI Settings (1MHz, Mode 0 is standard for MA782)
const uint32_t SPI_HZ = 1000000;
const uint8_t SPI_MODE_USED = SPI_MODE0; 
SPISettings ma782_spi_settings(SPI_HZ, MSBFIRST, SPI_MODE_USED);

// State Variables
uint16_t raw_w[NUM_ENC];
float averagedJointDegs[NUM_ENC];
float jointDegs[NUM_ENC]; // Defined as extern in Globals.h

// Simple Moving Average Filter State
const int FILTER_SAMPLES = 20;
float filter_buffer[NUM_ENC][FILTER_SAMPLES];
int filter_index[NUM_ENC] = {0, 0, 0, 0};

// Calibration Offsets (Update these based on your physical zero positions)
float joint_zero_offsets[NUM_ENC] = {0.0f, 0.0f, 0.0f, 0.0f};

// ==============================================================================
// INITIALIZATION
// ==============================================================================
void setupMA782() {
  Serial.println("--- Initializing MA782 Encoders ---");
  SPI.begin();
  
  for (int i = 0; i < NUM_ENC; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH); // Deselect (Active Low)
    
    // Initialize filter buffers to 0
    for(int j = 0; j < FILTER_SAMPLES; j++) {
      filter_buffer[i][j] = 0.0f;
    }
  }
}

// ==============================================================================
// HARDWARE SPI READ
// ==============================================================================
uint16_t spiRead16(int cs_pin) {
  uint16_t result = 0;
  SPI.beginTransaction(ma782_spi_settings);
  digitalWrite(cs_pin, LOW);
  
  // MA782 requires sending 16 bits to read 16 bits
  result = SPI.transfer16(0x0000); 
  
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  return result;
}

// ==============================================================================
// DATA PROCESSING
// ==============================================================================
float computeMovingAverage(int sensor_idx, float new_val) {
  filter_buffer[sensor_idx][filter_index[sensor_idx]] = new_val;
  filter_index[sensor_idx] = (filter_index[sensor_idx] + 1) % FILTER_SAMPLES;
  
  float sum = 0.0f;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += filter_buffer[sensor_idx][i];
  }
  return sum / (float)FILTER_SAMPLES;
}

// Main function called by the rest of the system
float* getJointAngles() {
  for (int i = 0; i < NUM_ENC; i++) {
    // 1. Read Raw Sensor
    raw_w[i] = spiRead16(CS_PINS[i]);
    
    // 2. Convert 16-bit absolute angle (0-65535) to Degrees (0-360)
    float raw_deg = ((float)raw_w[i] / 65536.0f) * 360.0f;
    
    // 3. Apply Offset and Handle Wrap-around
    float jointDeg = raw_deg - joint_zero_offsets[i];
    if (jointDeg > 180.0f) jointDeg -= 360.0f;
    if (jointDeg < -180.0f) jointDeg += 360.0f;
    
    // 4. Filter and Store
    jointDegs[i] = jointDeg;
    averagedJointDegs[i] = computeMovingAverage(i, jointDeg);
  }

  return averagedJointDegs;
}