#include <math.h>
#include "Globals.h"

// ==============================================================================
// CONFIGURATION
// ==============================================================================
const int FORCE_PIN = A0; // Teensy analog pin 14

// Force Calibration Constants
const float RM_OHMS = 10000.0f;        // 100k Ohm resistor
const float FORCE_SLOPE = 0.098f;      // Calibration 'm'
const float FORCE_INTERCEPT = -0.286f; // Calibration 'b'
const int ADC_MAX = 1023;

// ==============================================================================
// SENSOR READ FUNCTION
// ==============================================================================
float getForce() {
  int adc_val = analogRead(FORCE_PIN);
  
  // Prevent divide-by-zero or negative readings
  if (adc_val <= 0 || adc_val >= ADC_MAX) {
    return 0.0f; 
  }
  
  // 1. Calculate Sensor Resistance (Rs)
  float r_sensor = RM_OHMS * (((float)ADC_MAX / (float)adc_val) - 1.0f);
  
  // 2. Calculate Conductance in micro-Siemens
  float conductance = 1000000.0f / r_sensor;
  
  // 3. Calculate Force in Newtons
  float force_newtons = (FORCE_SLOPE * conductance) + FORCE_INTERCEPT;
  
  // Clamp negative force values to 0
  if (force_newtons < 0.0f) {
    return 0.0f;
  }
  
  return force_newtons;
}