#include <math.h>

// ===================== Pin settings =====================
const int FORCE_PIN = A0; // 14

// ===================== Force calibration variables =====================
const float RM_OHMS = 10000.0;     // Your Rm value (e.g., 100k Ohms)
const float FORCE_SLOPE = 0.098;    // Calculated m
const float FORCE_INTERCEPT = -0.286;  // Calculated b

const int ADC_MAX = 1023;

int forceReading;
float forceN;

// ===================== Force conversion =====================
float forceAnalogToNewtons(int adc_val) {
  if (adc_val <= 0) {
    return 0.0; 
  }
  
  // 2. Calculate Sensor Resistance (Rs)
  // Casting to float to ensure decimal division
  float r_sensor = RM_OHMS * (((float)ADC_MAX / (float)adc_val) - 1.0);
  
  // 3. Calculate Conductance in micro-Siemens
  float conductance = 1000000.0 / r_sensor;
  
  // 4. Calculate Force
  float force_newtons = (FORCE_SLOPE * conductance) + FORCE_INTERCEPT;
  if (force_newtons < 0.0) {
    return 0.0;
  }
  
  return force_newtons;
}

// Get force
float getForce() {
  forceReading = analogRead(FORCE_PIN);
  forceN = forceAnalogToNewtons(forceReading);
  return forceN;
}

// Print force
void printForce() {
  Serial.print("Force_Raw:");
  Serial.print(forceReading);
  Serial.print(",");
  Serial.print("Force_N:");
  Serial.println(forceN, 2);
  Serial.println();
}