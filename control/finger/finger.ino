#include "Globals.h"

const uint32_t UART_BAUDRATE = 115200;
unsigned long lastMotionTime = 0;
bool streamTelemetry = false;
bool streamForceTelemetry = false;
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
  float* joints = getJointAngles(); // Capture the pointer so we can print it
  getForce();
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastMotionTime >= 20) {
    float dt = (currentMillis - lastMotionTime) / 1000.0f;
    lastMotionTime = currentMillis;
    
    updateMotion(dt);
    
    if (streamTelemetry) {
      float t = (currentMillis - streamStartTime) / 1000.0f;
      if (currentMode == MODE_CONTROL_MOTOR) {
        float actual_motors[NUM_MOTORS] = {
          odrive_data[0].last_feedback.Pos_Estimate - motor_zero_offsets[0],
          odrive_data[1].last_feedback.Pos_Estimate - motor_zero_offsets[1],
          odrive_data[2].last_feedback.Pos_Estimate - motor_zero_offsets[2],
          odrive_data[3].last_feedback.Pos_Estimate - motor_zero_offsets[3],
          odrive_data[4].last_feedback.Pos_Estimate - motor_zero_offsets[4]
        };
        Serial.printf("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        t, currentMotorTarget[0], currentMotorTarget[1], currentMotorTarget[2], currentMotorTarget[3], currentMotorTarget[4],
        actual_motors[0], actual_motors[1], actual_motors[2], actual_motors[3], actual_motors[4]);
      }
      else {
        if (feedbackEnabled) {
          Serial.printf("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
          t, currentJointTarget[0], currentJointTarget[1], currentJointTarget[2], currentJointTarget[3],
          joints[0], joints[1], joints[2], joints[3]);
        } else {
          float estimatedJoints[NUM_ENC];
          estimateJointAnglesFromMotors(estimatedJoints);
          Serial.printf("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
          t, currentJointTarget[0], currentJointTarget[1], currentJointTarget[2], currentJointTarget[3],
          estimatedJoints[0], estimatedJoints[1], estimatedJoints[2], estimatedJoints[3]);
        }
      }
    }

    if (streamForceTelemetry) {
      float actual_force = useForceSensor ? getForce() : getEstimatedTipForceScalar();
      float t_sec = (millis() - streamStartTime) / 1000.0f;
      Serial.printf("%.3f,%.3f,%.3f\n", t_sec, currentForceTarget, actual_force);
    }
  }
}