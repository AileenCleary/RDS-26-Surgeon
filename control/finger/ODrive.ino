#include <FlexCAN_T4.h>

// Config
const int NUM_MOTORS = 5;
const uint32_t CAN_BAUDRATE  = 250000;
const uint8_t ODRV0_NODE_IDS[NUM_MOTORS] = {0, 1, 2, 3, 4};

// ODrive CAN Simple Command IDs
const uint32_t CMD_SET_AXIS_STATE = 0x07;
const uint32_t CMD_SET_CONTROLLER_MODE = 0x0B;
const uint32_t CMD_SET_INPUT_POS = 0x0C;
const uint32_t CMD_SET_LIMITS = 0x0F;
const uint32_t CMD_SET_POS_GAIN = 0x01A;
const uint32_t CMD_SET_VEL_GAINS = 0x01B;

// Gains
const float POS_GAIN = 0.2f;
const float VEL_GAIN = 0.01f;
const float VEL_INT_GAIN = 0.0f;

// Constraints
const float MAX_TORQUE = 0.05; // N-m
const float MAX_POWER = 100; // W
const float MOTOR_KT = 0.049f;  // Nm/A (From Motor KV = 167.5)
const float LIMIT_CURRENT = MAX_TORQUE / MOTOR_KT;
const float MAX_POWER_PER_MOTOR = MAX_POWER / 3;  // max motors used at a time
const float MAX_VEL_RAD_S = MAX_POWER_PER_MOTOR / MAX_TORQUE;
const float LIMIT_VELOCITY = MAX_VEL_RAD_S / (2.0f * PI);

// Can Object
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// Setup function
void setupODrive() {
  can1.begin();
  can1.setBaudRate(CAN_BAUDRATE);
  delay(3000);

  Serial.println("Initializing ODrives...");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint32_t id = ODRV0_NODE_IDS[i];
    sendCANFloat(id, CMD_SET_POS_GAIN, POS_GAIN);
    sendCANFloatFloat(id, CMD_SET_VEL_GAINS, VEL_GAIN, VEL_INT_GAIN);
    sendCANFloatFloat(id, CMD_SET_LIMITS, LIMIT_VELOCITY, LIMIT_CURRENT);
    sendCANIntInt(id, CMD_SET_CONTROLLER_MODE, 3, 1);
    sendCANInt(id, CMD_SET_AXIS_STATE, 8);
    delay(50); 
  }
  Serial.println("All 5 ODrives set to Closed Loop Position Control.");
}

// Functions to move individual motors
void moveSplay(float angle) {
  float turns = angle / 360.0f;
  sendCANPosition(ODRV0_NODE_IDS[0], turns, 0.0f, 0.0f);
}

void moveMCPFlex(float angle) {
  float turns = angle / 360.0f;
  sendCANPosition(ODRV0_NODE_IDS[1], turns, 0.0f, 0.0f);
}

void moveMCPExt(float angle) {
  float turns = angle / 360.0f;
  sendCANPosition(ODRV0_NODE_IDS[2], turns, 0.0f, 0.0f);
}

void movePIPFlex(float angle) {
  float turns = angle / 360.0f;
  sendCANPosition(ODRV0_NODE_IDS[3], turns, 0.0f, 0.0f);
}

void movePIPExt(float angle) {
  float turns = angle / 360.0f;
  sendCANPosition(ODRV0_NODE_IDS[4], turns, 0.0f, 0.0f);
}

// Function to move all 5 motors based on an array of angles in degrees
void moveMotors(float angles[5]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    float turns = angles[i] / 360.0f;
    sendCANPosition(ODRV0_NODE_IDS[i], turns, 0.0f, 0.0f);
  }
}

// CAN Helper functions
void sendCANPosition(uint32_t node, float pos, float vel_ff, float torque_ff) {
  CAN_message_t msg;
  msg.id = (node << 5) | CMD_SET_INPUT_POS;
  msg.len = 8;
  
  int16_t v_ff = (int16_t)(vel_ff * 1000.0f);
  int16_t t_ff = (int16_t)(torque_ff * 1000.0f);
  
  memcpy(&msg.buf[0], &pos, 4);
  memcpy(&msg.buf[4], &v_ff, 2);
  memcpy(&msg.buf[6], &t_ff, 2);
  
  can1.write(msg);
}

void sendCANFloatFloat(uint32_t node, uint32_t cmd_id, float val1, float val2) {
  CAN_message_t msg;
  msg.id = (node << 5) | cmd_id;
  msg.len = 8;
  memcpy(&msg.buf[0], &val1, 4);
  memcpy(&msg.buf[4], &val2, 4);
  can1.write(msg);
}

void sendCANFloat(uint32_t node, uint32_t cmd_id, float val1) {
  CAN_message_t msg;
  msg.id = (node << 5) | cmd_id;
  msg.len = 8;
  memcpy(&msg.buf[0], &val1, 4);
  can1.write(msg);
}

void sendCANIntInt(uint32_t node, uint32_t cmd_id, int32_t val1, int32_t val2) {
  CAN_message_t msg;
  msg.id = (node << 5) | cmd_id;
  msg.len = 8;
  memcpy(&msg.buf[0], &val1, 4);
  memcpy(&msg.buf[4], &val2, 4);
  can1.write(msg);
}

void sendCANInt(uint32_t node, uint32_t cmd_id, int32_t val1) {
  CAN_message_t msg;
  msg.id = (node << 5) | cmd_id;
  msg.len = 4;
  memcpy(&msg.buf[0], &val1, 4);
  can1.write(msg);
}