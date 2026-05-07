// ==============================================================================
// CONFIGURATION
// ==============================================================================
const int NUM_MOTORS = 5;
const uint32_t CAN_BAUDRATE = 250000;

// Hardware Limits 
const float VEL_LIMIT_TURNS_S = 100.0f;
const float I_SOFT_A = 2.63f;

// Gains
const float POS_GAIN = 5.0f;
const float VEL_GAIN = 0.01f;
const float VEL_INT_GAIN = 0.0f;

// Conversion & Offsets
const float DEGREES_PER_TURN = 14.12f;
float motor_zero_offsets[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// ==============================================================================
// ODRIVE OBJECTS & CAN SETUP
// ==============================================================================
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can1;

ODriveCAN odrv0(wrap_can_intf(can1), 0);
ODriveCAN odrv1(wrap_can_intf(can1), 1);
ODriveCAN odrv2(wrap_can_intf(can1), 2);
ODriveCAN odrv3(wrap_can_intf(can1), 3);
ODriveCAN odrv4(wrap_can_intf(can1), 4);

ODriveCAN* odrives[NUM_MOTORS] = { &odrv0, &odrv1, &odrv2, &odrv3, &odrv4 };

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};
ODriveUserData odrive_data[NUM_MOTORS];

// ==============================================================================
// CALLBACKS
// ==============================================================================
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  auto* ud = static_cast<ODriveUserData*>(user_data);
  ud->last_heartbeat = msg;
  ud->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  auto* ud = static_cast<ODriveUserData*>(user_data);
  ud->last_feedback = msg;
  ud->received_feedback = true;
}

void onCanMessage(const CAN_message_t& msg) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    onReceive(msg, *odrives[i]);
  }
}

// Helper to keep the CAN bus flowing
void pumpODriveCAN() {
  pumpEvents(can1);
}

// ==============================================================================
// MAIN FUNCTIONS
// ==============================================================================

void setupODrive() {
  Serial.println("\n--- Initializing CAN Bus ---");
  can1.begin();
  can1.setBaudRate(CAN_BAUDRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(onCanMessage);

  // Attach callbacks for all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    odrives[i]->onStatus(onHeartbeat, &odrive_data[i]);
    odrives[i]->onFeedback(onFeedback, &odrive_data[i]);
  }

  // 1. INDEFINITE WAIT FOR MOTORS THAT ARE BEING TESTED
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.printf("Waiting for ODrive Node %d heartbeat (Teensy will wait here until ODrive boots)...\n", i);
    while (!odrive_data[i].received_heartbeat) {
      pumpODriveCAN();
      delay(10);
    }
    Serial.printf("Node %d Heartbeat OK! ODrive is awake.\n", i);
  }

  // 2. CHECK FOR OTHER MOTORS
  Serial.println("Checking for other connected ODrives (Waiting 5 seconds)...");
  unsigned long t0 = millis();
  while(millis() - t0 < 5000) {
    pumpODriveCAN();
    delay(5);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (odrive_data[i].received_heartbeat) {
      Serial.printf("Node %d is ALIVE.\n", i);
    } else {
      Serial.printf("Node %d is NOT CONNECTED (Skipping).\n", i);
    }
  }

  // 3. CAPTURE STARTING POSITIONS AS RELATIVE ZERO
  Serial.println("Waiting for initial encoder feedback to set zero offsets...");
  t0 = millis();
  while (millis() - t0 < 2000) {
    pumpODriveCAN();
    bool all_ready = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (odrive_data[i].received_heartbeat && !odrive_data[i].received_feedback) {
        all_ready = false;
      }
    }
    if (all_ready) break;
    delay(5);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (odrive_data[i].received_feedback) {
      motor_zero_offsets[i] = odrive_data[i].last_feedback.Pos_Estimate;
      Serial.printf("Node %d Zero Offset set to: %.4f turns\n", i, motor_zero_offsets[i]);
    }
  }

  // 4. CONFIGURE AND REQUEST CLOSED LOOP
  Serial.println("Configuring Connected ODrives for Closed Loop Control...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue; // Skip missing motors
    
    odrives[i]->clearErrors();
    delay(10);
    odrives[i]->setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrives[i]->setLimits(VEL_LIMIT_TURNS_S, I_SOFT_A);
    odrives[i]->setPosGain(POS_GAIN);
    odrives[i]->setVelGains(VEL_GAIN, VEL_INT_GAIN);
    delay(10);
    
    // Command the motor to strictly hold its current relative zero position before engaging
    odrives[i]->setPosition(motor_zero_offsets[i], 0.0f, 0.0f);
    delay(10);
    
    odrives[i]->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  // 5. VERIFY CLOSED LOOP STATE
  Serial.println("Verifying Axis States...");
  t0 = millis();
  while (millis() - t0 < 1000) {
    pumpODriveCAN();
    delay(5);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue;
    
    uint8_t state = odrive_data[i].last_heartbeat.Axis_State;
    uint32_t err = odrive_data[i].last_heartbeat.Axis_Error;
    
    if (state == AXIS_STATE_CLOSED_LOOP_CONTROL && err == 0) {
      Serial.printf("✅ Node %d entered CLOSED LOOP.\n", i);
    } else {
      Serial.printf("❌ Node %d REJECTED Closed Loop! State: %d | Error: 0x%08X\n", i, state, err);
    }
  }
  
  Serial.println("--- ODrive Setup Complete --- \n");
}

// ==============================================================================
// MOVEMENT COMMANDS
// ==============================================================================

void moveMotors(float angles[5]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue; 
    
    float turns = motor_zero_offsets[i] + (angles[i] / DEGREES_PER_TURN);
    odrives[i]->setPosition(turns, 0.0f, 0.0f);
  }
}

void moveSplay(float angle) {
  if (odrive_data[0].received_heartbeat) {
    float turns = motor_zero_offsets[0] + (angle / DEGREES_PER_TURN);
    odrv0.setPosition(turns, 0.0f, 0.0f);
  }
}

void moveMCPFlex(float angle) {
  if (odrive_data[1].received_heartbeat) {
    float turns = motor_zero_offsets[1] + (angle / DEGREES_PER_TURN);
    odrv1.setPosition(turns, 0.0f, 0.0f);
  }
}

void moveMCPExt(float angle) {
  if (odrive_data[2].received_heartbeat) {
    float turns = motor_zero_offsets[2] + (angle / DEGREES_PER_TURN);
    odrv2.setPosition(turns, 0.0f, 0.0f);
  }
}

void movePIPFlex(float angle) {
  if (odrive_data[3].received_heartbeat) {
    float turns = motor_zero_offsets[3] + (angle / DEGREES_PER_TURN);
    odrv3.setPosition(turns, 0.0f, 0.0f);
  }
}

void movePIPExt(float angle) {
  if (odrive_data[4].received_heartbeat) {
    float turns = motor_zero_offsets[4] + (angle / DEGREES_PER_TURN);
    odrv4.setPosition(turns, 0.0f, 0.0f);
  }
}

void enableSingleMotor(int targetIdx) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue;
    
    if (i == targetIdx) {
      // Ensure the target motor is awake and tracking
      if (odrive_data[i].last_heartbeat.Axis_State != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrives[i]->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      }
    } else {
      // Put all other motors to sleep (freewheeling/limp)
      if (odrive_data[i].last_heartbeat.Axis_State != AXIS_STATE_IDLE) {
        odrives[i]->setState(AXIS_STATE_IDLE);
      }
    }
  }
}

void enableAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue;
    
    // Wake up any motors that were previously put to sleep
    if (odrive_data[i].last_heartbeat.Axis_State != AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrives[i]->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
  }
}