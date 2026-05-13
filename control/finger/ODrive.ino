// ==============================================================================
// CONFIGURATION
// ==============================================================================
const int NUM_MOTORS = 5;
const uint32_t CAN_BAUDRATE = 250000;

// Hardware Limits 
const float VEL_LIMIT_TURNS_S = 100.0f;
const float I_SOFT_A = 2.0f;

// Gains
const float POS_GAIN = 3.0f;
const float VEL_GAIN = 0.01f;
const float VEL_INT_GAIN = 0.0f;

// Conversion & Offsets
const float DEGREES_PER_TURN = 14.054f;
float motor_zero_offsets[NUM_MOTORS] = {0.0f, 0.0f, 0.07f, 0.0f, 0.0f};


// 
// Safe homing parameters
const float HOME_DEADBAND_DEG = 1.0f;          // strict zero target
const float HOME_STEP_DEG = 4.0f;              // smaller step for safer, more precise homing
const float HOME_MIN_PROGRESS_DEG = 0.05f;     // allow small progress near zero
const int HOME_STUCK_LIMIT = 2;               // allow more slow steps before declaring stuck
const int HOME_STEP_DELAY_MS = 60;            // give encoder/motor time to settle
const int HOME_MAX_STEPS = 400;                // hard limit
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
void moveMotors(float angles[]);
void disableAllMotors();
bool safeHomeToZero(float startMotorAngles[]);
void setCurrentMotorPositionsAsZero();
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

float activeJointErrorSum(float joints[4]) {
  // Only use active joints for homing decision:
  // joints[0] = splay, joints[1] = MCP, joints[2] = PIP
  return fabs(joints[0]) + fabs(joints[1]) + fabs(joints[2]);
}

bool activeJointsNearZero(float joints[4]) {
  return fabs(joints[0]) < HOME_DEADBAND_DEG &&
         fabs(joints[1]) < HOME_DEADBAND_DEG &&
         fabs(joints[2]) < HOME_DEADBAND_DEG;
}

void setCurrentMotorPositionsAsZero() {
  Serial.println("Setting current COMMANDED motor positions as final safe motor zero...");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_feedback) {
      Serial.printf("WARNING: Motor %d has no feedback; zero not updated.\n", i);
      continue;
    }

    float commandedTurns = motor_zero_offsets[i] + (currentMotorTarget[i] / DEGREES_PER_TURN);

    motor_zero_offsets[i] = commandedTurns;
    currentMotorTarget[i] = 0.0f;
    odrives[i]->setPosition(commandedTurns, 0.0f, 0.0f);
    Serial.printf("SAFE ZERO: Motor %d zero = %.4f turns\n", i, motor_zero_offsets[i]);
  }
}

bool safeHomeToZero(float startMotorAngles[]) {
  Serial.println("===== SAFE HOMING TO ZERO (STRICT KINEMATIC LIMIT) =====");

  float cmd[NUM_MOTORS];

  // 1. Initialize holding commands
  for (int i = 0; i < NUM_MOTORS; i++) {
    cmd[i] = startMotorAngles[i];
    currentMotorTarget[i] = cmd[i];
    if (odrive_data[i].received_feedback) {
      odrives[i]->setPosition(odrive_data[i].last_feedback.Pos_Estimate, 0.0f, 0.0f);
    }
  }
  delay(300);
  pumpODriveCAN();

  for (int step = 0; step < HOME_MAX_STEPS; step++) {
    bool anyMotorMoving = false;

    // 2. SAFE STEPPING: 每个电机独立向 0 逼近，到了 0 就死死停住！
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (cmd[i] > HOME_STEP_DEG) {
        cmd[i] -= HOME_STEP_DEG;
        anyMotorMoving = true;
      } else if (cmd[i] < -HOME_STEP_DEG) {
        cmd[i] += HOME_STEP_DEG;
        anyMotorMoving = true;
      } else if (cmd[i] != 0.0f) {
        cmd[i] = 0.0f; // 精准停在理论 0 点
        anyMotorMoving = true;
      }
      currentMotorTarget[i] = cmd[i];
    }

    moveMotors(cmd);

    unsigned long t0 = millis();
    while (millis() - t0 < HOME_STEP_DELAY_MS) {
      pumpODriveCAN(); delay(5);
    }

    // 3. 读取当前关节状态
    float* jointsPtr = getJointAngles();
    float newJoints[4] = {jointsPtr[0], jointsPtr[1], jointsPtr[2], jointsPtr[3]};

    if (step % 5 == 0) {
      Serial.printf("Step %d | CMDs: M0:%.1f M1:%.1f M2:%.1f M3:%.1f M4:%.1f\n", 
                    step, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
    }

    // 4. 双重停止条件：要么关节到 0 了，要么所有电机的命令都走完（归零）了
    if (activeJointsNearZero(newJoints)) {
      Serial.println("SUCCESS: Joints reached 0 before motors finished.");
      setCurrentMotorPositionsAsZero();
      return true;
    }

    if (!anyMotorMoving) {
      Serial.println("SAFE STOP: All motors reached their kinematic 0 limits.");
      // 就算关节没完全到 0，也不准再拉了！保护肌腱！
      setCurrentMotorPositionsAsZero(); 
      return true;
    }
  }

  Serial.println("ERROR: Homing timed out.");
  disableAllMotors();
  return false;
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

  // // 2. CHECK FOR OTHER MOTORS
  // Serial.println("Checking for other connected ODrives (Waiting 5 seconds)...");
  // unsigned long t0 = millis();
  // while(millis() - t0 < 5000) {
  //   pumpODriveCAN();
  //   delay(5);
  // }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (odrive_data[i].received_heartbeat) {
      Serial.printf("Node %d is ALIVE.\n", i);
    } else {
      Serial.printf("Node %d is NOT CONNECTED (Skipping).\n", i);
    }
  }

  // for (int i = 0; i < NUM_MOTORS; i++) {
  //   if (odrive_data[i].received_feedback) {
  //     motor_zero_offsets[i] = odrive_data[i].last_feedback.Pos_Estimate;
  //     Serial.printf("Node %d Zero Offset set to: %.4f turns\n", i, motor_zero_offsets[i]);
  //   }
  // }

  Serial.println("Setting motor zero offsets to current power-on positions.");
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (odrive_data[i].received_feedback) {
        float current_odrive_turns = odrive_data[i].last_feedback.Pos_Estimate;
        motor_zero_offsets[i] = current_odrive_turns; // 直接将当前上电位置设为 0 点
        currentMotorTarget[i] = 0.0f;                 // 初始化目标位置为 0
        Serial.printf("Node %d Power-on Zero Offset set to: %.4f turns\n", i, motor_zero_offsets[i]);
      }
    }
    Serial.println();

  // 4. CONFIGURE AND REQUEST CLOSED LOOP
  Serial.println("Configuring Connected ODrives for Closed Loop Control...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue; // Skip missing motors

    odrives[i]->clearErrors();
    delay(50);
    pumpODriveCAN();

    odrives[i]->setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
    delay(20);
    pumpODriveCAN();

    odrives[i]->setLimits(VEL_LIMIT_TURNS_S, I_SOFT_A);
    delay(20);
    pumpODriveCAN();

    // odrives[i]->setPosGain(POS_GAIN);
    if (i == 0) {
      odrives[i]->setPosGain(3.0f);   // splay lower gain
    } else {
      odrives[i]->setPosGain(POS_GAIN);
    }
    delay(20);
    pumpODriveCAN();

    odrives[i]->setVelGains(VEL_GAIN, VEL_INT_GAIN);
    delay(20);
    pumpODriveCAN();

    odrives[i]->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);

    unsigned long closeLoopStart = millis();
    while (millis() - closeLoopStart < 1500) {
      pumpODriveCAN();

      if (odrive_data[i].last_heartbeat.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        break;
      }

      delay(10);
    }
    if (odrive_data[i].last_heartbeat.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL) {
      Serial.printf("✅ Node %d entered CLOSED LOOP during setup.\n", i);
    } else {
      Serial.printf("❌ Node %d failed to enter CLOSED LOOP during setup. State: %d | Error: 0x%08X\n",
                    i,
                    odrive_data[i].last_heartbeat.Axis_State,
                    odrive_data[i].last_heartbeat.Axis_Error);
    }
  }
  // for (int i = 0; i < NUM_MOTORS; i++) {
  //   odrives[i]->setPosition(motor_zero_offsets[i], 0.0f, 0.0f);
  // }


  // Do not auto-home in setup.
  // Use the serial HOME command after the system is ready.
  Serial.println("AUTO HOMING DISABLED. Type HOME to run safe homing.");

  // // TEMP TEST MODE FOR RE-WRAPPED TENDONS:
  // // Set each motor's CURRENT startup position as temporary motor zero.
  // // This means MOVE MOTOR i 0 will return to the startup position.
  // Serial.println("TEMP TEST MODE: Setting current motor positions as temporary zero.");

  // for (int i = 0; i < NUM_MOTORS; i++) {
  //   if (!odrive_data[i].received_feedback) {
  //     Serial.printf("WARNING: Node %d has no feedback; cannot set temporary zero.\n", i);
  //     continue;
  //   }

  //   float currentTurns = odrive_data[i].last_feedback.Pos_Estimate;

  //   motor_zero_offsets[i] = currentTurns;
  //   currentMotorTarget[i] = 0.0f;

  //   odrives[i]->setPosition(motor_zero_offsets[i], 0.0f, 0.0f);

  //   Serial.printf("TEMP ZERO: Motor %d zero = %.4f turns\n",
  //                 i, motor_zero_offsets[i]);
  // }

  delay(1000);

  // 5. VERIFY CLOSED LOOP STATE
  Serial.println("Verifying Axis States...");
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
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
  if (odrive_data[4].received_heartbeat) {
    float turns = motor_zero_offsets[4] + (angle / DEGREES_PER_TURN);
    odrv4.setPosition(turns, 0.0f, 0.0f);
  }
}

void moveMCPExt(float angle) {
  if (odrive_data[1].received_heartbeat) {
    float turns = motor_zero_offsets[1] + (angle / DEGREES_PER_TURN);
    odrv1.setPosition(turns, 0.0f, 0.0f);
  }
}

void movePIPFlex(float angle) {
  if (odrive_data[2].received_heartbeat) {
    float turns = motor_zero_offsets[2] + (angle / DEGREES_PER_TURN);
    odrv2.setPosition(turns, 0.0f, 0.0f);
  }
}

void movePIPExt(float angle) {
  if (odrive_data[3].received_heartbeat) {
    float turns = motor_zero_offsets[3] + (angle / DEGREES_PER_TURN);
    odrv3.setPosition(turns, 0.0f, 0.0f);
  }
}

void enableSingleMotor(int targetIdx) {
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    pumpODriveCAN();
    delay(5);
  }
  
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
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    pumpODriveCAN();
    delay(5);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue;
    
    // Wake up any motors that were previously put to sleep
    if (odrive_data[i].last_heartbeat.Axis_State != AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrives[i]->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
  }
}

void disableAllMotors() {
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    pumpODriveCAN();
    delay(5);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue;
    
    // Wake up any motors that were previously put to sleep
    if (odrive_data[i].last_heartbeat.Axis_State != AXIS_STATE_IDLE) {
      odrives[i]->setState(AXIS_STATE_IDLE);
    }
  }
}

void printMotorPositions() {
  Serial.println("===== Motor Positions =====");

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_feedback) {
      Serial.printf("Motor %d: no feedback received yet\n", i);
      continue;
    }

    float rawTurns = odrive_data[i].last_feedback.Pos_Estimate;
    float zeroTurns = motor_zero_offsets[i];
    float relTurns = rawTurns - zeroTurns;
    float relMotorDeg = relTurns * DEGREES_PER_TURN;

    Serial.printf(
      "Motor %d: raw=%.4f turns, zero=%.4f turns, rel=%.3f, target=%.3f\n",
      i,
      rawTurns,
      zeroTurns,
      relMotorDeg,
      currentMotorTarget[i]
    );
  }

  Serial.println("===========================");
}



void runSafeHomeCommand() {
  Serial.println("Manual HOME command received.");

  enableAllMotors();

  delay(500);
  pumpODriveCAN();

  bool allClosedLoop = true;

  Serial.println("Checking closed-loop states before HOME...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    pumpODriveCAN();
    delay(50);

    uint8_t state = odrive_data[i].last_heartbeat.Axis_State;
    uint32_t err = odrive_data[i].last_heartbeat.Axis_Error;

    if (state == AXIS_STATE_CLOSED_LOOP_CONTROL && err == 0) {
      Serial.printf("✅ Node %d ready for HOME.\n", i);
    } else {
      Serial.printf("❌ Node %d NOT ready for HOME. State: %d | Error: 0x%08X\n",
                    i, state, err);
      allClosedLoop = false;
    }
  }

  if (!allClosedLoop) {
    Serial.println("ERROR: Not all motors are in CLOSED LOOP. HOME cancelled.");
    return;
  }

  float motorAngles[NUM_MOTORS];
    float* jointAngles = getJointAngles();
    calculateMotorAngles(jointAngles, motorAngles);

    Serial.println("Re-aligning motor zero offsets to theoretical joint-zero...");
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (odrive_data[i].received_feedback) {
        float current_odrive_turns = odrive_data[i].last_feedback.Pos_Estimate;
        float expected_kinematic_turns = motorAngles[i] / DEGREES_PER_TURN;
        motor_zero_offsets[i] = current_odrive_turns - expected_kinematic_turns;
      }
      Serial.printf("Motor %d starting kinematic angle = %.2f deg\n", i, motorAngles[i]);
    }

    Serial.println("Starting SAFE HOME to zero...");
      bool success = safeHomeToZero(motorAngles); 

      if (success) {
        currentMode = MODE_IDLE; 
        for (int i = 0; i < 4; i++) currentJointTarget[i] = 0.0f;
        for (int i = 0; i < 5; i++) currentMotorTarget[i] = 0.0f;
        Serial.println("Motion targets reset to 0. Holding straight position.");
      }
}