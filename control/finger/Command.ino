#include "Globals.h"

void handleCommand() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); // Remove whitespace/newlines
  cmd.toUpperCase(); // Make parsing case-insensitive

  if (cmd == "STOP") {
    currentMode = MODE_IDLE;
    Serial.println("ACK: Stopped continuous motion.");
    return;
  }

  // Buffers for parsing
  float v0, v1, v2, v3, v4; 
  char type[10];
  
  // ---------------------------------------------------------
  // 1. MOVE COMMANDS (Single point)
  // Format: MOVE <LEVEL> <V0> <V1> <V2> ...
  // ---------------------------------------------------------
  if (cmd.startsWith("MOVE ")) {
    currentMode = MODE_IDLE; // Stop any active sine waves
    
    if (sscanf(cmd.c_str(), "MOVE TIP %f %f %f", &v0, &v1, &v2) == 3) {
      currentTipTarget[0] = v0; currentTipTarget[1] = v1; currentTipTarget[2] = v2;
      
      float jointsOut[4], motorsOut[5];
      calculateJointAngles(currentTipTarget, jointsOut);
      calculateMotorAngles(jointsOut, motorsOut);
      moveMotors(motorsOut);
      Serial.printf("ACK: Moved TIP to %.1f, %.1f, %.1f\n", v0, v1, v2);
    } 
    else if (sscanf(cmd.c_str(), "MOVE JOINT %f %f %f %f", &v0, &v1, &v2, &v3) == 4) {
      currentJointTarget[0] = v0; currentJointTarget[1] = v1; 
      currentJointTarget[2] = v2; currentJointTarget[3] = v3;
      
      float motorsOut[5];
      calculateMotorAngles(currentJointTarget, motorsOut);
      moveMotors(motorsOut);
      Serial.printf("ACK: Moved JOINTS to %.1f, %.1f, %.1f, %.1f\n", v0, v1, v2, v3);
    }
    else if (sscanf(cmd.c_str(), "MOVE MOTOR %f %f %f %f %f", &v0, &v1, &v2, &v3, &v4) == 5) {
      currentMotorTarget[0] = v0; currentMotorTarget[1] = v1; currentMotorTarget[2] = v2; 
      currentMotorTarget[3] = v3; currentMotorTarget[4] = v4;
      
      moveMotors(currentMotorTarget);
      Serial.println("ACK: Moved MOTORS directly.");
    }
  }
  
  // ---------------------------------------------------------
  // 2. SINE COMMANDS (Continuous path)
  // Format: SINE <LEVEL> <AXIS_IDX> <AMPLITUDE> <FREQUENCY> <OFFSET>
  // ---------------------------------------------------------
  else if (cmd.startsWith("SINE ")) {
    int axis;
    if (sscanf(cmd.c_str(), "SINE %s %d %f %f %f", type, &axis, &v0, &v1, &v2) == 5) {
      sineAxis = axis;
      sineAmp = v0;
      sineFreq = v1;
      sineOffset = v2;
      motionStartTime = millis();
      
      if (strcmp(type, "TIP") == 0) currentMode = MODE_SINE_TIP;
      else if (strcmp(type, "JOINT") == 0) currentMode = MODE_SINE_JOINT;
      else if (strcmp(type, "MOTOR") == 0) currentMode = MODE_SINE_MOTOR;
      
      Serial.printf("ACK: Started SINE on %s axis %d (Amp:%.1f, Freq:%.1f, Offset:%.1f)\n", type, axis, v0, v1, v2);
    }
  }

  // ---------------------------------------------------------
  // 3. TRAJECTORY STREAMING (Fast passthrough from Python script)
  // Format: TRAJ <LEVEL> <V0> <V1> ...
  // ---------------------------------------------------------
  else if (cmd.startsWith("TRAJ ")) {
    currentMode = MODE_TRAJ_STREAMING;
    // Fast path calculation without printing ACKs back to Serial to avoid bottlenecking

    if (sscanf(cmd.c_str(), "TRAJ TIP %f %f %f", &v0, &v1, &v2) == 3) {
      float dynamicTip[3] = {v0, v1, v2};
      float jointsOut[4], motorsOut[5];
      calculateJointAngles(dynamicTip, jointsOut);
      calculateMotorAngles(jointsOut, motorsOut);
      moveMotors(motorsOut);
    } 
    else if (sscanf(cmd.c_str(), "TRAJ JOINT %f %f %f %f", &v0, &v1, &v2, &v3) == 4) {
      float dynamicJoints[4] = {v0, v1, v2, v3};
      float motorsOut[5];
      calculateMotorAngles(dynamicJoints, motorsOut);
      moveMotors(motorsOut);
    }
    else if (sscanf(cmd.c_str(), "TRAJ MOTOR %f %f %f %f %f", &v0, &v1, &v2, &v3, &v4) == 5) {
      float dynamicMotors[5] = {v0, v1, v2, v3, v4};
      moveMotors(dynamicMotors);
    }
  }
  else if (cmd == "PID ON") {
    feedbackEnabled = true;
    resetPIDs(); // Crucial: clear old integral windup before starting
    Serial.println("ACK: Feedback PID Control ENABLED.");
  }
  else if (cmd == "PID OFF") {
    feedbackEnabled = false;
    Serial.println("ACK: Feedback PID Control DISABLED.");
  }
  // Format: TUNE PID <AXIS_IDX> <P> <I> <D>
  else if (cmd.startsWith("TUNE PID ")) {
    int axis; 
    float p, i, d;
    if (sscanf(cmd.c_str(), "TUNE PID %d %f %f %f", &axis, &p, &i, &d) == 4) {
      if (axis >= 0 && axis < 3) {
        jointPIDs[axis].Kp = p;
        jointPIDs[axis].Ki = i;
        jointPIDs[axis].Kd = d;
        jointPIDs[axis].reset();
        Serial.printf("ACK: Tuned PID Axis %d to P:%.3f I:%.4f D:%.3f\n", axis, p, i, d);
      }
    }
  }
  else if (cmd.startsWith("TEST ")) {
    currentMode = MODE_IDLE; // Ensure any background motion is safely stopped
    
    if (cmd == "TEST MOTORS") {
      testAllMotorsTogether();
    }
    else if (cmd == "TEST SPLAY") {
      testJointSplay();
    }
    else if (cmd == "TEST MCP") {
      testJointMCP();
    }
    else if (cmd == "TEST PIP") {
      testJointPIP();
    }
    else if (cmd == "TEST KINEMATICS") {
      testKinematics();
    }
    else if (cmd == "TEST FORCE") {
      testForceSensor();
    }
    else if (cmd == "TEST ENCODERS") {
      testJointSensors();
    }
    else if (cmd == "TEST DEMO") {
      testDemo();
    } else {
      // Parse individual motor test (e.g., "TEST MOTOR 2")
      int mIdx;
      if (sscanf(cmd.c_str(), "TEST MOTOR %d", &mIdx) == 1) {
        testSingleMotor(mIdx);
      }
    }
  }
}