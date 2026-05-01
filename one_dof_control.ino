#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <AS5X47.h>
#include <EEPROM.h>
#ifdef CAN_ERROR_BUS_OFF
  #undef CAN_ERROR_BUS_OFF
#endif

#define IS_TEENSY_BUILTIN
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"

// ------------------------- User config -------------------------
static constexpr uint32_t CAN_BAUDRATE  = 250000;
static constexpr uint8_t  ODRV0_NODE_ID = 0;
static volatile int log_mark = 0;   // NEW: set to 1 for one-sample marker
/*
  你给的关系： joint_deg = motor_deg * 2/3
  => motor_deg = joint_deg * 3/2
  => motor_turns = joint_deg/360 * 1.5
*/
static constexpr float MOTOR_TURNS_PER_JOINT_TURN = 1.5f;

// ROM=90deg, 75% peak-to-peak => amplitude = 0.75*90/2 = 33.75deg
static constexpr float ROM_DEG       = 90.0f;
static constexpr float JOINT_MIN_DEG = -90.0f;
static constexpr float JOINT_MAX_DEG = 90.0f;
static constexpr float SINE_FREQ_HZ  = 1.0f;
static constexpr float SINE_AMP_DEG  = 0.75f * ROM_DEG * 0.5f;  // 33.75

// 控制参数
static constexpr float POS_GAIN     = 10.0f;
static constexpr float VEL_GAIN     = 0.01f;
static constexpr float VEL_INT_GAIN = 0.0001f;

static constexpr float ENC_KP = 0.05f;
static constexpr float ENC_KD = 0.0f;
static constexpr float ENC_KI = 0.0001f;

float prev_error = 0.0f;
float error_i = 0.0f;
float prev_pos;
float prev_vel;
float dt = 5.0;
float r = 45.27;
float m = 184;
float g = 9.81;

// 更安全：够跑2Hz正弦，又不至于一下子飞太快
static float VEL_LIMIT_TURNS_S = 10.0f;
static constexpr float I_SOFT_A = 3.0f;

// 正弦振幅爬升，避免刚进闭环瞬间猛转
static constexpr float AMP_RAMP_TIME_S = 0.6f;

static constexpr float LOOP_HZ = 200.0f;
// --------------------------------------------------------------


// ------------------------- CAN + ODrive objects -------------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN* odrives[] = { &odrv0 };
AS5X47 as5047(10);

// ------------------------- User data -------------------------
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;

  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};
ODriveUserData odrv0_user_data;

// ------------------------- Callbacks -------------------------
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

void onCanMessage(const CanMsg& msg) {
  for (auto* odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

// ------------------------- Helpers -------------------------
static inline float joint_deg_to_motor_turns(float joint_deg) {
  return (joint_deg / 360.0f) * MOTOR_TURNS_PER_JOINT_TURN;
}

static inline float motor_turns_to_joint_deg(float motor_turns) {
  return (motor_turns / MOTOR_TURNS_PER_JOINT_TURN) * 360.0f;
}

static inline void pump_can_for_ms(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    pumpEvents(can_intf);
    delay(1);
  }
}

static bool setupCAN() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

static void print_help() {
  Serial.println();
  Serial.println("=== Commands (newline end) ===");
  Serial.println("h              : help");
  Serial.println("c              : ENTER closed-loop control (hold current pos, no jump)");
  Serial.println("i              : EXIT to IDLE (disable control)");
  Serial.println("z              : set software zero (current joint angle -> 0 deg)");
  Serial.println("p <deg>        : go to JOINT angle (deg) relative to zero, e.g.  p 30");
  Serial.println("s              : start sine (1Hz, amp=33.75deg around 0deg), ramp up");
  Serial.println("x              : stop sine, hold current position");
  Serial.println("g              : print gains/limits");
  Serial.println("L <turn/s>     : set vel_limit, e.g.  L 5");
  Serial.println("  a                : print current joint angle (deg)");
  Serial.println();
}

static bool apply_controller_config() {
  bool ok = true;
  ok &= odrv0.setControllerMode((uint8_t)CONTROL_MODE_POSITION_CONTROL,
                                (uint8_t)INPUT_MODE_POS_FILTER);
  ok &= odrv0.setLimits(VEL_LIMIT_TURNS_S, I_SOFT_A);
  ok &= odrv0.setPosGain(POS_GAIN);
  ok &= odrv0.setVelGains(VEL_GAIN, VEL_INT_GAIN);
  return ok;
}

static bool is_closed_loop() {
  return odrv0_user_data.received_heartbeat &&
         odrv0_user_data.last_heartbeat.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL &&
         odrv0_user_data.last_heartbeat.Axis_Error == 0;
}

static bool enter_closed_loop_hold_current(float turns_hold) {
  Serial.println("Entering CLOSED_LOOP_CONTROL (will hold current position) ...");

  // 先把 setpoint 设到当前，避免“刚进闭环就冲”
  float hold_now = odrv0_user_data.last_feedback.Pos_Estimate;  // 当前电机位置
  turns_hold = hold_now;                                         // 可选：同步变量
  odrv0.setPosition(hold_now, 0.0f, 0.0f);
  pump_can_for_ms(50);

  odrv0.clearErrors();
  delay(10);
  odrv0.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);

  for (int k = 0; k < 40; ++k) {
    pump_can_for_ms(25);
    if (is_closed_loop()) {
      Serial.println("CLOSED_LOOP_CONTROL OK");
      return true;
    }
  }

  Serial.print("CLOSED_LOOP_CONTROL NOT CONFIRMED. Axis_State=");
  if (odrv0_user_data.received_heartbeat) Serial.print((int)odrv0_user_data.last_heartbeat.Axis_State);
  else Serial.print("?");
  Serial.print(" Axis_Error=0x");
  if (odrv0_user_data.received_heartbeat) Serial.println((uint32_t)odrv0_user_data.last_heartbeat.Axis_Error, HEX);
  else Serial.println("?");

  return false;
}

static void exit_to_idle() {
  Serial.println("Exiting to IDLE...");
  odrv0.setState(AXIS_STATE_IDLE);
  pump_can_for_ms(200);
}

// ------------------------- Control state -------------------------
static bool  sine_enabled = false;
static float turns_zero   = 0.0f;   // motor turns at joint=0deg
static float turns_hold   = 0.0f;   // motor turns setpoint when not sine
static float sine_start_t = 0.0f;
static float amp_scale    = 0.0f;
static float angle_hold = 0.0f;
static bool have_joint_target = false;   // NEW: only run joint control after we receive 'p ...' (or sine)
static bool log_enabled = false;
static uint32_t log_t0_ms = 0;
static float joint_zero_deg = 0.0f;   // AS5047 reading at "joint = 0deg" when you press 'z'
static bool zero_valid = false;  // set true after 'z'
// ---------- Persistent zero (EEPROM) ----------
struct ZeroData {
  float turns_zero;
  float joint_zero_deg;
  uint32_t magic;
};

static constexpr uint32_t ZERO_MAGIC = 0xA5A55A5A;
static constexpr int EEPROM_ADDR_ZERO = 0;   // start address
static void save_zero_to_eeprom() {
  ZeroData data;
  data.turns_zero = turns_zero;
  data.joint_zero_deg = joint_zero_deg;
  data.magic = ZERO_MAGIC;

  EEPROM.put(EEPROM_ADDR_ZERO, data);   // Teensy: no commit() needed
  Serial.println("Zero saved to EEPROM.");
}

static bool load_zero_from_eeprom() {
  ZeroData data;
  EEPROM.get(EEPROM_ADDR_ZERO, data);

  if (data.magic != ZERO_MAGIC) {
    Serial.println("No valid zero in EEPROM. Use 'z' once.");
    return false;
  }

  turns_zero = data.turns_zero;
  joint_zero_deg = data.joint_zero_deg;
  zero_valid = true;
  Serial.println("Zero loaded from EEPROM.");
  return true;
}

static inline float wrap_deg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

static elapsedMicros loop_timer_us;
static float now_s() { return 1e-6f * (float)micros(); }

static void set_zero_from_feedback() {
  if (odrv0_user_data.received_feedback) {
    turns_zero = odrv0_user_data.last_feedback.Pos_Estimate;
    turns_hold = turns_zero;

    // NEW: also zero the joint encoder reference
    joint_zero_deg = readAngleSensor();
    angle_hold = 0.0f;  // keep commanded target consistent with zero
    zero_valid = true;
    save_zero_to_eeprom();
    Serial.print("Software zero set. turns_zero=");
    Serial.print(turns_zero, 6);
    Serial.print(" joint_zero_deg=");
    Serial.println(joint_zero_deg, 3);
    odrv0.setPosition(turns_zero, 0.0f, 0.0f);
  } else {
    Serial.println("No feedback yet; cannot set zero. (Check GUI: Send feedback every 10ms)");
  }
}

static void hold_current_from_feedback() {
  if (odrv0_user_data.received_feedback) {
    turns_hold = odrv0_user_data.last_feedback.Pos_Estimate;
  }
  odrv0.setPosition(turns_hold, 0.0f, 0.0f);
}

float readAngleSensor() {
  float angle = as5047.readAngle();
  if (angle > 180.0) {
    angle-=360.0;
  }
  return angle;
}

static void set_target_joint_deg(float joint_deg) {
  float current_joint_deg = wrap_deg(readAngleSensor() - joint_zero_deg);
  float target_joint_deg  = joint_deg;  // command is relative to the zero you set with 'z'
  target_joint_deg = constrain(target_joint_deg, JOINT_MIN_DEG, JOINT_MAX_DEG);
  float error             = wrap_deg(target_joint_deg - current_joint_deg);

  float error_d = (error - prev_error);
  prev_error = error;   // NEW: update prev_error so D works
  // float inertia = get_inertia(current_joint_deg);

  error_i+=error;
  if (error_i > 1000) {
    error_i = 1000;
  }
  if (error_i < -1000)  {
    error_i = -1000;
  }
  // Clamp target within safe ROM
  target_joint_deg = constrain(target_joint_deg, JOINT_MIN_DEG, JOINT_MAX_DEG);

  // Outer-loop correction (keep it small!)
  float diff = ENC_KP*error + ENC_KD*error_d + ENC_KI*error_i;
  diff = constrain(diff, -0.05f, 0.05f);   // <-- IMPORTANT safety limit (turns)

  // Absolute mapping: joint target -> motor target (prevents runaway)
  float target_motor_pos = turns_zero + joint_deg_to_motor_turns(target_joint_deg);
  target_motor_pos += diff;

  turns_hold = target_motor_pos;
  odrv0.setPosition(target_motor_pos, 0.0f, 0.0f);

  // Serial.print("current_joint_deg=");
  // Serial.print(current_joint_deg, 3);
  // Serial.print(" target_joint_deg=");
  // Serial.print(joint_deg, 3);
  // Serial.print(" current_motor_turns=");
  // Serial.println(current_motor_pos, 6);
  // Serial.print(" target_motor_turns=");
  // Serial.println(target_motor_pos, 6);
  // Serial.print("Inertia: ");
  // Serial.println(inertia);
}

float get_inertia(float angle_deg) {
  float angle = angle_deg*PI/180.0;
  float vel = (angle-prev_pos)*1000/dt;
  float acc = (vel-prev_vel)*1000/dt;
  float inertia = ((m*g/1000)*(r*sin(angle)/1000)/acc)*1000*(100*100);
  prev_pos = angle;
  prev_vel = vel;
  return inertia;
}

static void start_sine() {
  sine_enabled = true;
  sine_start_t = now_s();
  amp_scale = 0.0f;

  // 先 hold 在零点，避免跳
  turns_hold = turns_zero;
  odrv0.setPosition(turns_hold, 0.0f, 0.0f);

  Serial.print("Sine start: f=");
  Serial.print(SINE_FREQ_HZ, 2);
  Serial.print("Hz, amp=");
  Serial.print(SINE_AMP_DEG, 2);
  Serial.println("deg (ramping)");
}

static void stop_sine_hold() {
  sine_enabled = false;
  amp_scale = 0.0f;

  // 停止时 hold 当前
  hold_current_from_feedback();
  Serial.print("Sine stop, hold turns=");
  Serial.println(turns_hold, 6);
}

// ------------------------- Serial parsing -------------------------
static bool read_line(String& out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      out = buf;
      buf = "";
      return true;
    }
    buf += c;
  }
  return false;
}

static void handle_command(const String& line_raw) {
  String line = line_raw;
  line.trim();
  if (line.length() == 0) return;

  if (line == "h") { print_help(); return; }

  if (line == "g") {
    Serial.print("Applied config: pos_gain="); Serial.print(POS_GAIN, 4);
    Serial.print(" vel_gain="); Serial.print(VEL_GAIN, 4);
    Serial.print(" vel_int_gain="); Serial.print(VEL_INT_GAIN, 4);
    Serial.print(" vel_limit(turn/s)="); Serial.print(VEL_LIMIT_TURNS_S, 2);
    Serial.print(" I_soft(A)="); Serial.println(I_SOFT_A, 2);
    Serial.print("MOTOR_TURNS_PER_JOINT_TURN="); Serial.println(MOTOR_TURNS_PER_JOINT_TURN, 4);
    return;
  }

  if (line == "z") { set_zero_from_feedback(); return; }
  if (line == "a") {
  float cur_joint = wrap_deg(readAngleSensor() - joint_zero_deg);
  Serial.print("Current joint angle (deg): ");
  Serial.println(cur_joint, 3);
  return;
  }
  if (line == "log1") {
  log_enabled = true;
  log_t0_ms = millis();
  Serial.println("t,cmd_deg,act_deg,err_deg,mark");    // CSV header
  return;
  }
  if (line == "log0") {
    log_enabled = false;
    Serial.println("LOG OFF");
    return;
  }

  if (line == "c") {
    // ENTER control: apply config then enter closed loop and hold current
    bool ok = apply_controller_config();
    Serial.print("cfg_done (ok="); Serial.print(ok ? "true" : "false"); Serial.println(")");

    // hold current before entering closed loop
    have_joint_target = false;  
    hold_current_from_feedback();
    enter_closed_loop_hold_current(turns_hold);
    return;
  }
  if (line == "mk") {
    log_mark = 1;
    Serial.println("MARK");
    return;
  }
  if (line == "i") {
    sine_enabled = false;
    exit_to_idle();
    return;
  }

  if (line == "s") { start_sine(); return; }
  if (line == "x") { stop_sine_hold(); return; }

  // p <deg>  (JOINT degrees)
  if (line.startsWith("p")) {
    float deg = 0.0f;
    if (sscanf(line.c_str(), "p %f", &deg) == 1) {
      if (!is_closed_loop()) {
        Serial.println("Not in CLOSED_LOOP_CONTROL. Type 'c' to enter control first.");
        return;
      }
      if (!zero_valid) {
         Serial.println("Zero not set. Type 'z' once to define joint 0 deg.");
         return;
       }
      deg = constrain(deg, JOINT_MIN_DEG, JOINT_MAX_DEG);   // clamp to ROM
      angle_hold = deg;
      have_joint_target = true;                              // NEW: now we have an explicit joint target
      set_target_joint_deg(angle_hold);
    } else {
      Serial.println("Usage: p <deg>   e.g. p 30");
    }
    return;
  }

  // L <turn/s>
  if (line.startsWith("L")) {
    float v = 0.0f;
    if (sscanf(line.c_str(), "L %f", &v) == 1) {
      VEL_LIMIT_TURNS_S = v;
      bool ok = odrv0.setLimits(VEL_LIMIT_TURNS_S, I_SOFT_A);
      Serial.print("vel_limit set to ");
      Serial.print(VEL_LIMIT_TURNS_S, 2);
      Serial.print(" turns/s  ok=");
      Serial.println(ok ? "true" : "false");
    } else {
      Serial.println("Usage: L <turn/s>   e.g. L 5");
    }
    return;
  }

  Serial.println("Unknown command. Type 'h' for help.");
}

// ------------------------- Arduino setup/loop -------------------------
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);
  prev_pos = as5047.readAngle()*PI/180.0;
  prev_vel = 0.0;

  Serial.println("Starting RDS_JointControl (Teensy CAN -> ODrive)");
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv0.onFeedback(onFeedback, &odrv0_user_data);

  if (!setupCAN()) {
    Serial.println("CAN init failed.");
    while (true) delay(100);
  }

  Serial.println("Waiting for ODrive heartbeat...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(10);
  }
  Serial.println("ODrive heartbeat OK.");

  Serial.println("Waiting for feedback (encoder estimates)...");
  uint32_t t0 = millis();
  while (!odrv0_user_data.received_feedback && (millis() - t0 < 2000)) {
    pumpEvents(can_intf);
    delay(5);
  }
  if (!odrv0_user_data.received_feedback) {
    Serial.println("No feedback yet (check GUI: Send feedback every = 10ms).");
  } else {
    // 启动时先把 hold 设为当前，保证不会跳
    hold_current_from_feedback();
    if (load_zero_from_eeprom()) {
    angle_hold = 0.0f;
    have_joint_target = false;  // avoid chasing on entry
  }
  }

  Serial.println("READY. Type 'c' to enter control, 'h' for help.");
  print_help();

  loop_timer_us = 0;
}

void loop() {
  pumpEvents(can_intf);

  String line;
  if (read_line(line)) handle_command(line);

  // 固定频率更新
  const uint32_t period_us = (uint32_t)(1e6f / LOOP_HZ);
  if (loop_timer_us < period_us) return;
  loop_timer_us = 0;

  // 不在闭环就不发命令
  if (!is_closed_loop()) return;

  if (sine_enabled) {
    const float t = now_s() - sine_start_t;

    // amplitude ramp
    if (amp_scale < 1.0f) {
      amp_scale += (1.0f / (AMP_RAMP_TIME_S * LOOP_HZ));
      if (amp_scale > 1.0f) amp_scale = 1.0f;
    }

    const float phase = 2.0f * (float)M_PI * SINE_FREQ_HZ * t;
    const float target_joint_deg = (SINE_AMP_DEG * amp_scale) * sinf(phase);
    angle_hold = target_joint_deg; 
    set_target_joint_deg(target_joint_deg);
  //   const float target_turns = turns_zero + joint_deg_to_motor_turns(target_joint_deg);

  //   odrv0.setPosition(target_turns, 0.0f, 0.0f);

  //   // 输出用于画图：t, target_joint_deg, actual_joint_deg
  //   if (odrv0_user_data.received_feedback) {
  //     const float actual_turns = odrv0_user_data.last_feedback.Pos_Estimate;
  //     const float actual_joint_deg = motor_turns_to_joint_deg(actual_turns - turns_zero);
  //     Serial.print("log,");
  //     Serial.print(t, 4);
  //     Serial.print(",");
  //     Serial.print(target_joint_deg, 3);
  //     Serial.print(",");
  //     Serial.println(actual_joint_deg, 3);
  //   }
  } else {
    // NEW: If user hasn't sent a target, just hold current motor position
    if (!have_joint_target) {
      hold_current_from_feedback();
    } else {
      set_target_joint_deg(angle_hold);
    }
  }
  // ---- CSV logging at 100 Hz ----
  static uint32_t last_log_ms = 0;
  if (log_enabled && (millis() - last_log_ms >= 10)) {
    last_log_ms = millis();
    float t = 0.001f * (millis() - log_t0_ms);
    float cmd = angle_hold;  // 位置保持目标角
    float act = wrap_deg(readAngleSensor() - joint_zero_deg);  // 实际角
    float err = wrap_deg(act - cmd);  // NEW: 误差（受控挠动）

    int mk = log_mark;       // NEW
    log_mark = 0;            // NEW: consume marker

    Serial.print(t, 3); Serial.print(",");
    Serial.print(cmd, 3); Serial.print(",");
    Serial.print(act, 3); Serial.print(",");
    Serial.print(err, 3); Serial.print(",");
    Serial.println(mk);
  }
}