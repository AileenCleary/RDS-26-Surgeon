#include <SPI.h>
#include <math.h>

// ===================== Pin settings =====================
// Teensy 4.1 default SPI pins:
// MOSI = 11, MISO = 12, SCK = 13

const int NUM_ENC = 4;

const int CS_PINS[NUM_ENC] = {
  10,  // JENC1_CS = SPLAY
  25,  // JENC2_CS = MCP
  36,  // JENC3_CS = PIP
  37   // JENC4_CS = DIP
};

const int FORCE_PIN = A0;

const char* ENC_NAMES[NUM_ENC] = {
  "SPLAY",
  "MCP",
  "PIP",
  "DIP"
};

const uint32_t SPI_HZ = 1000000;
const uint32_t PERIOD_MS = 200;

const uint8_t SPI_MODE_USED = SPI_MODE0;

// ===================== MA782 BCT settings =====================

const uint8_t REG_BCT = 0x02;       // BCT[7:0]
const uint8_t REG_TRIM_DIR = 0x03;  // bit0 = ETX, bit1 = ETY

// Test [0, 86, 129, 155, 172, 184, 194, 201, 207]
const uint8_t SPLAY_BCT_VALUE = 0;
const uint8_t MCP_BCT_VALUE = 0;
const uint8_t PIP_BCT_VALUE = 0;
const uint8_t DIP_BCT_VALUE = 0;
const uint8_t BCT_VALUES[NUM_ENC] = {SPLAY_BCT_VALUE, MCP_BCT_VALUE, PIP_BCT_VALUE, DIP_BCT_VALUE};

// Try X first. If the linearity gets worse, switch to TRIM_X=false, TRIM_Y=true.
const bool TRIM_X = true;
const bool TRIM_Y = false;

const float RM_OHMS = 10000.0;     // Your Rm value (e.g., 100k Ohms)
const float FORCE_SLOPE = 0.098;    // Calculated m
const float FORCE_INTERCEPT = -0.286;  // Calculated b

const int ADC_MAX = 1023;

// ===================== Calibration variables =====================

bool haveZero = true;
bool haveCal[NUM_ENC] = {false, false, false, false};

uint16_t w_zero[NUM_ENC] = {40052, 49917, 16690, 10495};
uint16_t w_cal[NUM_ENC]  = {0, 0, 0, 0};

// SPLAY: +10 deg
// MCP/PIP/DIP: +90 deg
const float CAL_TARGET_DEG[NUM_ENC] = {
  10.0f,  // SPLAY
  90.0f,  // MCP
  90.0f,  // PIP
  90.0f   // DIP
};

// ===================== WMA Filter Variables =====================
const int WMA_WINDOW = 20;

float WMA_WEIGHTS[WMA_WINDOW];
float WMA_WEIGHT_SUM = 0.0;

float angleHistory[NUM_ENC][WMA_WINDOW] = {0};
bool firstRead[NUM_ENC] = {true, true, true, true};

// ===================== SPI functions =====================

uint16_t spiTransfer16(int csPin, uint16_t data) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE_USED));

  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  uint16_t w = SPI.transfer16(data);

  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);

  SPI.endTransaction();

  return w;
}

uint16_t spiRead16(int csPin) {
  return spiTransfer16(csPin, 0x0000);
}

void readAllEncoders(uint16_t w[NUM_ENC]) {
  for (int i = 0; i < NUM_ENC; i++) {
    w[i] = spiRead16(CS_PINS[i]);
  }
}

// MA782 write register:
// first 16-bit frame: [100][5-bit register address][8-bit value]
// second 16-bit frame: 0x0000, returns register value in low 8 bits
uint8_t ma782WriteRegister(int csPin, uint8_t regAddr, uint8_t value) {
  uint16_t cmd = (0b100 << 13) | ((regAddr & 0x1F) << 8) | value;

  spiTransfer16(csPin, cmd);
  delayMicroseconds(2);

  uint16_t resp = spiTransfer16(csPin, 0x0000);
  return resp & 0xFF;
}

// MA782 read register:
// first 16-bit frame: [010][5-bit register address][00000000]
// second 16-bit frame: 0x0000, returns register value in low 8 bits
uint8_t ma782ReadRegister(int csPin, uint8_t regAddr) {
  uint16_t cmd = (0b010 << 13) | ((regAddr & 0x1F) << 8);

  spiTransfer16(csPin, cmd);
  delayMicroseconds(2);

  uint16_t resp = spiTransfer16(csPin, 0x0000);
  return resp & 0xFF;
}

void setBCTForAllEncoders() {
  uint8_t trimReg = 0x00;

  if (TRIM_X) {
    trimReg |= 0x01;  // ETX = bit0
  }

  if (TRIM_Y) {
    trimReg |= 0x02;  // ETY = bit1
  }

  Serial.println("Setting BCT for all MA782 encoders...");
  Serial.print("SPLAY_BCT_VALUE = ");
  Serial.println(SPLAY_BCT_VALUE);
  Serial.print("MCP_BCT_VALUE = ");
  Serial.println(MCP_BCT_VALUE);
  Serial.print("PIP_BCT_VALUE = ");
  Serial.println(PIP_BCT_VALUE);
  Serial.print("DIP_BCT_VALUE = ");
  Serial.println(DIP_BCT_VALUE);
  Serial.print("Trim register = 0x");
  Serial.println(trimReg, HEX);

  for (int i = 0; i < NUM_ENC; i++) {
    uint8_t bctAck = ma782WriteRegister(CS_PINS[i], REG_BCT, BCT_VALUES[i]);
    delayMicroseconds(5);

    uint8_t trimAck = ma782WriteRegister(CS_PINS[i], REG_TRIM_DIR, trimReg);
    delayMicroseconds(5);

    uint8_t bctRead = ma782ReadRegister(CS_PINS[i], REG_BCT);
    delayMicroseconds(5);

    uint8_t trimRead = ma782ReadRegister(CS_PINS[i], REG_TRIM_DIR);
    delayMicroseconds(5);

    Serial.print("  ");
    Serial.print(ENC_NAMES[i]);
    Serial.print(" BCT ack=");
    Serial.print(bctAck);
    Serial.print(" read=");
    Serial.print(bctRead);

    Serial.print(" | trim ack=0x");
    Serial.print(trimAck, HEX);
    Serial.print(" read=0x");
    Serial.println(trimRead, HEX);
  }

  Serial.println();
}

// ===================== Angle conversion =====================

// 16-bit wrap-around difference
int32_t diff16(uint16_t a, uint16_t b) {
  int32_t d = (int32_t)a - (int32_t)b;

  if (d > 32768) {
    d -= 65536;
  }

  if (d < -32768) {
    d += 65536;
  }

  return d;
}

float countsToDeg(int32_t counts) {
  return counts * (360.0f / 65536.0f);
}

float computeJointDeg(int i, uint16_t w_now) {
  float deg_sensor = 0.0f;

  if (haveZero) {
    deg_sensor = countsToDeg(diff16(w_now, w_zero[i]));
  }

  float jointDeg = deg_sensor;

  if (haveZero && haveCal[i]) {
    float deg_cal_sensor = countsToDeg(diff16(w_cal[i], w_zero[i]));

    if (fabs(deg_cal_sensor) > 5.0f) {
      float scale = CAL_TARGET_DEG[i] / deg_cal_sensor;
      jointDeg = scale * deg_sensor;
    } else {
      jointDeg = deg_sensor;

      Serial.print("WARNING: ");
      Serial.print(ENC_NAMES[i]);
      Serial.print(" calibration span too small: ");
      Serial.print(deg_cal_sensor, 3);
      Serial.print(" deg. Using raw sensor angle. ");
    }
  }

  return jointDeg;
}

float computeWMA(int encIdx, float newAngle) {
  // If this is the first time reading, fill the entire history buffer
  // with the first value so the filter doesn't slowly ramp up from 0
  if (firstRead[encIdx]) {
    for (int i = 0; i < WMA_WINDOW; i++) {
      angleHistory[encIdx][i] = newAngle;
    }
    firstRead[encIdx] = false;
  } else {
    // Shift history to the right (oldest data at the end is overwritten)
    for (int i = WMA_WINDOW - 1; i > 0; i--) {
      angleHistory[encIdx][i] = angleHistory[encIdx][i - 1];
    }
    // Insert new reading at the beginning (index 0 is the newest data)
    angleHistory[encIdx][0] = newAngle;
  }

  // Calculate the weighted sum
  float wma = 0;
  for (int i = 0; i < WMA_WINDOW; i++) {
    wma += angleHistory[encIdx][i] * WMA_WEIGHTS[i];
  }

  // Divide by total weight to get the final averaged degree
  return wma / WMA_WEIGHT_SUM;
}

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

// ===================== Setup =====================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  SPI.begin();

  for (int i = 0; i < NUM_ENC; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }

  for (int i = 0; i < WMA_WINDOW; i++) {
    WMA_WEIGHTS[i] = (float)(WMA_WINDOW - i);
    WMA_WEIGHT_SUM += WMA_WEIGHTS[i];
  }

  Serial.println("4-Encoder SPI angle test + BCT");
  Serial.println("JENC1 = SPLAY, range should be about -10 to +10 deg");
  Serial.println("JENC2 = MCP, range 0 to 90 deg");
  Serial.println("JENC3 = PIP, range 0 to 90 deg");
  Serial.println("JENC4 = DIP, range 0 to 90 deg");
  Serial.println();

  // setBCTForAllEncoders();

  Serial.println("Commands:");
  Serial.println("  z : set all current positions as 0 deg");
  Serial.println("  s : set SPLAY current position as +10 deg");
  Serial.println("  m : set MCP current position as +90 deg");
  Serial.println("  p : set PIP current position as +90 deg");
  Serial.println("  d : set DIP current position as +90 deg");
  Serial.println();
}

// ===================== Main loop =====================

void loop() {
  uint16_t w[NUM_ENC];

  // Handle serial commands
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'z') {
      readAllEncoders(w);

      for (int i = 0; i < NUM_ENC; i++) {
        w_zero[i] = w[i];
      }

      haveZero = true;

      Serial.println("Set ZERO for all encoders:");
      for (int i = 0; i < NUM_ENC; i++) {
        Serial.print("  ");
        Serial.print(ENC_NAMES[i]);
        Serial.print(" ZERO W=0x");
        Serial.println(w_zero[i], HEX);
      }
      Serial.println();
    }
    else if (c == 's') {
      uint16_t w_now = spiRead16(CS_PINS[0]);
      w_cal[0] = w_now;
      haveCal[0] = true;

      Serial.print("Set SPLAY calibration at +10 deg, W=0x");
      Serial.println(w_cal[0], HEX);
    }

    else if (c == 'm') {
      uint16_t w_now = spiRead16(CS_PINS[1]);
      w_cal[1] = w_now;
      haveCal[1] = true;

      Serial.print("Set MCP calibration at +90 deg, W=0x");
      Serial.println(w_cal[1], HEX);
    }

    else if (c == 'p') {
      uint16_t w_now = spiRead16(CS_PINS[2]);
      w_cal[2] = w_now;
      haveCal[2] = true;

      Serial.print("Set PIP calibration at +90 deg, W=0x");
      Serial.println(w_cal[2], HEX);
    }

    else if (c == 'd') {
      uint16_t w_now = spiRead16(CS_PINS[3]);
      w_cal[3] = w_now;
      haveCal[3] = true;

      Serial.print("Set DIP calibration at +90 deg, W=0x");
      Serial.println(w_cal[3], HEX);
    }

  }

  // Read all encoders
  readAllEncoders(w);

  for (int i = 0; i < NUM_ENC; i++) {
    float jointDeg = computeJointDeg(i, w[i]);
    float wmaDeg = computeWMA(i, jointDeg);

    Serial.print(ENC_NAMES[i]);
    Serial.print("_Raw:");
    Serial.print(w[i]);
    Serial.print(",");
    
    Serial.print(ENC_NAMES[i]);
    Serial.print("_WMA:");
    Serial.print(wmaDeg, 2);

    Serial.print(",");
  }

  int forceReading = analogRead(FORCE_PIN);
  float forceN = forceAnalogToNewtons(forceReading);
  Serial.print("Force_Raw:");
  Serial.print(forceReading);
  Serial.print(",");
  Serial.print("Force_N:");
  Serial.println(forceN, 2);

  Serial.println();
  delay(PERIOD_MS);
}