// ============================================================
//  FULL RUN — ESP32-S3 + BNO085 + VL53L0X + Pixy2 + MDD 3A
//
//  FLOW:
//    1. Init all hardware
//    2. Continuously sample Pixy2 & update zone decision
//    3. Show zone via built-in RGB LED:
//         BOT-RIGHT → GREEN
//         BOT-LEFT  → RED
//         TOP-RIGHT → ORANGE
//         TOP-LEFT  → PURPLE
//         UNKNOWN   → WHITE (slow blink)
//    4. When toggle switch flipped ON → run trajectory + rectangle
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>
#include <Pixy2SPI_SS.h>

// ============================================================
//  ★ TUNE THESE ★
// ============================================================

// --- Trajectory 1 (Bot-Left) ---
const unsigned long T1_PHASE1_MS = 400;
const unsigned long T1_PHASE2_MS = 300;
const unsigned long T1_PHASE3_MS = 200;
const unsigned long T1_PAUSE_MS  = 200;
const int           T1_SPEED     = 180;

// --- Trajectory 2 (Top-Left) ---
const unsigned long T2_PHASE1_MS = 400;
const unsigned long T2_PHASE2_MS = 200;
const int           T2_SPEED     = 180;

// --- Trajectory 3 (Bot-Right) ---
const unsigned long T3_PHASE1_MS = 450;
const unsigned long T3_PHASE2_MS = 300;
const unsigned long T3_PHASE3_MS = 200;
const unsigned long T3_PAUSE_MS  = 200;
const int           T3_SPEED     = 180;

// --- Trajectory 4 (Top-Right) ---
const unsigned long T4_PHASE1_MS = 400;
const unsigned long T4_PHASE2_MS = 200;
const int           T4_SPEED     = 180;

// --- Rectangle ---
const float STOP_DISTANCE_CM = 12.0;
int         breadth_pause    = 500;
int         pause_ms         = 50;

// --- Pixy2 zone detection ---
#define SIG_PURPLE        2
#define MIN_AREA          200
#define ROI_TOP_Y         55
#define SPLIT_X           210
#define SPLIT_Y           81
#define DEAD_X            5
#define DEAD_Y            5
#define PIXY_SAMPLE_MS    1500   // one full re-sample window

// ============================================================
//  HARDWARE PINS
// ============================================================

const int M1A = 42;
const int M1B = 41;
const int M2A = 15;
const int M2B = 16;

#define BNO08X_SDA 18
#define BNO08X_SCL 17
#define BNO08X_RST 12

#define TOF_SDA 46
#define TOF_SCL 9

const int INTAKE_IN1   = 38;
const int INTAKE_IN2   = 39;
const int INTAKE_SPEED = 255;

#define SERVO_PIN 40
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

const int SERVO_NEUTRAL = 0;
const int SERVO_SHOOT   = 50;
const int SERVO_LOAD    = 140;

// --- Toggle switch ---
#define TOGGLE_PIN 20   // GPIO 5, pull-down to GND, other side to 3.3V

// --- Built-in RGB LED (ESP32-S3 DevKit) ---
// ESP32-S3-DevKitC uses GPIO 38 for the WS2812 onboard LED
// If yours is different, change this pin
#define RGB_LED_PIN 38
#define RGB_LED_COUNT 1

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel rgbLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
//  PID CONSTANTS
// ============================================================

const float FWD_KP          = 2.6;
const float FWD_KI          = 0.0;
const float FWD_KD          = 0.5;
const int   FWD_BASE_SPEED  = 220;
const int   FWD_MAX_CORRECT = 60;
const float FWD_DEADBAND    = 1.5;
const int   FWD_INTERVAL    = 20;

const float TRN_KP           = 5.0;
const float TRN_KI           = 0.0;
const float TRN_KD           = 0.8;
const int   TRN_MIN_SPEED    = 60;
const int   TRN_MAX_SPEED    = 255;
const float TRN_DEADBAND     = 8.0;
const int   TRN_INTERVAL     = 20;
const int   TRN_STABLE_COUNT = 8;

// ============================================================
//  GLOBALS
// ============================================================

Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;
float             yawOffset = 0.0;

Adafruit_VL53L0X  lox;
Pixy2SPI_SS       pixy;

enum Zone { UNKNOWN, TOP_LEFT, BOT_LEFT, TOP_RIGHT, BOT_RIGHT };

Zone  lastDecision  = UNKNOWN;      // most recent zone decision
bool  decisionReady = false;        // true once we have at least one clean sample

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================

Zone  detectPurpleBall();
Zone  classifyZone(int cx, int cy);

void  runTrajectory1();
void  runTrajectory2();
void  runTrajectory3();
void  runTrajectory4();

void  runRectangleLap();
void  runRectangleLapFromSide3();

void  executeDrive(unsigned long durationMs, float targetHeading);
void  executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway = false);
void  executeTurn(float targetAngle);

void  runForwardPID(float targetHeading, float &prevErr, float &integ);
void  runTurnPID(float targetAngle, float &prevErr, float &integ);

void  driveMotors(int leftSpeed, int rightSpeed);
void  turnClockwise(int speed);
void  turnAntiClockwise(int speed);
void  motorsStop();
void  setMotorPins();
void  waitMs(int ms);

void  initServo();
void  servoWrite(int angle);
void  initIntake();
void  intakeOn();
void  intakeOff();

void  initIMU();
void  zeroIMU();
float getRawYaw();
float getHeading();
float shortestError(float target, float current);
float wrapAngle(float a);

void  initTOF();
float getDistanceCm();

void  setLedForZone(Zone z);
void  setLed(uint8_t r, uint8_t g, uint8_t b);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Toggle switch pin
  pinMode(TOGGLE_PIN, INPUT);   // external pull-down resistor on this pin

  // RGB LED
  rgbLed.begin();
  rgbLed.setBrightness(80);
  setLed(255, 255, 255);        // white = booting

  setMotorPins();
  motorsStop();
  initIntake();
  initServo();
  initIMU();
  zeroIMU();
  initTOF();

  pixy.init();
  pixy.changeProg("color_connected_components");
  Serial.println("Pixy2 Ready.");

  Serial.println("All hardware ready.");
  Serial.println("Sampling Pixy2 continuously — flip toggle to run trajectory.");

  delay(500);
  zeroIMU();

  // Initial blink: white rapid = ready for detection
  for (int i = 0; i < 6; i++) {
    setLed(255, 255, 255);
    delay(150);
    setLed(0, 0, 0);
    delay(150);
  }
}

// ============================================================
//  LOOP
//  Phase A: Continuously detect zone, update LED
//  Phase B: Once toggle is ON, run trajectory and lock into rect
// ============================================================
void loop() {

  // ── PHASE A: Detection loop ───────────────────────────────
  // Keep resampling pixy and showing LED until toggle is flipped
  while (digitalRead(TOGGLE_PIN) == LOW) {

    Serial.println("[STANDBY] Sampling Pixy2...");
    Zone z = detectPurpleBall();

    if (z != UNKNOWN) {
      lastDecision  = z;
      decisionReady = true;
    }

    // Update LED to reflect current best decision
    setLedForZone(lastDecision);

    // Print status
    Serial.print("[STANDBY] Current decision: ");
    switch (lastDecision) {
      case BOT_RIGHT: Serial.println("BOT-RIGHT (GREEN)");   break;
      case BOT_LEFT:  Serial.println("BOT-LEFT  (RED)");     break;
      case TOP_RIGHT: Serial.println("TOP-RIGHT (ORANGE)");  break;
      case TOP_LEFT:  Serial.println("TOP-LEFT  (PURPLE)");  break;
      default:        Serial.println("UNKNOWN   (WHITE)");   break;
    }

    // Small gap between re-samples so serial isn't flooded
    // (detectPurpleBall already takes ~1500ms, so this is minimal)
    delay(100);
  }

  // ── PHASE B: Toggle flipped ON ────────────────────────────
  Serial.println(">>> TOGGLE ON — launching trajectory!");

  // If we never got a clean read, do one final sample now
  if (!decisionReady) {
    Serial.println("No prior decision — running emergency sample...");
    lastDecision = detectPurpleBall();
  }

  // Freeze LED on the decision colour (visual confirmation)
  setLedForZone(lastDecision);
  delay(500);   // half-second hold so you can see the colour before motors spin

  // Zero IMU right before run for fresh heading reference
  zeroIMU();

  // ── Run correct trajectory + rectangle ───────────────────
  if (lastDecision == BOT_LEFT) {
    Serial.println("DECISION: BOT-LEFT → Trajectory 1 → Rect from Side 1");
    runTrajectory1();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 1) ===");
    while (true) { runRectangleLap(); }

  } else if (lastDecision == TOP_LEFT) {
    Serial.println("DECISION: TOP-LEFT → Trajectory 2 → Rect from Side 1");
    runTrajectory2();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 1) ===");
    while (true) { runRectangleLap(); }

  } else if (lastDecision == BOT_RIGHT) {
    Serial.println("DECISION: BOT-RIGHT → Trajectory 3 → Rect from Side 3");
    runTrajectory3();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 3) ===");
    while (true) { runRectangleLapFromSide3(); }

  } else if (lastDecision == TOP_RIGHT) {
    Serial.println("DECISION: TOP-RIGHT → Trajectory 4 → Rect from Side 3");
    runTrajectory4();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 3) ===");
    while (true) { runRectangleLapFromSide3(); }

  } else {
    Serial.println("DECISION: UNKNOWN → defaulting to Trajectory 1 → Rect from Side 1");
    runTrajectory1();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 1) ===");
    while (true) { runRectangleLap(); }
  }
}

// ============================================================
//  LED HELPERS
// ============================================================

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
  rgbLed.show();
}

void setLedForZone(Zone z) {
  switch (z) {
    case BOT_RIGHT: setLed(0,   255, 0);   break;  // GREEN
    case BOT_LEFT:  setLed(255, 0,   0);   break;  // RED
    case TOP_RIGHT: setLed(255, 80,  0);   break;  // ORANGE
    case TOP_LEFT:  setLed(148, 0,   211); break;  // PURPLE
    default:        setLed(255, 255, 255); break;  // WHITE = unknown
  }
}

// ============================================================
//  detectPurpleBall()
// ============================================================
Zone detectPurpleBall() {

  int voteTL    = 0;
  int voteBL    = 0;
  int voteTR    = 0;
  int voteBR    = 0;
  int voteOther = 0;
  int missed    = 0;

  Serial.println("=== PIXY: Sampling purple ball for 1500ms... ===");

  unsigned long start = millis();

  while (millis() - start < PIXY_SAMPLE_MS) {

    pixy.ccc.getBlocks();

    int      bestCx   = 0;
    int      bestCy   = 0;
    uint32_t bestArea = 0;
    bool     found    = false;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      auto &b = pixy.ccc.blocks[i];
      if (b.m_signature != SIG_PURPLE) continue;
      if (b.m_y < ROI_TOP_Y)           continue;

      uint32_t area = (uint32_t)b.m_width * b.m_height;
      if (area < MIN_AREA)              continue;

      if (area > bestArea) {
        bestArea = area;
        bestCx   = b.m_x;
        bestCy   = b.m_y;
        found    = true;
      }
    }

    if (found) {
      Zone z = classifyZone(bestCx, bestCy);
      Serial.print("PIXY | cx="); Serial.print(bestCx);
      Serial.print(" cy="); Serial.print(bestCy);
      Serial.print(" → ");

      switch (z) {
        case TOP_LEFT:  voteTL++;    Serial.println("TOP-LEFT");  break;
        case BOT_LEFT:  voteBL++;    Serial.println("BOT-LEFT");  break;
        case TOP_RIGHT: voteTR++;    Serial.println("TOP-RIGHT"); break;
        case BOT_RIGHT: voteBR++;    Serial.println("BOT-RIGHT"); break;
        default:        voteOther++; Serial.println("OTHER");     break;
      }
    } else {
      missed++;
      Serial.println("PIXY | NOT DETECTED");
    }

    delay(50);
  }

  // Vote summary
  Serial.println("─── PIXY VOTE SUMMARY ───");
  Serial.print("  TOP-LEFT : "); Serial.println(voteTL);
  Serial.print("  BOT-LEFT : "); Serial.println(voteBL);
  Serial.print("  TOP-RIGHT: "); Serial.println(voteTR);
  Serial.print("  BOT-RIGHT: "); Serial.println(voteBR);
  Serial.print("  OTHER    : "); Serial.println(voteOther);
  Serial.print("  MISSED   : "); Serial.println(missed);

  int best = max({voteTL, voteBL, voteTR, voteBR});

  if (best == 0) {
    Serial.println("  RESULT: No valid detections → UNKNOWN");
    return UNKNOWN;
  }

  if      (voteTL == best) { Serial.println("  RESULT: TOP-LEFT");  return TOP_LEFT;  }
  else if (voteBL == best) { Serial.println("  RESULT: BOT-LEFT");  return BOT_LEFT;  }
  else if (voteTR == best) { Serial.println("  RESULT: TOP-RIGHT"); return TOP_RIGHT; }
  else                     { Serial.println("  RESULT: BOT-RIGHT"); return BOT_RIGHT; }
}

// ============================================================
//  classifyZone()
// ============================================================
Zone classifyZone(int cx, int cy) {
  bool isLeft  = cx < (SPLIT_X - DEAD_X);
  bool isRight = cx > (SPLIT_X + DEAD_X);
  bool isTop   = cy < (SPLIT_Y - DEAD_Y);
  bool isBot   = cy > (SPLIT_Y + DEAD_Y);

  if (isLeft  && isTop) return TOP_LEFT;
  if (isLeft  && isBot) return BOT_LEFT;
  if (isRight && isTop) return TOP_RIGHT;
  if (isRight && isBot) return BOT_RIGHT;
  return UNKNOWN;
}

// ============================================================
//  TRAJECTORY 1 — Bot-Left
// ============================================================
void runTrajectory1() {
  Serial.println("=== TRAJECTORY 1 START (Bot-Left: Purple → Orange) ===");
  intakeOn();
  Serial.println("T1 S1: Servo → LOAD");
  servoWrite(SERVO_LOAD);
  delay(500);
  Serial.println("T1 S2: Forward phase 1");
  executeDrive(T1_PHASE1_MS, 0.0);
  Serial.println("T1 S3: Servo → SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(500);
  Serial.println("T1 S4: Forward phase 2 (intake ON)");
  executeDrive(T1_PHASE2_MS, 0.0);
  Serial.println("T1 S5: Pause");
  motorsStop();
  delay(T1_PAUSE_MS);
  Serial.println("T1 S6: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);
  Serial.println("T1 S7: Forward phase 3");
  executeDrive(T1_PHASE3_MS, 0.0);
  Serial.println("T1 S8: Stop");
  motorsStop();
  intakeOff();
  delay(200);
  Serial.println("T1 S9: Final SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(250);
  Serial.println("=== TRAJECTORY 1 DONE ===");
  waitMs(pause_ms);
}

// ============================================================
//  TRAJECTORY 2 — Top-Left
// ============================================================
void runTrajectory2() {
  Serial.println("=== TRAJECTORY 2 START (Top-Left: Orange → Purple) ===");
  intakeOn();
  Serial.println("T2 S1: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);
  Serial.println("T2 S2: Forward phase 1");
  executeDrive(T2_PHASE1_MS, 0.0);
  Serial.println("T2 S3: Stop");
  motorsStop();
  delay(150);
  Serial.println("T2 S4: SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(250);
  Serial.println("T2 S5: Servo → LOAD");
  servoWrite(SERVO_LOAD);
  delay(250);
  Serial.println("T2 S6: Forward phase 2 (intake ON)");
  executeDrive(T2_PHASE2_MS, 0.0);
  Serial.println("T2 S7: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);
  Serial.println("T2 S8: Stop");
  motorsStop();
  Serial.println("=== TRAJECTORY 2 DONE ===");
  waitMs(pause_ms);
}

// ============================================================
//  TRAJECTORY 3 — Bot-Right
// ============================================================
void runTrajectory3() {
  Serial.println("=== TRAJECTORY 3 START (Bot-Right: Purple → Orange) ===");
  Serial.println("T3 S1: Turn → 60°");
  executeTurn(30.0);
  waitMs(pause_ms);
  intakeOn();
  Serial.println("T3 S2: Servo → LOAD");
  servoWrite(SERVO_LOAD);
  delay(500);
  Serial.println("T3 S3: Forward phase 1 @ 60°");
  executeDrive(T3_PHASE1_MS, 30.0);
  Serial.println("T3 S4: Servo → SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(500);
  Serial.println("T3 S5: Forward phase 2 @ 60°");
  executeDrive(T3_PHASE2_MS, 30.0);
  Serial.println("T3 S6: Pause");
  motorsStop();
  delay(T3_PAUSE_MS);
  Serial.println("T3 S7: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);
  Serial.println("T3 S8: Forward phase 3 @ 60°");
  executeDrive(T3_PHASE3_MS, 30.0);
  Serial.println("T3 S9: Stop");
  motorsStop();
  intakeOff();
  delay(200);
  Serial.println("T3 S10: Final SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(250);
  Serial.println("T3 S11: Realign → 180°");
  executeTurn(180.0);
  waitMs(pause_ms);
  Serial.println("=== TRAJECTORY 3 DONE ===");
  waitMs(pause_ms);
}

// ============================================================
//  TRAJECTORY 4 — Top-Right
// ============================================================
void runTrajectory4() {
  Serial.println("=== TRAJECTORY 4 START (Top-Right: Orange → Purple) ===");
  Serial.println("T4 S1: Turn → 60°");
  executeTurn(30.0);
  waitMs(pause_ms);
  intakeOn();
  Serial.println("T4 S2: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);
  Serial.println("T4 S3: Forward phase 1 @ 60°");
  executeDrive(T4_PHASE1_MS, 30.0);
  Serial.println("T4 S4: Stop");
  motorsStop();
  delay(150);
  Serial.println("T4 S5: SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(250);
  Serial.println("T4 S6: Servo → LOAD");
  servoWrite(SERVO_LOAD);
  delay(250);
  Serial.println("T4 S7: Forward phase 2 @ 60°");
  executeDrive(T4_PHASE2_MS, 30.0);
  Serial.println("T4 S8: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);
  Serial.println("T4 S9: Stop");
  motorsStop();
  Serial.println("T4 S10: Realign → 180°");
  executeTurn(180.0);
  waitMs(pause_ms);
  Serial.println("=== TRAJECTORY 4 DONE ===");
  waitMs(pause_ms);
}

// ============================================================
//  runRectangleLap() — from Side 1
// ============================================================
void runRectangleLap() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  Serial.println("=== RECT: Side 1 @ 0° (TOF + shoot midway) ===");
  executeDriveUntilClose(0.0, STOP_DISTANCE_CM, true);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 1 → 45° ===");
  executeTurn(45.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Side 2 @ 90° (time) ===");
  executeDrive(breadth_pause, 90.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 2 → 135° ===");
  executeTurn(135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Side 3 @ 180° (TOF) ===");
  executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 3 → -135° ===");
  executeTurn(-135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Side 4 @ -90° (time) ===");
  executeDrive(breadth_pause, -90.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 4 → -45° ===");
  executeTurn(-45.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Lap complete ===");
}

// ============================================================
//  runRectangleLapFromSide3() — from Side 3
// ============================================================
void runRectangleLapFromSide3() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  Serial.println("=== RECT(S3): Side 3 @ 180° (TOF) ===");
  executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 3 → -135° ===");
  executeTurn(-135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Side 4 @ -90° (time) ===");
  executeDrive(breadth_pause, -90.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 4 → -45° ===");
  executeTurn(-45.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Side 1 @ 0° (TOF + shoot midway) ===");
  executeDriveUntilClose(0.0, STOP_DISTANCE_CM, true);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 1 → 45° ===");
  executeTurn(45.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Side 2 @ 90° (time) ===");
  executeDrive(breadth_pause, 90.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 2 → 135° ===");
  executeTurn(135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Lap complete ===");
}

// ============================================================
//  executeDrive()
// ============================================================
void executeDrive(unsigned long durationMs, float targetHeading) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  while (millis() - startTime < durationMs) {
    unsigned long now = millis();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPID(targetHeading, prevErr, integ);
    }
  }
  motorsStop();
}

// ============================================================
//  executeDriveUntilClose()
// ============================================================
void executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 2000;

  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  while (true) {
    unsigned long now = millis();

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("TOF: MAX TIMEOUT — stopping.");
      break;
    }

    if (shootMidway && !shootDone) {
      if (!shootTriggered && (now - startTime >= 600)) {
        Serial.println("Mid-drive SHOOT triggered");
        servoWrite(SERVO_SHOOT);
        shootTriggered = true;
        shootStartTime = now;
        shootPhase = 1;
      }
      if (shootPhase == 1 && (now - shootStartTime >= 250)) {
        servoWrite(SERVO_NEUTRAL);
        shootPhase = 2;
      }
      if (shootPhase == 2 && (now - shootStartTime >= 500)) {
        shootDone = true;
        Serial.println("Mid-drive SHOOT complete");
      }
    }

    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;

      float dist = getDistanceCm();
      Serial.print("TOF | Dist:");
      if (dist >= 0) { Serial.print(dist, 1); Serial.print("cm"); }
      else             Serial.print("OOR");

      if (dist > 0 && dist < stopDistCm) {
        Serial.println(" → STOP");
        motorsStop();
        break;
      }

      Serial.println();
      runForwardPID(targetHeading, prevErr, integ);
    }
  }

  motorsStop();
}

// ============================================================
//  executeTurn()
// ============================================================
void executeTurn(float targetAngle) {
  float prevErr     = 0.0;
  float integ       = 0.0;
  int   stableCount = 0;
  unsigned long lastPIDTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - lastPIDTime >= TRN_INTERVAL) {
      lastPIDTime = now;

      float heading = getHeading();
      float error   = shortestError(targetAngle, heading);

      if (abs(error) <= TRN_DEADBAND) {
        motorsStop();
        integ   = 0.0;
        prevErr = error;
        stableCount++;
        Serial.print("Stable "); Serial.print(stableCount);
        Serial.print("/"); Serial.print(TRN_STABLE_COUNT);
        Serial.print(" | H:"); Serial.print(heading, 1);
        Serial.print(" E:"); Serial.println(error, 1);
        if (stableCount >= TRN_STABLE_COUNT) {
          Serial.print("Turn done. H:"); Serial.print(heading, 1); Serial.println("deg");
          return;
        }
      } else {
        stableCount = 0;
        runTurnPID(targetAngle, prevErr, integ);
      }
    }
  }
}

// ============================================================
//  runForwardPID()
// ============================================================
void runForwardPID(float targetHeading, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetHeading, heading);
  float dt      = FWD_INTERVAL / 1000.0;

  if (abs(error) <= FWD_DEADBAND) {
    integ   = 0.0;
    prevErr = error;
    driveMotors(FWD_BASE_SPEED, FWD_BASE_SPEED);
    Serial.print("FWD STRAIGHT | H:"); Serial.println(heading, 1);
    return;
  }

  integ += error * dt;
  integ  = constrain(integ, -50, 50);

  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float correction = (FWD_KP * error) + (FWD_KI * integ) + (FWD_KD * derivative);
  correction = constrain(correction, -FWD_MAX_CORRECT, FWD_MAX_CORRECT);

  int leftSpeed  = constrain(FWD_BASE_SPEED + (int)correction, 0, 255);
  int rightSpeed = constrain(FWD_BASE_SPEED - (int)correction, 0, 255);

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("FWD | H:"); Serial.print(heading, 1);
  Serial.print(" E:"); Serial.print(error, 1);
  Serial.print(" Corr:"); Serial.print(correction, 1);
  Serial.print(" L:"); Serial.print(leftSpeed);
  Serial.print(" R:"); Serial.println(rightSpeed);
}

// ============================================================
//  runTurnPID()
// ============================================================
void runTurnPID(float targetAngle, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetAngle, heading);
  float dt      = TRN_INTERVAL / 1000.0;

  integ += error * dt;
  integ  = constrain(integ, -100, 100);

  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float output = (TRN_KP * error) + (TRN_KI * integ) + (TRN_KD * derivative);

  int speed = (int)abs(output);
  speed = constrain(speed, TRN_MIN_SPEED, TRN_MAX_SPEED);

  if (error > 0) turnClockwise(speed);
  else           turnAntiClockwise(speed);

  Serial.print("TRN | H:"); Serial.print(heading, 1);
  Serial.print(" Target:"); Serial.print(targetAngle, 1);
  Serial.print(" E:"); Serial.print(error, 1);
  Serial.print(" Spd:"); Serial.println(speed);
}

// ============================================================
//  TOF
// ============================================================
void initTOF() {
  Wire1.begin(TOF_SDA, TOF_SCL);
  if (!lox.begin(0x29, false, &Wire1)) {
    Serial.println("VL53L0X not found! Check wiring.");
    while (1) delay(10);
  }
  Serial.println("VL53L0X Ready.");
}

float getDistanceCm() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter / 10.0;
  }
  return -1.0;
}

// ============================================================
//  SERVO
// ============================================================
void initServo() {
  ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_RES);
  servoWrite(SERVO_NEUTRAL);
  delay(500);
  Serial.println("Servo ready.");
}

void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t duty = map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
  ledcWrite(SERVO_PIN, duty);
}

// ============================================================
//  INTAKE
// ============================================================
void initIntake() {
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, 0);
}

void intakeOn() {
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, INTAKE_SPEED);
}

void intakeOff() {
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, 0);
}

// ============================================================
//  IMU
// ============================================================
void initIMU() {
  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not found!");
    while (1) delay(10);
  }
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  delay(200);
  Serial.println("BNO085 Ready.");
}

void zeroIMU() {
  for (int i = 0; i < 20; i++) {
    bno.getSensorEvent(&imuData);
    delay(10);
  }
  yawOffset = getRawYaw();
  Serial.print("IMU Zeroed at: "); Serial.print(yawOffset, 2); Serial.println("deg");
}

float getRawYaw() {
  if (bno.wasReset()) bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  while (true) {
    if (bno.getSensorEvent(&imuData)) {
      if (imuData.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float qw = imuData.un.gameRotationVector.real;
        float qx = imuData.un.gameRotationVector.i;
        float qy = imuData.un.gameRotationVector.j;
        float qz = imuData.un.gameRotationVector.k;
        return -degrees(atan2(2.0f*(qw*qz + qx*qy),
                              1.0f - 2.0f*(qy*qy + qz*qz)));
      }
    }
  }
}

float getHeading() {
  return wrapAngle(getRawYaw() - yawOffset);
}

float shortestError(float targetDeg, float currentDeg) {
  float error = wrapAngle(targetDeg) - wrapAngle(currentDeg);
  return wrapAngle(error);
}

float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ============================================================
//  MOTORS
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(M1A, 0);          analogWrite(M1B, leftSpeed);
  } else if (leftSpeed < 0) {
    analogWrite(M1A, -leftSpeed); analogWrite(M1B, 0);
  } else {
    analogWrite(M1A, 0);          analogWrite(M1B, 0);
  }
  if (rightSpeed > 0) {
    analogWrite(M2A, rightSpeed); analogWrite(M2B, 0);
  } else if (rightSpeed < 0) {
    analogWrite(M2A, 0);          analogWrite(M2B, -rightSpeed);
  } else {
    analogWrite(M2A, 0);          analogWrite(M2B, 0);
  }
}

void turnClockwise(int speed) {
  analogWrite(M1A, 0);     analogWrite(M1B, speed);
  analogWrite(M2A, 0);     analogWrite(M2B, speed);
}

void turnAntiClockwise(int speed) {
  analogWrite(M1A, speed); analogWrite(M1B, 0);
  analogWrite(M2A, speed); analogWrite(M2B, 0);
}

void motorsStop() {
  analogWrite(M1A, 0); analogWrite(M1B, 0);
  analogWrite(M2A, 0); analogWrite(M2B, 0);
}

void setMotorPins() {
  pinMode(M1A, OUTPUT); pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT); pinMode(M2B, OUTPUT);
}

void waitMs(int ms) {
  motorsStop();
  delay(ms);
}