// --------------------OLIVIA -------------

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>
#include <Pixy2SPI_SS.h>

#include <FastLED.h>
#define LED_PIN   37
#define NUM_LEDS  1
CRGB leds[NUM_LEDS];
int lap3Counter = 0;   // counts laps; every 3rd lap uses shorter breadth_pause for Side 4
int   lapCounter     = 0;    // counts laps for heading drift correction
float headingOffset  = 0.0;  // added to all forward-drive headings every 5 laps
float driftCorrection = 0.0;  // subtracted from heading every 2 laps
// ============================================================
//  ★ TUNE THESE ★
// ============================================================

// --- Trajectory 1 (Bot-Left: Purple → Orange) ---
const unsigned long T1_PHASE1_MS = 470;
const unsigned long T1_PHASE2_MS = 300;
const unsigned long T1_PHASE3_MS = 200;
const unsigned long T1_PAUSE_MS  = 200;
const int           T1_SPEED     = 180;

// --- Trajectory 2 (Top-Left: Orange → Purple) ---
const unsigned long T2_PHASE1_MS = 400;
const unsigned long T2_PHASE2_MS = 400;
const int           T2_SPEED     = 180;

// --- Trajectory 3 (Bot-Right: 60° turn + T1 logic + realign) ---
const unsigned long T3_PHASE1_MS = 500;
const unsigned long T3_PHASE2_MS = 300;
const unsigned long T3_PHASE3_MS = 200;
const unsigned long T3_PAUSE_MS  = 200;
const int           T3_SPEED     = 180;

// --- Trajectory 4 (Top-Right: 60° turn + T2 logic + realign) ---
const unsigned long T4_PHASE1_MS = 450;
const unsigned long T4_PHASE2_MS = 400;
const int           T4_SPEED     = 180;

// --- Rectangle ---
const float STOP_DISTANCE_CM   = 9.0;   // Side 3 TOF stop distance
int         breadth_pause      = 500;
int         pause_ms           = 50;

// --- Fixed two-minute match strategy (India on-site update) ---
const unsigned long MATCH_DURATION_MS       = 120000UL;
const unsigned long ENDGAME_START_MS        = 110000UL;
const unsigned long FIRST_SINGLE_SHOT_MS    = 114000UL;
const unsigned long SECOND_SINGLE_SHOT_MS   = 115000UL;
const unsigned long THIRD_SINGLE_SHOT_MS    = 116000UL;
const unsigned long FINAL_FLUSH_START_MS    = 117000UL;
const unsigned long ANTIJAM_INTERVAL_MS     = 4900UL;
const unsigned long ANTIJAM_REVERSE_MS      = 200UL;
const unsigned long TURN_TIMEOUT_MS         = 1800UL;
const float         ENDGAME_CORNER_LONG_DEG = 180.0f;
const float         ENDGAME_CORNER_SIDE_DEG = 90.0f;   // Olivia: bottom-right corner
const float         ENDGAME_SHOOT_DEG       = 5.0f;
const float         ENDGAME_SWEEP_DEG       = -30.0f;  // Olivia sweeps inward/left only
const float         SWEEP_FALLBACK_WALL_DEG = 21.0f;   // Olivia's right-wall lane
const float         ENDGAME_WALL_CLEAR_CM   = 18.0f;
const float         ENDGAME_RAMP_CLEAR_CM   = 24.0f;
const int           ENDGAME_HOME_SPEED      = 145;
const int           ENDGAME_TRACK_SPEED     = 135;
const int           ENDGAME_CREEP_SPEED     = 85;
const unsigned long SWEEP_MAX_TOTAL_MS       = 2000UL;
const unsigned long SWEEP_WALL_RESERVE_MS    = 250UL;
const unsigned long SWEEP_REVERSE_MAX_MS     = 600UL;
const int           SWEEP_REVERSE_SPEED      = 255;

// Orange is Pixy signature 1 in the supplied colour programme.
#define SIG_ORANGE          1
#define ORANGE_MIN_AREA     90
#define ORANGE_CAPTURE_Y    145
#define ORANGE_CAPTURE_AREA 1900
#define PIXY_FRAME_CENTRE_X 158
#define ORANGE_X_TOLERANCE  12
#define ENDGAME_PURPLE_MIN_AREA 70

// --- Pixy2 Sig3 stop (Side 1) ---
#define SIG3_STOP_TOP_CY    150      // stop when filtered top_cy reaches this value
#define SIG3_MIN_AREA       500     // ignore blobs smaller than this
#define SIG3_EMA_ALPHA      0.7f    // EMA smoothing (lower = smoother)
#define SIG3_TIMEOUT_MS     2500    // fallback timeout if sig3 never reaches target
#define SIG3_GUARD_MS       800     // minimum drive time before stop is allowed
#define SIG3_FULLSPEED_MS   600     // drive at full speed before slowing for detection
#define SIG3_FWD_SPEED      50     // detection speed for Side 1

// --- Pixy2 zone detection ---
#define SIG_PURPLE        2
#define MIN_AREA          200
#define ROI_TOP_Y         55
// #define SPLIT_X           210
// #define SPLIT_Y           79        // ★ TUNED from actual mat readings
// #define DEAD_X            10        // ★ TUNED
// #define DEAD_Y            2         // ★ TUNED
#define PIXY_SAMPLE_MS    1500


  #define SPLIT_X   208
  #define SPLIT_Y   88
  #define DEAD_X    15
  #define DEAD_Y    4

// --- Referee start toggle switch ---
#define SWITCH_PWR              48
#define SWITCH_SIG              21
#define SWITCH_DEBOUNCE_READS   5
#define SWITCH_POLL_MS          10

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

#define TOF_SDA 2
#define TOF_SCL 1

const int INTAKE_IN1   = 38;
const int INTAKE_IN2   = 39;
const int INTAKE_SPEED = 255;
const int SLOW = 120;

#define SERVO_PIN 40
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

const int SERVO_NEUTRAL = 30;
const int SERVO_SHOOT   = 61;
const int SERVO_LOAD    = 140;

// ============================================================
//  PID CONSTANTS
// ============================================================

const float FWD_KP          = 2.6;
const float FWD_KI          = 0.0;
const float FWD_KD          = 0.5;
const int   FWD_BASE_SPEED  = 220;
const int   FWD_MAX_CORRECT = 120;   // 60 
const float FWD_DEADBAND    = 0.8;
const int   FWD_INTERVAL    = 20;

const float TRN_KP           = 8.0;
const float TRN_KI           = 0.0;
const float TRN_KD           = 0.8;
const int   TRN_MIN_SPEED    = 60;
const int   TRN_MAX_SPEED    = 255;
const float TRN_DEADBAND     = 10.0;
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

Zone decidedZone = UNKNOWN;

unsigned long matchStartMs       = 0;
unsigned long lastAntiJamMs      = 0;
bool          matchClockRunning  = false;
bool          fixedEndgameActive = false;
bool          fixedEndgameDone   = false;
bool          tofReady           = false;
bool          endgamePurpleStored = false;
int           endgameOrangeEstimate = 0;
int           intakeModeState = 0;  // 0=off, 1=forward, 2=reverse

// EMA state for Pixy sig3 top_cy (used in executeDriveUntilPixy)
float ema_top_cy    = -1.0f;
bool  ema_init      = false;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================

Zone  detectPurpleBall();
Zone  classifyZone(int cx, int cy);
Zone  samplePurpleOnce();

void  runTrajectory1();
void  runTrajectory2();
void  runTrajectory3();
void  runTrajectory4();

void  runRectangleLap();
void  runRectangleLapFromTurn1();
void  runRectangleLapFromTurn3();
void  runRectangleLapFromSide3();


// void  runRectangleFromSide1();
// void  runRectangleFromSide3();
// void  runRectangleLap();
// void  runRectangleLapFromSide3();
// void  runRectangleLapFromTurn1();

void  executeDrive(unsigned long durationMs, float targetHeading);
void  executeDriveUntilPixy(float targetHeading, bool shootMidway, bool useFullSpeedPhase);
void  executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart = false);
void  executeTurn(float targetAngle, bool shootDuringTurn = false);
void  runForwardPID(float targetHeading, float &prevErr, float &integ, int baseSpeed);
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
void  intakeReverse();
void  intakeReverseSlowed();

void  initIMU();
void  zeroIMU();
float getRawYaw();
float getHeading();
float shortestError(float target, float current);
float wrapAngle(float a);

void  initTOF();
float getDistanceCm();

void  initStartSwitch();
bool  isStartSwitchOn();
void  waitForStartSwitch();
void  runMission(Zone z);

unsigned long matchElapsedMs();
bool  normalCycleExpired();
bool  matchExpired();
void  serviceAntiJam();
bool  readLargestOrange(int &x, int &y, uint32_t &area);
bool  readEndgameBall(bool &isPurple, int &x, int &y, uint32_t &area);
bool  readSweepTarget(bool &isPathPurple, int &x, int &y, uint32_t &area);
void  driveTimedEndgame(unsigned long durationMs, float targetHeading, int speed, bool reverseDrive = false);
void  driveToWallEndgame(float targetHeading, float stopDistCm, unsigned long timeoutMs, int speed);
bool  collectOneOrangeUntil(unsigned long absoluteMatchDeadlineMs);
bool  collectEndgameBallsUntil(unsigned long absoluteMatchDeadlineMs, bool stopAfterOneOrange);
void  executeEndgameTurn(float targetAngle, unsigned long timeoutMs = 650UL);
void  homeToOliviaCorner();
void  waitUntilMatchTime(unsigned long targetMs);
void  returnToRampBefore(unsigned long deadlineMs);
void  slowReleaseAtRamp(unsigned long durationMs);
void  fireOnePulse(unsigned long pulseMs = 220UL);
unsigned long estimateClosestBallTripMs(bool isPurple, int y, uint32_t area);
void  runMirroredWallFallbackUntil(unsigned long deadlineMs);
void  runSweepForTimedShot(unsigned long shotAtMs);
void  shootAllRemaining();
void  runFixedTwoMinuteFinish();

void setLedForZone(Zone z) {
  switch (z) {
    case BOT_LEFT:  leds[0] = CRGB(255,   0,   0); break;  // Red
    case TOP_LEFT:  leds[0] = CRGB(  0, 255,   0); break;  // Green
    case BOT_RIGHT: leds[0] = CRGB(255, 100,   0); break;  // Orange
    case TOP_RIGHT: leds[0] = CRGB(0, 0, 180); break;  // Dark Blue
    default:        leds[0] = CRGB(255, 255, 255); break;  // White = UNKNOWN
  }
  FastLED.show();
}
// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(80);
  leds[0] = CRGB::Black;
  FastLED.show();

  setMotorPins();
  motorsStop();
  initIntake();
  initServo();
  initIMU();
  zeroIMU();
  initTOF();
  initStartSwitch();

  pixy.init();
  pixy.changeProg("color_connected_components");
  Serial.println("Pixy2 Ready.");

  Serial.println("All hardware ready. Sampling Pixy2...");
  delay(500);
  zeroIMU();

  decidedZone = UNKNOWN;
  Serial.print("=== TRAJECTORY DECISION LOCKED IN: ");
  switch (decidedZone) {
    case TOP_LEFT:  Serial.println("TOP-LEFT ===");  break;
    case BOT_LEFT:  Serial.println("BOT-LEFT ===");  break;
    case TOP_RIGHT: Serial.println("TOP-RIGHT ==="); break;
    case BOT_RIGHT: Serial.println("BOT-RIGHT ==="); break;
    default:        Serial.println("UNKNOWN ===");   break;
  }

  setLedForZone(UNKNOWN);    // ← LED ON after detection, during wait

  waitForStartSwitch();

  leds[0] = CRGB::Black;         // ← LED OFF the moment switch fires
  FastLED.show();

  runMission(decidedZone);
}

void loop() {
  if (fixedEndgameDone) {
    runRectangleLap();
    return;
  }

  motorsStop();
  delay(100);
}

// ============================================================
//  runMission()
// ============================================================
void runMission(Zone z) {
  if (z == BOT_LEFT) {
    // T1 ends at Side1 wall → skip Side1, start from Turn1
    runTrajectory1();
    runRectangleLapFromTurn1();
    while (!normalCycleExpired()) { runRectangleLap(); }

  } else if (z == TOP_LEFT) {
    // T2 also ends at Side1 wall → same pattern as T1
    runTrajectory2();
    runRectangleLapFromTurn1();
    while (!normalCycleExpired()) { runRectangleLap(); }

  } else if (z == BOT_RIGHT) {
    // T3 ends at Side3 wall → skip Side3, start from Turn3
    runTrajectory3();
    runRectangleLapFromTurn1();
    while (!normalCycleExpired()) { runRectangleLap(); }

  } else if (z == TOP_RIGHT) {
    // T4 ends at Side3 wall → same pattern as T3
    runTrajectory4();
    runRectangleLapFromTurn1();
    while (!normalCycleExpired()) { runRectangleLap(); }

  } else {
    // // UNKNOWN → fallback same as BOT_LEFT
    // runTrajectory1();
    // runRectangleLapFromTurn1();
    while (!normalCycleExpired()) { runRectangleLap(); }
  }

  runFixedTwoMinuteFinish();
  fixedEndgameDone = true;
  matchClockRunning = false;
}

// ============================================================
//  START SWITCH
// ============================================================
void initStartSwitch() {
  pinMode(SWITCH_PWR, OUTPUT);
  digitalWrite(SWITCH_PWR, HIGH);
  pinMode(SWITCH_SIG, INPUT_PULLDOWN);
  Serial.println("Start switch ready.");
}

bool isStartSwitchOn() {
  return digitalRead(SWITCH_SIG) == LOW;
}

void waitForStartSwitch() {
  Serial.println("=== Waiting for REFEREE START (live tracking active) ===");
  int stableHighCount = 0;
  int consecutiveMisses = 0;
  unsigned long lastPixySample = 0;
  unsigned long lastPrint      = millis();

  while (true) {
    unsigned long now = millis();

    // ── Live Pixy tracking every 50ms ──────────────────────
    if (now - lastPixySample >= 120) {
      lastPixySample = now;
      Zone detected  = samplePurpleOnce();
      if (detected != UNKNOWN) {
        decidedZone = detected;
        consecutiveMisses = 0;
      } else if (++consecutiveMisses >= 8) {
        decidedZone = UNKNOWN;
      }
      setLedForZone(decidedZone);
    }

    // ── Switch debounce ─────────────────────────────────────
    if (isStartSwitchOn()) {
      stableHighCount++;
      if (stableHighCount >= SWITCH_DEBOUNCE_READS) {
        matchStartMs = millis();
        lastAntiJamMs = matchStartMs;
        matchClockRunning = true;
        Serial.print("=== START — Zone locked: ");
        switch (decidedZone) {
          case TOP_LEFT:  Serial.println("TOP-LEFT ===");  break;
          case BOT_LEFT:  Serial.println("BOT-LEFT ===");  break;
          case TOP_RIGHT: Serial.println("TOP-RIGHT ==="); break;
          case BOT_RIGHT: Serial.println("BOT-RIGHT ==="); break;
          default:        Serial.println("UNKNOWN ===");   break;
        }
        return;
      }
    } else {
      stableHighCount = 0;
    }

    if (now - lastPrint >= 2000) {
      lastPrint = now;
      Serial.println("... waiting for start switch ...");
    }

    delay(SWITCH_POLL_MS);
  }
}

Zone samplePurpleOnce() {
    pixy.ccc.getBlocks();
    uint32_t bestArea = 0;
    int bestCx = 0, bestCy = 0;
    bool found = false;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        auto &b = pixy.ccc.blocks[i];
        if (b.m_signature != SIG_PURPLE) continue;
        if (b.m_y < ROI_TOP_Y) continue;
        uint32_t area = (uint32_t)b.m_width * b.m_height;
        if (area < MIN_AREA) continue;
        if (area > bestArea) { bestArea = area; bestCx = b.m_x; bestCy = b.m_y; found = true; }
    }
    if (found) return classifyZone(bestCx, bestCy);
    return UNKNOWN;
}
// ============================================================
//  detectPurpleBall()
// ============================================================
Zone detectPurpleBall() {
  int voteTL = 0, voteBL = 0, voteTR = 0, voteBR = 0, voteOther = 0, missed = 0;
  Serial.println("=== PIXY: Sampling purple ball for 1500ms... ===");
  unsigned long start = millis();

  while (millis() - start < PIXY_SAMPLE_MS) {
    pixy.ccc.getBlocks();
    int bestCx = 0, bestCy = 0;
    uint32_t bestArea = 0;
    bool found = false;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      auto &b = pixy.ccc.blocks[i];
      if (b.m_signature != SIG_PURPLE) continue;
      if (b.m_y < ROI_TOP_Y)           continue;
      uint32_t area = (uint32_t)b.m_width * b.m_height;
      if (area < MIN_AREA)              continue;
      if (area > bestArea) { bestArea = area; bestCx = b.m_x; bestCy = b.m_y; found = true; }
    }

    if (found) {
      Zone z = classifyZone(bestCx, bestCy);
      Serial.print("PIXY | cx="); Serial.print(bestCx);
      Serial.print(" cy="); Serial.print(bestCy); Serial.print(" → ");
      switch (z) {
        case TOP_LEFT:  voteTL++; Serial.println("TOP-LEFT");  break;
        case BOT_LEFT:  voteBL++; Serial.println("BOT-LEFT");  break;
        case TOP_RIGHT: voteTR++; Serial.println("TOP-RIGHT"); break;
        case BOT_RIGHT: voteBR++; Serial.println("BOT-RIGHT"); break;
        default:        voteOther++; Serial.println("OTHER");  break;
      }
    } else { missed++; Serial.println("PIXY | NOT DETECTED"); }
    delay(50);
  }

  Serial.println("─── PIXY VOTE SUMMARY ───");
  Serial.print("  TL:"); Serial.print(voteTL);
  Serial.print("  BL:"); Serial.print(voteBL);
  Serial.print("  TR:"); Serial.print(voteTR);
  Serial.print("  BR:"); Serial.print(voteBR);
  Serial.print("  OTHER:"); Serial.print(voteOther);
  Serial.print("  MISSED:"); Serial.println(missed);

  int best = max({voteTL, voteBL, voteTR, voteBR});
  if (best == 0) return UNKNOWN;
  if      (voteTL == best) return TOP_LEFT;
  else if (voteBL == best) return BOT_LEFT;
  else if (voteTR == best) return TOP_RIGHT;
  else                     return BOT_RIGHT;
}

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
//  TRAJECTORIES
//  T1 & T2 → end with Pixy slow-crawl to Side1 wall (0°)
//  T3 & T4 → end with TOF slow-crawl to Side3 wall (180°)
// ============================================================

void runTrajectory1() {
  Serial.println("=== TRAJECTORY 1 START ===");
  intakeOn();
  servoWrite(SERVO_LOAD);   delay(500);
  executeDrive(T1_PHASE1_MS, 0.0);
  servoWrite(SERVO_NEUTRAL); 
   delay(500);
  executeDrive(T1_PHASE2_MS, 0.0);
  motorsStop();             delay(T1_PAUSE_MS);
  servoWrite(SERVO_NEUTRAL); delay(250);
  // executeDrive(T1_PHASE3_MS, 0.0);
  motorsStop(); intakeOff(); delay(200);
  // servoWrite(SERVO_SHOOT);  delay(250);
    Serial.println("T1: Pixy slow-crawl to wall...");
  ema_init = false;
executeDriveUntilPixy(0.0, false, false);   // no full speed — already mid-field
  Serial.println("=== TRAJECTORY 1 DONE ===");
  waitMs(pause_ms);
}

void runTrajectory2() {
  Serial.println("=== TRAJECTORY 2 START ===");
  intakeOn();
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDrive(T2_PHASE1_MS, 0.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT);   delay(250);
  servoWrite(SERVO_LOAD);    delay(250);
  executeDrive(T2_PHASE2_MS, 0.0);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop();
  waitMs(pause_ms);

  // ── Slow Pixy crawl to Side1 wall ──
  Serial.println("=== T2: Pixy crawl to Side1 wall ===");
  ema_init = false;
  executeDriveUntilPixy(0.0, false, false);  // no shoot, no full-speed phase

  Serial.println("=== TRAJECTORY 2 DONE ===");
  
}

void runTrajectory3() {
  Serial.println("=== TRAJECTORY 3 START ===");
  executeTurn(30.0); 
  waitMs(pause_ms);
    intakeOn();
  servoWrite(SERVO_LOAD);   delay(500);
  executeDrive(T3_PHASE1_MS, 30.0);
  servoWrite(SERVO_NEUTRAL); 
   delay(500);
  executeDrive(T1_PHASE2_MS, 30.0);
  motorsStop();             delay(T1_PAUSE_MS);
  servoWrite(SERVO_NEUTRAL); delay(250);
  // executeDrive(T1_PHASE3_MS, 0.0);
  motorsStop(); intakeOff(); delay(200);
  
  // servoWrite(SERVO_SHOOT);  delay(250);
    Serial.println("T1: Pixy slow-crawl to wall...");
  ema_init = false;
executeDriveUntilPixy(5.0, false, false); 
  Serial.println("=== TRAJECTORY 3 DONE ===");
    waitMs(pause_ms);

}

void runTrajectory4() {
  Serial.println("=== TRAJECTORY 4 START ===");
  executeTurn(30.0); waitMs(pause_ms);
  intakeOn();
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDrive(T4_PHASE1_MS, 30.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT);   delay(250);
  servoWrite(SERVO_LOAD);    delay(250);
  executeDrive(T4_PHASE2_MS, 30.0);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop();
  // executeTurn(180.0); waitMs(pause_ms);
  waitMs(pause_ms);

  // ── TOF crawl to Side3 wall at 180° ──
  ema_init = false;
  executeDriveUntilPixy(5.0, false, false);  // no shoot, no full-speed phase


  Serial.println("=== TRAJECTORY 4 DONE ===");
}

// ============================================================
//  runRectangleLap()
//  Full lap: Side1(Pixy) → Turn1 → Side2 → Turn2 →
//            Side3(TOF)  → Turn3 → Side4 → Turn4
// ============================================================
void runRectangleLap() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  // SIDE 1 — Pixy sig3, shoot midway, full-speed phase ON
  Serial.println("=== RECT: Side 1 @ 0° (Pixy stop, shoot midway) ===");
  ema_init = false;
  executeDriveUntilPixy(10.0, true, true);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 1 → -45° ===");
  executeTurn(-75.0, true);        // ← shoot fires as turn start;
  intakeOn();              // resume forward

  servoWrite(SERVO_NEUTRAL);   // ← release shoot after Turn 1
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.2;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);

  Serial.println("=== RECT: Side 2 @ -90° (time) ===");
  executeDrive(breadth_pause, -85.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 2 → -135° ===");
  executeTurn(-135.0);
  waitMs(pause_ms);

  // SIDE 3 — TOF, no shoot
  Serial.println("=== RECT: Side 3 @ 180° (TOF) ===");
executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false, true);  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 3 → 135° ===");
  executeTurn(135.0);
  waitMs(pause_ms);

    lap3Counter++;                          // ← increment here
  int side4Duration = breadth_pause;      // default
  if (lap3Counter >= 3) {
    side4Duration = 200;                  // shorter on every 3rd lap
    lap3Counter = 0;                      // reset counter
    Serial.println("=== LAP 3 SPECIAL: Side 4 shortened ===");
  }

  Serial.println("=== RECT: Side 4 @ 90° (time) ===");
  executeDrive(side4Duration, 95.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 4 → 45° ===");
  executeTurn(45.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Lap complete ===");
}

// ============================================================
//  runRectangleLapFromTurn1()
//  Called after T1/T2 which already approached Side1.
//  Starts from Turn1 — skips Side1, completes the rest.
//  After this, loop with runRectangleLap() (full laps).
// ============================================================
void runRectangleLapFromTurn1() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  Serial.println("=== RECT(T1): Turn 1 → -45° ===");
  executeTurn(-75.0, true);        // ← shoot fires as turn start;
  intakeOn();              // resume forward

  servoWrite(SERVO_NEUTRAL);   // ← add this
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.2;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);

  Serial.println("=== RECT(T1): Side 2 @ -90° (time) ===");
  executeDrive(breadth_pause, -85.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T1): Turn 2 → -135° ===");
  executeTurn(-135.0);
  waitMs(pause_ms);

  // SIDE 3 — TOF, no shoot
  Serial.println("=== RECT(T1): Side 3 @ 180° (TOF) ===");
executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false, true);  waitMs(pause_ms);

  Serial.println("=== RECT(T1): Turn 3 → 135° ===");
  executeTurn(135.0);
  waitMs(pause_ms);

    lap3Counter++;                          // ← increment here
  int side4Duration = breadth_pause;      // default
  if (lap3Counter >= 3) {
    side4Duration = 200;                  // shorter on every 3rd lap
    lap3Counter = 0;                      // reset counter
    Serial.println("=== LAP 3 SPECIAL: Side 4 shortened ===");
  }

  Serial.println("=== RECT(T1): Side 4 @ 90° (time) ===");
  executeDrive(side4Duration, 95.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T1): Turn 4 → 45° ===");
  executeTurn(45.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T1): First lap from Turn1 complete ===");
}

// ============================================================
//  runRectangleLapFromTurn3()
//  Called after T3/T4 which already approached Side3.
//  Starts from Turn3 — skips Side3, completes the rest.
//  After this, loop with runRectangleLapFromSide3() (full laps).
// ============================================================
void runRectangleLapFromTurn3() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);
    zeroIMU();    // ← re-zero here every lap, bot is always at 0° heading


  Serial.println("=== RECT(T3): Turn 3 → 135° ===");
  executeTurn(135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Side 4 @ 90° (time) ===");
  executeDrive(breadth_pause, 95.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Turn 4 → 45° ===");
  executeTurn(45.0);
  waitMs(pause_ms);

  // SIDE 1 — Pixy sig3, shoot midway, full-speed phase ON
  Serial.println("=== RECT(T3): Side 1 @ 0° (Pixy stop, shoot midway) ===");
  ema_init = false;
  executeDriveUntilPixy(10.0, true, true);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Turn 1 → -45° ===");
  executeTurn(-75.0, true);        // ← shoot fires as turn start;
  intakeOn();              // resume forward

  servoWrite(SERVO_NEUTRAL);   // ← release shoot after Turn 1
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.2;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Side 2 @ -90° (time) ===");
  executeDrive(breadth_pause, -85.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Turn 2 → -135° ===");
  executeTurn(-135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): First lap from Turn3 complete ===");
}

// ============================================================
//  runRectangleLapFromSide3()
//  Full lap starting from Side3 approach.
//  Used as the steady-state loop for T3/T4.
// ============================================================
void runRectangleLapFromSide3() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);
    zeroIMU();    // ← re-zero here every lap, bot is always at 0° heading


  // SIDE 3 — TOF, no shoot
  Serial.println("=== RECT(S3): Side 3 @ 180° (TOF) ===");
executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false, true);  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 3 → 135° ===");
  executeTurn(135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Side 4 @ 90° (time) ===");
  executeDrive(breadth_pause, 95.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 4 → 45° ===");
  executeTurn(45.0);
  waitMs(pause_ms);

  // SIDE 1 — Pixy sig3, shoot midway, full-speed phase ON
  Serial.println("=== RECT(S3): Side 1 @ 0° (Pixy stop, shoot midway) ===");
  ema_init = false;
  executeDriveUntilPixy(10.0, true, true);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 1 → -45° ===");
  executeTurn(-75.0, true);        // ← shoot fires as turn start;
  intakeOn();              // resume forward

  servoWrite(SERVO_NEUTRAL);   // ← release shoot after Turn 1
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.2;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Side 2 @ -90° (time) ===");
  executeDrive(breadth_pause, -85.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 2 → -135° ===");
  executeTurn(-135.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Lap complete ===");
}

// ============================================================
//  FIXED TWO-MINUTE MATCH CONTROL
// ============================================================
unsigned long matchElapsedMs() {
  if (!matchClockRunning) return 0;
  return millis() - matchStartMs;
}

bool normalCycleExpired() {
  return matchClockRunning && !fixedEndgameActive && matchElapsedMs() >= ENDGAME_START_MS;
}

bool matchExpired() {
  return matchClockRunning && matchElapsedMs() >= MATCH_DURATION_MS;
}

void serviceAntiJam() {
  if (matchExpired()) return;
  if (fixedEndgameActive) return;
  if (intakeModeState != 1) return;

  unsigned long now = millis();
  if (now - lastAntiJamMs < ANTIJAM_INTERVAL_MS) return;

  motorsStop();
  intakeReverseSlowed();
  delay(ANTIJAM_REVERSE_MS);
  intakeOn();                 // transition to forward restarts the 4.9s timer
  Serial.println("ANTI-JAM: 200ms reverse; 4.9s timer restarted");
}

bool readLargestOrange(int &x, int &y, uint32_t &area) {
  pixy.ccc.getBlocks();
  x = 0;
  y = 0;
  area = 0;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature != SIG_ORANGE) continue;

    uint32_t candidateArea = (uint32_t)b.m_width * b.m_height;
    if (candidateArea < ORANGE_MIN_AREA || candidateArea <= area) continue;

    area = candidateArea;
    x = b.m_x;
    y = b.m_y;
  }

  return area > 0;
}

// One Pixy frame, with purple always taking priority over orange.  That keeps
// the sorter in LOAD whenever a purple ball is mixed into the endgame cluster.
bool readEndgameBall(bool &isPurple, int &x, int &y, uint32_t &area) {
  pixy.ccc.getBlocks();
  uint32_t bestPurpleArea = 0;
  uint32_t bestOrangeArea = 0;
  int purpleX = 0, purpleY = 0, orangeX = 0, orangeY = 0;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto &b = pixy.ccc.blocks[i];
    uint32_t candidateArea = (uint32_t)b.m_width * b.m_height;

    if (b.m_signature == SIG_PURPLE && candidateArea >= ENDGAME_PURPLE_MIN_AREA &&
        candidateArea > bestPurpleArea) {
      bestPurpleArea = candidateArea;
      purpleX = b.m_x;
      purpleY = b.m_y;
    } else if (b.m_signature == SIG_ORANGE && candidateArea >= ORANGE_MIN_AREA &&
               candidateArea > bestOrangeArea) {
      bestOrangeArea = candidateArea;
      orangeX = b.m_x;
      orangeY = b.m_y;
    }
  }

  if (bestPurpleArea > 0) {
    isPurple = true;
    x = purpleX;
    y = purpleY;
    area = bestPurpleArea;
    return true;
  }
  if (bestOrangeArea > 0) {
    isPurple = false;
    x = orangeX;
    y = orangeY;
    area = bestOrangeArea;
    return true;
  }

  isPurple = false;
  x = 0;
  y = 0;
  area = 0;
  return false;
}

// Sweep target: the largest orange blob is the nearest visible orange and is
// always the steering target. Purple only interrupts when it is already inside
// the intake corridor, so it is stored instead of being pushed or released.
bool readSweepTarget(bool &isPathPurple, int &x, int &y, uint32_t &area) {
  pixy.ccc.getBlocks();
  uint32_t closestOrangeArea = 0;
  uint32_t pathPurpleArea = 0;
  int orangeX = 0, orangeY = 0, purpleX = 0, purpleY = 0;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto &b = pixy.ccc.blocks[i];
    uint32_t candidateArea = (uint32_t)b.m_width * b.m_height;

    if (b.m_signature == SIG_ORANGE && candidateArea >= ORANGE_MIN_AREA &&
        candidateArea > closestOrangeArea) {
      closestOrangeArea = candidateArea;
      orangeX = b.m_x;
      orangeY = b.m_y;
    }

    bool purpleInIntakePath = b.m_signature == SIG_PURPLE &&
                              candidateArea >= ENDGAME_PURPLE_MIN_AREA &&
                              abs((int)b.m_x - PIXY_FRAME_CENTRE_X) < 38 &&
                              (b.m_y >= 108 || candidateArea >= 850);
    if (purpleInIntakePath && candidateArea > pathPurpleArea) {
      pathPurpleArea = candidateArea;
      purpleX = b.m_x;
      purpleY = b.m_y;
    }
  }

  if (pathPurpleArea > 0) {
    isPathPurple = true;
    x = purpleX;
    y = purpleY;
    area = pathPurpleArea;
    return true;
  }
  if (closestOrangeArea > 0) {
    isPathPurple = false;
    x = orangeX;
    y = orangeY;
    area = closestOrangeArea;
    return true;
  }

  isPathPurple = false;
  x = 0;
  y = 0;
  area = 0;
  return false;
}

void driveTimedEndgame(unsigned long durationMs, float targetHeading, int speed, bool reverseDrive) {
  unsigned long started = millis();
  while (millis() - started < durationMs && !matchExpired()) {
    serviceAntiJam();

    float error = shortestError(targetHeading, getHeading());
    int correction = constrain((int)(2.2f * error), -55, 55);

    if (reverseDrive) {
      driveMotors(constrain(-speed + correction, -255, 0),
                  constrain(-speed - correction, -255, 0));
    } else {
      driveMotors(constrain(speed + correction, 0, 255),
                  constrain(speed - correction, 0, 255));
    }
  }
  motorsStop();
}

void driveToWallEndgame(float targetHeading, float stopDistCm, unsigned long timeoutMs, int speed) {
  unsigned long started = millis();
  int closeReadings = 0;

  while (millis() - started < timeoutMs && !matchExpired()) {
    serviceAntiJam();

    float distance = getDistanceCm();
    if (distance > 0.0f && distance <= stopDistCm) {
      bool isPurple = false;
      int ballX = 0;
      int ballY = 0;
      uint32_t ballArea = 0;
      bool ballInPath = readEndgameBall(isPurple, ballX, ballY, ballArea) &&
                        abs(ballX - PIXY_FRAME_CENTRE_X) < 42 &&
                        (ballY > 105 || ballArea > 850);
      servoWrite(ballInPath && isPurple ? SERVO_LOAD : SERVO_NEUTRAL);
      if (ballInPath) {
        closeReadings = 0;
        intakeOn();
      } else {
        closeReadings++;
        if (closeReadings >= 3 && millis() - started > 260UL) break;
      }
    } else {
      closeReadings = 0;
      servoWrite(SERVO_NEUTRAL);
    }

    float error = shortestError(targetHeading, getHeading());
    int correction = constrain((int)(2.2f * error), -50, 50);
    driveMotors(constrain(speed + correction, 0, 255),
                constrain(speed - correction, 0, 255));
  }
  motorsStop();
}

bool collectOneOrangeUntil(unsigned long absoluteMatchDeadlineMs) {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);
  int lostFrames = 0;

  while (matchElapsedMs() < absoluteMatchDeadlineMs && !matchExpired()) {
    serviceAntiJam();

    int orangeX = 0;
    int orangeY = 0;
    uint32_t orangeArea = 0;

    if (readLargestOrange(orangeX, orangeY, orangeArea)) {
      lostFrames = 0;
      int xError = orangeX - PIXY_FRAME_CENTRE_X;

      if (orangeY >= ORANGE_CAPTURE_Y || orangeArea >= ORANGE_CAPTURE_AREA) {
        driveTimedEndgame(180, getHeading(), ENDGAME_CREEP_SPEED, false);
        motorsStop();
        Serial.println("ENDGAME: one orange captured by Pixy crossing");
        return true;
      }

      int correction = constrain((int)(0.72f * xError), -70, 70);
      int approachSpeed = orangeY > 105 ? ENDGAME_CREEP_SPEED : ENDGAME_TRACK_SPEED;
      driveMotors(constrain(approachSpeed + correction, 0, 255),
                  constrain(approachSpeed - correction, 0, 255));
    } else {
      lostFrames++;
      if (lostFrames < 4) {
        driveMotors(70, 70);
      } else {
        // Olivia searches only anticlockwise/left, never outside her original right boundary.
        turnAntiClockwise(62);
      }
    }
    delay(18);
  }

  motorsStop();
  return false;
}

bool collectEndgameBallsUntil(unsigned long absoluteMatchDeadlineMs, bool stopAfterOneOrange) {
  intakeOn();
  int lostFrames = 0;
  bool orangeCaptured = false;

  while (matchElapsedMs() < absoluteMatchDeadlineMs && !matchExpired()) {
    bool isPurple = false;
    int ballX = 0;
    int ballY = 0;
    uint32_t ballArea = 0;

    bool targetFound = stopAfterOneOrange
                       ? readSweepTarget(isPurple, ballX, ballY, ballArea)
                       : readEndgameBall(isPurple, ballX, ballY, ballArea);
    if (targetFound) {
      lostFrames = 0;
      servoWrite(isPurple ? SERVO_LOAD : SERVO_NEUTRAL);
      int xError = ballX - PIXY_FRAME_CENTRE_X;

      if (ballY >= ORANGE_CAPTURE_Y || ballArea >= ORANGE_CAPTURE_AREA) {
        unsigned long elapsedNow = matchElapsedMs();
        unsigned long remaining = absoluteMatchDeadlineMs > elapsedNow
                                  ? absoluteMatchDeadlineMs - elapsedNow : 0UL;
        unsigned long captureMs = remaining < 140UL ? remaining : 140UL;
        if (captureMs > 0) driveTimedEndgame(captureMs, getHeading(), ENDGAME_CREEP_SPEED, false);
        motorsStop();

        if (isPurple) {
          endgamePurpleStored = true;
          servoWrite(SERVO_LOAD);
          elapsedNow = matchElapsedMs();
          remaining = absoluteMatchDeadlineMs > elapsedNow
                      ? absoluteMatchDeadlineMs - elapsedNow : 0UL;
          delay(remaining < 180UL ? remaining : 180UL);
          Serial.println("ENDGAME: purple crossing -> LOAD/store");
        } else {
          endgameOrangeEstimate++;
          orangeCaptured = true;
          Serial.print("ENDGAME: orange crossing estimate=");
          Serial.println(endgameOrangeEstimate);
          if (stopAfterOneOrange) {
            servoWrite(SERVO_NEUTRAL);
            return true;
          }
        }

        servoWrite(SERVO_NEUTRAL);
        elapsedNow = matchElapsedMs();
        remaining = absoluteMatchDeadlineMs > elapsedNow
                    ? absoluteMatchDeadlineMs - elapsedNow : 0UL;
        if (remaining > 80UL) driveTimedEndgame(80UL, getHeading(), 75, true);
      } else {
        int correction = constrain((int)(0.72f * xError), -70, 70);
        int speed = ballY > 105 ? ENDGAME_CREEP_SPEED : ENDGAME_TRACK_SPEED;
        driveMotors(constrain(speed + correction, 0, 255),
                    constrain(speed - correction, 0, 255));
      }
    } else {
      servoWrite(SERVO_NEUTRAL);
      lostFrames++;
      if (lostFrames < 4) driveMotors(70, 70);
      else                turnAntiClockwise(62);  // Olivia searches inward/left only
    }
    delay(16);
  }

  motorsStop();
  servoWrite(SERVO_NEUTRAL);
  return orangeCaptured;
}

void executeEndgameTurn(float targetAngle, unsigned long timeoutMs) {
  unsigned long started = millis();
  int stable = 0;

  while (millis() - started < timeoutMs && !matchExpired()) {
    float error = shortestError(targetAngle, getHeading());
    if (abs(error) <= 7.0f) {
      motorsStop();
      if (++stable >= 3) break;
    } else {
      stable = 0;
      int speed = constrain((int)(55.0f + 3.8f * abs(error)), 65, 230);
      if (error > 0) turnClockwise(speed);
      else           turnAntiClockwise(speed);
    }
    delay(5);
  }
  motorsStop();
}

void homeToOliviaCorner() {
  Serial.println("ENDGAME: homing to Olivia bottom-right corner");
  intakeOn();
  servoWrite(SERVO_NEUTRAL);
  executeEndgameTurn(ENDGAME_CORNER_LONG_DEG, 650UL);
  driveToWallEndgame(ENDGAME_CORNER_LONG_DEG, ENDGAME_WALL_CLEAR_CM, 620UL, ENDGAME_HOME_SPEED);
  executeEndgameTurn(ENDGAME_CORNER_SIDE_DEG, 650UL);
  driveToWallEndgame(ENDGAME_CORNER_SIDE_DEG, ENDGAME_WALL_CLEAR_CM, 560UL, ENDGAME_HOME_SPEED);
}

void fireOnePulse(unsigned long pulseMs) {
  motorsStop();
  intakeReverse();
  servoWrite(SERVO_SHOOT);
  delay(pulseMs);
  servoWrite(SERVO_NEUTRAL);
  intakeOn();
}

void waitUntilMatchTime(unsigned long targetMs) {
  motorsStop();
  while (matchElapsedMs() < targetMs && !matchExpired()) {
    intakeOn();
    delay(4);
  }
}

void returnToRampBefore(unsigned long deadlineMs) {
  if (matchExpired() || matchElapsedMs() >= deadlineMs) return;

  unsigned long remaining = deadlineMs - matchElapsedMs();
  executeEndgameTurn(ENDGAME_SHOOT_DEG, remaining < 420UL ? remaining : 420UL);

  if (matchElapsedMs() >= deadlineMs || matchExpired()) return;
  remaining = deadlineMs - matchElapsedMs();
  if (remaining > 80UL) {
    unsigned long driveBudget = remaining - 40UL;
    if (driveBudget > 460UL) driveBudget = 460UL;
    driveToWallEndgame(ENDGAME_SHOOT_DEG, ENDGAME_RAMP_CLEAR_CM,
                       driveBudget, ENDGAME_HOME_SPEED);
  }
  motorsStop();
}

void slowReleaseAtRamp(unsigned long durationMs) {
  unsigned long remainingMatch = MATCH_DURATION_MS - matchElapsedMs();
  if (durationMs > remainingMatch) durationMs = remainingMatch;
  motorsStop();
  servoWrite(SERVO_NEUTRAL);     // never use SHOOT during the slow ramp unload
  intakeReverseSlowed();
  delay(durationMs);
  intakeOff();
  endgameOrangeEstimate = 0;
  Serial.println("ENDGAME: slow orange release complete (no shot)");
}

unsigned long estimateClosestBallTripMs(bool isPurple, int y, uint32_t area) {
  if (isPurple) return 180UL + SWEEP_WALL_RESERVE_MS;

  int boundedY = constrain(y, 20, 170);
  unsigned long approachMs = 720UL - (unsigned long)(boundedY - 20) * 4UL;
  if (approachMs < 120UL) approachMs = 120UL;
  if (area >= 1400 && approachMs > 90UL) approachMs -= 90UL;
  else if (area >= 700 && approachMs > 45UL) approachMs -= 45UL;
  return approachMs + SWEEP_WALL_RESERVE_MS;
}

void runMirroredWallFallbackUntil(unsigned long deadlineMs) {
  if (matchExpired() || matchElapsedMs() >= deadlineMs) return;
  unsigned long budget = deadlineMs - matchElapsedMs();
  Serial.println("SWEEP: closest orange cannot return in time -> Olivia right-wall attempt");
  intakeOn();
  servoWrite(SERVO_NEUTRAL);
  driveToWallEndgame(SWEEP_FALLBACK_WALL_DEG, ENDGAME_RAMP_CLEAR_CM,
                     budget, SWEEP_REVERSE_SPEED);
}

void runSweepForTimedShot(unsigned long shotAtMs) {
  if (matchExpired() || matchElapsedMs() >= shotAtMs) {
    if (!matchExpired()) fireOnePulse(190UL);
    return;
  }

  unsigned long sweepStarted = matchElapsedMs();
  unsigned long hardDeadline = sweepStarted + SWEEP_MAX_TOTAL_MS;
  if (hardDeadline > shotAtMs) hardDeadline = shotAtMs;

  // No turn-away manoeuvre: apply maximum reverse output immediately. The
  // rear wall is time-capped because this robot has no rear-facing distance sensor.
  unsigned long available = hardDeadline - matchElapsedMs();
  if (available > SWEEP_WALL_RESERVE_MS + 260UL) {
    unsigned long reverseMs = available - SWEEP_WALL_RESERVE_MS - 260UL;
    if (reverseMs > SWEEP_REVERSE_MAX_MS) reverseMs = SWEEP_REVERSE_MAX_MS;
    if (reverseMs < 100UL) reverseMs = 100UL;
    driveTimedEndgame(reverseMs, ENDGAME_SHOOT_DEG, SWEEP_REVERSE_SPEED, true);
  }

  unsigned long collectDeadline = hardDeadline > SWEEP_WALL_RESERVE_MS
                                  ? hardDeadline - SWEEP_WALL_RESERVE_MS
                                  : hardDeadline;
  bool isPathPurple = false;
  int targetX = 0;
  int targetY = 0;
  uint32_t targetArea = 0;
  bool targetSeen = readSweepTarget(isPathPurple, targetX, targetY, targetArea);

  available = hardDeadline > matchElapsedMs() ? hardDeadline - matchElapsedMs() : 0UL;
  unsigned long tripEstimate = targetSeen
                               ? estimateClosestBallTripMs(isPathPurple, targetY, targetArea)
                               : SWEEP_MAX_TOTAL_MS + 1UL;
  bool targetFits = targetSeen && tripEstimate <= available;

  Serial.print("SWEEP ETA | available="); Serial.print(available);
  Serial.print("ms estimate="); Serial.print(tripEstimate);
  Serial.print("ms fits="); Serial.println(targetFits ? 1 : 0);

  if (targetFits && matchElapsedMs() < collectDeadline) {
    // stopAfterOneOrange selects the largest orange every frame. A purple only
    // interrupts when already centred in the intake corridor.
    collectEndgameBallsUntil(collectDeadline, true);
  } else {
    runMirroredWallFallbackUntil(collectDeadline);
  }

  returnToRampBefore(hardDeadline);
  waitUntilMatchTime(shotAtMs);
  if (!matchExpired()) {
    fireOnePulse(190UL);
    if (endgameOrangeEstimate > 0) endgameOrangeEstimate--;
  }
}

void shootAllRemaining() {
  Serial.println("ENDGAME 1:57: SHOOT ALL remaining orange balls");
  while (!matchExpired()) {
    motorsStop();
    intakeReverse();
    servoWrite(SERVO_SHOOT);
    unsigned long remaining = MATCH_DURATION_MS - matchElapsedMs();
    delay(remaining < 185UL ? remaining : 185UL);
    if (matchExpired()) break;

    // Brief feed/reload gap; stored purple remains in its separate LOAD path.
    servoWrite(SERVO_NEUTRAL);
    intakeOn();
    remaining = MATCH_DURATION_MS - matchElapsedMs();
    delay(remaining < 70UL ? remaining : 70UL);
  }
  endgameOrangeEstimate = 0;
}

void runFixedTwoMinuteFinish() {
  if (fixedEndgameDone || !matchClockRunning) return;

  fixedEndgameActive = true;
  lastAntiJamMs = millis();
  motorsStop();
  servoWrite(SERVO_NEUTRAL);
  intakeOn();
  Serial.println("=== OLIVIA FIXED 2:00 FINISH START ===");

  // 0:00-1:50 is the supplied route.  At 1:50 abandon the current leg and
  // home to the two-wall intersection shown in the drawing.
  homeToOliviaCorner();

  // Intake the visible corner cluster. Purple is always selected first and sent
  // to LOAD/storage; orange stays on the normal feed path.
  if (matchElapsedMs() < 112950UL) {
    driveTimedEndgame(110UL, ENDGAME_CORNER_SIDE_DEG, 95, true);
    collectEndgameBallsUntil(112950UL, false);
  }

  // Carry the cluster to the ramp, then roll the oranges out slowly. This is a
  // release operation: SHOOT is deliberately never selected here.
  returnToRampBefore(113650UL);
  if (matchElapsedMs() < FIRST_SINGLE_SHOT_MS) {
    unsigned long releaseBudget = FIRST_SINGLE_SHOT_MS - matchElapsedMs();
    if (releaseBudget > 220UL) releaseBudget -= 200UL;
    else releaseBudget = 0;
    if (releaseBudget > 360UL) releaseBudget = 360UL;
    if (releaseBudget > 0) slowReleaseAtRamp(releaseBudget);
  }

  // Only one powered shot after the initial slow unload. Fire immediately so
  // the complete 1:54-to-1:55 interval remains available for the next sweep.
  if (!matchExpired()) {
    Serial.println("ENDGAME: one single shot after slow release");
    fireOnePulse(190UL);
  }

  // Re-sweep, intake one orange, return, and shoot once at 1:55 and 1:56.
  runSweepForTimedShot(SECOND_SINGLE_SHOT_MS);
  runSweepForTimedShot(THIRD_SINGLE_SHOT_MS);

  // Stay at the ramp and clear every remaining orange from 1:57 to 2:00.
  returnToRampBefore(FINAL_FLUSH_START_MS);
  waitUntilMatchTime(FINAL_FLUSH_START_MS);
  shootAllRemaining();

  motorsStop();
  intakeOff();
  servoWrite(SERVO_NEUTRAL);
  fixedEndgameActive = false;
  Serial.print("ENDGAME purpleStored="); Serial.print(endgamePurpleStored ? 1 : 0);
  Serial.print(" orangeEstimate="); Serial.println(endgameOrangeEstimate);
  Serial.println("=== OLIVIA MATCH COMPLETE ===");
}

// ============================================================
//  executeDriveUntilPixy()
//  Drives along targetHeading until EMA-filtered sig3 top_cy
//  reaches SIG3_STOP_TOP_CY or timeout fires.
//
//  useFullSpeedPhase=true  → full speed for SIG3_FULLSPEED_MS,
//                            then slow to SIG3_FWD_SPEED.
//                            Use this for normal rectangle laps
//                            (bot starts far from wall).
//  useFullSpeedPhase=false → SIG3_FWD_SPEED the whole time.
//                            Use this for trajectory endings
//                            (bot already near wall).
// ============================================================
void executeDriveUntilPixy(float targetHeading, bool shootMidway, bool useFullSpeedPhase) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  while (true) {
    unsigned long now     = millis();
    unsigned long elapsed = now - startTime;

    if (normalCycleExpired() || matchExpired()) break;
    serviceAntiJam();

    // ── Timeout fallback ───────────────────────────────────
    if (elapsed > SIG3_TIMEOUT_MS) {
      Serial.println("PIXY STOP: TIMEOUT — stopping.");
      break;
    }

    // ── Midway shoot state machine (non-blocking) ──────────
// REPLACE the entire shoot state machine block with this:
if (shootMidway && !shootTriggered && elapsed >= 500) {
  Serial.println("Mid-drive SHOOT triggered — holding until Turn 1");
  servoWrite(SERVO_SHOOT);
  shootTriggered = true;
}

    // ── Pixy read + EMA + stop check ──────────────────────
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;

      // Find largest sig3 blob
      pixy.ccc.getBlocks();
      int      best_raw_top_cy = -1;
      uint32_t bestArea        = 0;

      for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        auto &b = pixy.ccc.blocks[i];
        if (b.m_signature != 3) continue;
        uint32_t area = (uint32_t)b.m_width * b.m_height;
        if (area < SIG3_MIN_AREA) continue;
        if (area > bestArea) {
          bestArea        = area;
          best_raw_top_cy = b.m_y - b.m_height / 2;
        }
      }

      if (best_raw_top_cy >= 0) {
        // Seed or update EMA
        if (!ema_init) {
          ema_top_cy = (float)best_raw_top_cy;
          ema_init   = true;
        } else {
          // Outlier rejection — skip frames that jump too much
          if (abs(best_raw_top_cy - ema_top_cy) < 30) {
            ema_top_cy = SIG3_EMA_ALPHA * best_raw_top_cy
                         + (1.0f - SIG3_EMA_ALPHA) * ema_top_cy;
          } else {
            Serial.println("PIXY: outlier rejected");
          }
        }

        Serial.print("PIXY | raw_top_cy="); Serial.print(best_raw_top_cy);
        Serial.print("  ema_top_cy=");      Serial.print((int)ema_top_cy);
        Serial.print("  target=");          Serial.println(SIG3_STOP_TOP_CY);

        // Stop condition: past guard time AND EMA reached target
        if (elapsed > SIG3_GUARD_MS && ema_top_cy >= SIG3_STOP_TOP_CY) {
          Serial.println("PIXY STOP: target top_cy reached — stopping.");
          break;
        }
      } else {
        Serial.println("PIXY | sig3 not detected");
      }

      // ── Speed: full-speed phase then slow ────────────────
      int currentSpeed = (useFullSpeedPhase && elapsed < SIG3_FULLSPEED_MS)
                         ? FWD_BASE_SPEED
                         : SIG3_FWD_SPEED;
      runForwardPID(targetHeading, prevErr, integ, currentSpeed);
    }
  }

  motorsStop();
}

// ============================================================
//  executeDrive()
// ============================================================
void executeDrive(unsigned long durationMs, float targetHeading) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  while (millis() - startTime < durationMs && !normalCycleExpired() && !matchExpired()) {
    unsigned long now = millis();
    serviceAntiJam();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPID(targetHeading, prevErr, integ, FWD_BASE_SPEED);
    }
  }
  motorsStop();
}

// ============================================================
//  executeDriveUntilClose() — Side 3 (TOF)
// ============================================================
void executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 2000;

  bool intakeSwitched = false;
    bool servoShot       = false;          // ← ADD

  if (reverseIntakeAtStart) intakeReverseSlowed();   // ← only when caller asks
  else                      intakeOn();        // ← normal: keep forward


  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;   // ← ADD

    if (normalCycleExpired() || matchExpired()) break;
    serviceAntiJam();

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("TOF: MAX TIMEOUT — stopping.");
      break;
    }

    //     if (!intakeSwitched && elapsed >= 250) {   // ← ADD
    //   intakeOn();                               // ← ADD
    //   intakeSwitched = true;                    // ← ADD
    //   Serial.println("Side3: intake switched to forward"); // ← ADD
    // }   


        // ── Side3 servo shoot at 50ms ──────────────────────────
    if (reverseIntakeAtStart && !servoShot && elapsed >= 50) {
      servoWrite(SERVO_SHOOT);
      servoShot = true;
      Serial.println("Side3: servo SHOOT at 50ms");
    }

    // ── Side3 intake forward + servo neutral at 1000ms ─────
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 250) {
      intakeOn();
      servoWrite(SERVO_NEUTRAL);
      intakeSwitched = true;
      Serial.println("Side3: intake ON + servo NEUTRAL at 1000ms");
    }

    if (shootMidway && !shootDone) {
      if (!shootTriggered && (now - startTime >= 600)) {
        servoWrite(SERVO_SHOOT);
        shootTriggered = true;
        shootStartTime = now;
        shootPhase = 1;
      }
      if (shootPhase == 1 && (now - shootStartTime >= 250)) { servoWrite(SERVO_NEUTRAL); shootPhase = 2; }
      if (shootPhase == 2 && (now - shootStartTime >= 500)) { shootDone = true; }
    }

    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      float dist = getDistanceCm();
      Serial.print("TOF | Dist:");
      if (dist >= 0) { Serial.print(dist, 1); Serial.print("cm"); }
      else             Serial.print("OOR");

      if (dist > 0 && dist < stopDistCm && now - startTime > 800) {
        Serial.println(" → STOP");
        motorsStop();
        break;
      }
      Serial.println();
      runForwardPID(targetHeading, prevErr, integ, FWD_BASE_SPEED);
    }
  }
  motorsStop();
}

// ============================================================
//  executeTurn()
// ============================================================
void executeTurn(float targetAngle, bool shootDuringTurn) {  float prevErr     = 0.0;
  float integ       = 0.0;
  int   stableCount = 0;
  unsigned long lastPIDTime = millis();
  unsigned long turnStarted = millis();

  if (shootDuringTurn) {
    intakeReverse();            // ← revrses immediately as turn begins
    Serial.println("SHOOT triggered during Turn 1");
  }

  while (true) {
    unsigned long now = millis();
    if ((!fixedEndgameActive && normalCycleExpired()) || matchExpired()) {
      motorsStop();
      return;
    }
    if (now - turnStarted >= TURN_TIMEOUT_MS) {
      motorsStop();
      Serial.println("TURN: timeout safety exit");
      return;
    }
    serviceAntiJam();
    if (now - lastPIDTime >= TRN_INTERVAL) {
      lastPIDTime = now;
      float heading = getHeading();
      float error   = shortestError(targetAngle, heading);

      if (abs(error) <= TRN_DEADBAND) {
        motorsStop();
        integ = 0.0; prevErr = error;
        stableCount++;
        if (stableCount >= TRN_STABLE_COUNT) {
          Serial.print("Turn done. H:"); Serial.println(heading, 1);
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
void runForwardPID(float targetHeading, float &prevErr, float &integ, int baseSpeed) {
  float heading = getHeading();
  float error   = shortestError(targetHeading, heading);
  float dt      = FWD_INTERVAL / 1000.0;

  if (abs(error) <= FWD_DEADBAND) {
    integ = 0.0; prevErr = error;
    driveMotors(baseSpeed, baseSpeed);
    return;
  }

  integ += error * dt;
  integ  = constrain(integ, -50, 50);
  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float correction = (FWD_KP * error) + (FWD_KI * integ) + (FWD_KD * derivative);
  correction = constrain(correction, -FWD_MAX_CORRECT, FWD_MAX_CORRECT);

  int leftSpeed  = constrain(baseSpeed + (int)correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - (int)correction, 0, 255);

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("FWD | H:"); Serial.print(heading, 1);
  Serial.print(" E:"); Serial.print(error, 1);
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
  int speed = constrain((int)abs(output), TRN_MIN_SPEED, TRN_MAX_SPEED);

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
    tofReady = false;
    Serial.println("VL53L0X not found! Continuing without TOF.");
    return;
  }
  tofReady = true;
  Serial.println("VL53L0X Ready.");
}

float getDistanceCm() {
  if (!tofReady) return -1.0f;
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) return measure.RangeMilliMeter / 10.0;
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
  intakeModeState = 0;
}

void intakeOn() {
  if (intakeModeState != 1) lastAntiJamMs = millis();
  intakeModeState = 1;
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, INTAKE_SPEED);
}

void intakeReverse() {
  intakeModeState = 2;
  analogWrite(INTAKE_IN1, INTAKE_SPEED);
  analogWrite(INTAKE_IN2, 0);
}

void intakeReverseSlowed() {
  intakeModeState = 2;
  analogWrite(INTAKE_IN1, SLOW);
  analogWrite(INTAKE_IN2, 0);
}

void intakeOff() {
  intakeModeState = 0;
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
  for (int i = 0; i < 20; i++) { bno.getSensorEvent(&imuData); delay(10); }
  yawOffset = getRawYaw();
  Serial.print("IMU Zeroed at: "); Serial.println(yawOffset, 2);
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
        return -degrees(atan2(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)));
      }
    }
  }
}

float getHeading() { return wrapAngle(getRawYaw() - yawOffset - driftCorrection); }
float shortestError(float target, float cur){ return wrapAngle(wrapAngle(target) - wrapAngle(cur)); }
float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ============================================================
//  MOTORS
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0)       { analogWrite(M1A, 0);          analogWrite(M1B, leftSpeed);  }
  else if (leftSpeed < 0)  { analogWrite(M1A, -leftSpeed); analogWrite(M1B, 0);          }
  else                     { analogWrite(M1A, 0);          analogWrite(M1B, 0);          }

  if (rightSpeed > 0)      { analogWrite(M2A, rightSpeed); analogWrite(M2B, 0);          }
  else if (rightSpeed < 0) { analogWrite(M2A, 0);          analogWrite(M2B, -rightSpeed);}
  else                     { analogWrite(M2A, 0);          analogWrite(M2B, 0);          }
}

void turnClockwise(int speed)     { analogWrite(M1A, 0);     analogWrite(M1B, speed); analogWrite(M2A, 0);     analogWrite(M2B, speed); }
void turnAntiClockwise(int speed) { analogWrite(M1A, speed); analogWrite(M1B, 0);     analogWrite(M2A, speed); analogWrite(M2B, 0);     }
void motorsStop()                 { analogWrite(M1A, 0); analogWrite(M1B, 0); analogWrite(M2A, 0); analogWrite(M2B, 0); }
void setMotorPins()               { pinMode(M1A,OUTPUT); pinMode(M1B,OUTPUT); pinMode(M2A,OUTPUT); pinMode(M2B,OUTPUT); }
void waitMs(int ms) {
  motorsStop();
  unsigned long started = millis();
  while (millis() - started < (unsigned long)ms && !normalCycleExpired() && !matchExpired()) {
    serviceAntiJam();
    delay(2);
  }
}
