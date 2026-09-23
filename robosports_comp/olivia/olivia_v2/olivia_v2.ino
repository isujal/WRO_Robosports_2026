// -------------------- OLIVIA -------------

// ============================================================
//  INCLUDES
// ============================================================
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>
#include <Pixy2SPI_SS.h>
#include <FastLED.h>

// ============================================================
//  LED (FastLED)
// ============================================================
#define LED_PIN   37
#define NUM_LEDS  1
CRGB leds[NUM_LEDS];

// ============================================================
//  ★ TUNABLE CONSTANTS ★
// ============================================================

// --- Match Timing ---
const unsigned long MATCH_DURATION_MS  = 999999000UL;  // hard stop — safety net, unchanged
const unsigned long PHASE1_LAP_END_MS  = 100000;        // ★ stop looping laps at this point
const unsigned long PHASE2_START_MS    = 111000;        // ★ run runRectangleLap3() at this point

// --- Trajectory 1 (Bot-Left: Purple → Orange) ---
const unsigned long T1_PHASE1_MS = 470;
const unsigned long T1_PHASE2_MS = 300;
const unsigned long T1_PHASE3_MS = 200;
const unsigned long T1_PAUSE_MS  = 200;
const int           T1_SPEED     = 180;

// --- Trajectory 2 (Top-Left: Orange → Purple) ---
const unsigned long T2_PHASE1_MS = 400;
const unsigned long T2_PHASE2_MS = 250;
const int           T2_SPEED     = 180;

// --- Trajectory 3 (Bot-Right: 60° turn + T1 logic + realign) ---
const unsigned long T3_PHASE1_MS = 500;
const unsigned long T3_PHASE2_MS = 300;
const unsigned long T3_PHASE3_MS = 200;
const unsigned long T3_PAUSE_MS  = 200;
const int           T3_SPEED     = 180;

// --- Trajectory 4 (Top-Right: 60° turn + T2 logic + realign) ---
const unsigned long T4_PHASE1_MS = 450;
const unsigned long T4_PHASE2_MS = 250;
const int           T4_SPEED     = 180;

// --- Trajectory Encoder Tick Counts (★ TUNE after first run) ---
const long T1_ENC_PHASE1 = 250;
const long T1_ENC_PHASE2 = 120;

const long T2_ENC_PHASE1 = 250;
const long T2_ENC_PHASE2 = 120;

const long T3_ENC_PHASE1 = 280;
const long T3_ENC_PHASE2 = 120;

const long T4_ENC_PHASE1 = 250;
const long T4_ENC_PHASE2 = 120;

// --- Rectangle Driving ---
const float STOP_DISTANCE_CM   = 9.0;   // Side 3 TOF stop distance
int         breadth_pause      = 1200;
int         pause_ms           = 5;

// --- Forward PID ---
const float FWD_KP            = 2.6;
const float FWD_KI            = 0.0;
const float FWD_KD            = 0.5;
const int   FWD_BASE_SPEED    = 220;
const int   FWD_BASE_SPEED_Slow = 150;
const int   FWD_MAX_CORRECT   = 100;
const float FWD_DEADBAND      = 0.8;
const int   FWD_INTERVAL      = 20;

// --- Rectangle-Lap Speed (does NOT affect runTrajectory1–4) ---
const int            RECT_SPEED       = 55;
const int            RECT_SPEED_2     = 60;
const unsigned long  RECT_FULLSPEED_MS = 500;

// --- Turn PID ---
const float TRN_KP          = 5.0;
const float TRN_KI          = 0.0;
const float TRN_KD          = 0.8;
const int   TRN_MIN_SPEED   = 60;
const int   TRN_MAX_SPEED   = 255;
const float TRN_DEADBAND    = 8.0;
const int   TRN_INTERVAL    = 20;
const int   TRN_STABLE_COUNT = 8;

// --- Pixy2 Sig3 Stop (Side 1) ---
#define SIG3_STOP_TOP_CY    160
#define SIG3_MIN_AREA       500     // ignore blobs smaller than this
#define SIG3_EMA_ALPHA      0.7f    // EMA smoothing (lower = smoother)
#define SIG3_TIMEOUT_MS     3500    // fallback timeout if sig3 never reaches target
#define SIG3_GUARD_MS       800     // minimum drive time before stop is allowed
#define SIG3_FULLSPEED_MS   500     // drive at full speed before slowing for detection
#define SIG3_FWD_SPEED      50      // detection speed for Side 1

// --- Pixy2 Zone Detection ---
#define SIG_PURPLE          1
#define MIN_AREA            75
#define ROI_TOP_Y           55

#define SPLIT_X             208     // ★ TUNED from actual mat readings
#define SPLIT_Y             88      // ★ TUNED
#define DEAD_X              15      // ★ TUNED
#define DEAD_Y              4       // ★ TUNED

#define PIXY_SAMPLE_MS      1500

// --- Zone EMA Filtering ---
#define ZONE_EMA_ALPHA      0.4f
#define ZONE_OUTLIER_JUMP   30

// --- Encoder Ticks: Side 1 (★ TUNE after one test run) ---
long ENC_SIDE1_SLOW_COUNT = 400;   // 80% of trajectory — full speed below this
long ENC_SIDE1_STOP_COUNT = 550;

// --- Encoder Ticks: Side 2 ---
long       ENC_SIDE2_SLOW_COUNT      = 120;   // non-const — saved/restored each call
const long ENC_SIDE2_STOP_COUNT      = 250;
long       ENC_SIDE2_LAP3_SLOW_COUNT = 0;     // start slow immediately (no ramp)
const long ENC_SIDE2_LAP3_STOP_COUNT = 200;   // stop earlier than normal 250
const int  SIDE2_LAP3_SPEED          = 65;    // slower speed after lap 3

// --- Encoder Ticks: Side 3 ---
long ENC_SIDE3_SLOW_COUNT     = 400;
long ENC_SIDE3_STOP_COUNT     = 550;
long ENC_END_SIDE3_SLOW_COUNT = 300;
long ENC_END_SIDE3_STOP_COUNT = 400;

// --- Encoder Ticks: Side 4 ---
const long ENC_SIDE4_SLOW_COUNT = 190;
const long ENC_SIDE4_STOP_COUNT = 300;

// ============================================================
//  HARDWARE PINS
// ============================================================

// --- Motors ---
const int M1A = 42;
const int M1B = 41;
const int M2A = 15;
const int M2B = 16;

// --- IMU (BNO08x) ---
#define BNO08X_SDA 18
#define BNO08X_SCL 17
#define BNO08X_RST 12

// --- TOF (VL53L0X) ---
#define TOF_SDA 2
#define TOF_SCL 1

// --- Intake ---
const int INTAKE_IN1   = 38;
const int INTAKE_IN2   = 39;
const int INTAKE_SPEED = 255;
const int SLOW         = 120;

// --- Servo ---
#define SERVO_PIN 40
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

const int SERVO_NEUTRAL = 20;
const int SERVO_SHOOT   = 64;   // 61
const int SERVO_LOAD    = 140;

// --- Encoder ---
#define ENC_A  4
#define ENC_B  5

// --- Start Switch ---
#define SWITCH_PWR              48
#define SWITCH_SIG              21
#define SWITCH_DEBOUNCE_READS   5
#define SWITCH_POLL_MS          10

// ============================================================
//  RUNTIME STATE & FLAGS
// ============================================================

// --- Match State ---
unsigned long matchStartTime = 0;
bool          matchEnded     = false;

// --- Lap & Drift Tracking ---
int   lap3Counter    = 0;     // counts laps; every 3rd lap uses shorter Side 4
int   lapCounter     = 0;     // counts laps for heading drift correction
float headingOffset  = 0.0;   // added to all forward-drive headings every 5 laps
float driftCorrection = 0.0;  // subtracted from heading every 2 laps

// --- Zone EMA State (Purple Ball Classification) ---
float zone_ema_cx   = 0.0f;
float zone_ema_cy   = 0.0f;
bool  zone_ema_init = false;

// --- Cross-Lap Flags ---
bool firstLap          = true;   // first Side 2 always crawls at slow speed
bool slowSide2NextLap  = false;  // set in lap3 block → next Side 2 crawls
bool lap3Side1NextCall = false;  // set in lap3 block → next Side 1 crawls

// --- Shoot State ---
bool shootTriggered = false;
bool shootDone      = false;

// --- Pixy EMA State (Side 1 / Sig3 Stop) ---
float ema_top_cy = -1.0f;
bool  ema_init   = false;

// --- Encoder ISR State ---
volatile long encCount = 0;

// ============================================================
//  HARDWARE OBJECTS
// ============================================================
Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;
float             yawOffset = 0.0;

Adafruit_VL53L0X  lox;
Pixy2SPI_SS       pixy;

// ============================================================
//  ENUMS
// ============================================================
enum Zone { UNKNOWN, TOP_LEFT, BOT_LEFT, TOP_RIGHT, BOT_RIGHT };
Zone decidedZone = UNKNOWN;
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
void  runRectangleLapFromTurn1();
void  runRectangleLapFromTurn3();
void  runRectangleLapFromSide3();

void executeDriveToEncoder(long targetTicks, float targetHeading, int speed);
// void  runRectangleFromSide1();
// void  runRectangleFromSide3();
// void  runRectangleLap();
// void  runRectangleLapFromSide3();
// void  runRectangleLapFromTurn1();

void  executeDrive(unsigned long durationMs, float targetHeading);
void  executeDriveUntilPixy(float targetHeading, bool shootMidway, bool useFullSpeedPhase);
void  executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart = false);
void executeDriveUntilEnc(float targetHeading, long stopCount, int speed);
// --- Rectangle-lap-only slow variants (RECT_SPEED = 140) ---
void  executeDriveRect(unsigned long durationMs, float targetHeading, bool reverseIntakeAtStart = false);
void  executeDriveUntilPixyRect(float targetHeading, bool shootMidway, bool useFullSpeedPhase);
void  executeDriveUntilCloseRect(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart = false);

void  executeTurn(float targetAngle, bool shootDuringTurn = false);
void  runForwardPID(float targetHeading, float &prevErr, float &integ, int baseSpeed);
void  runTurnPID(float targetAngle, float &prevErr, float &integ);

void  driveMotors(int leftSpeed, int rightSpeed);
void  turnClockwise(int speed);
void  turnAntiClockwise(int speed);
void  motorsStop();
void  setMotorPins();
void  waitMs(int ms);
void executeDriveUntilEncRect(float targetHeading, long slowCount, long stopCount, int fastSpeed, int slowSpeed, bool reverseIntakeAtStart = false, bool shootMidway = false);
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

void  initStartSwitch();
bool  isStartSwitchOn();
void  waitForStartSwitch();
void  runMission(Zone z);

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
bool matchTimeUp() {
  if (matchEnded) return true;
  if (millis() - matchStartTime >= MATCH_DURATION_MS) {
    matchEnded = true;
    return true;
  }
  return false;
}

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_B) == HIGH) encCount++;
  else                            encCount--;
}

void resetEncoder() { noInterrupts(); encCount = 0; interrupts(); }

bool phase1TimeUp() {              // ★ ADD — 25s mark: stop lap loop
  return millis() - matchStartTime >= PHASE1_LAP_END_MS;
}

bool phase2TimeUp() {              // ★ ADD — 28s mark: fire the finale
  return millis() - matchStartTime >= PHASE2_START_MS;
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

  pinMode(ENC_A, INPUT_PULLUP);
pinMode(ENC_B, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);
  pixy.changeProg("color_connected_components");
  Serial.println("Pixy2 Ready.");

  Serial.println("All hardware ready. Sampling Pixy2...");
  delay(500);
  zeroIMU();

  // decidedZone is set live in waitForStartSwitch() — no pre-detection needed here
  decidedZone = UNKNOWN;
  setLedForZone(UNKNOWN);

  waitForStartSwitch();
  matchStartTime = millis();      // ★ clock starts the instant the switch triggers
  matchEnded = false;
  leds[0] = CRGB::Black;         // ← LED OFF the moment switch fires
  FastLED.show();

  runMission(decidedZone);
}

void loop() {
  motorsStop();
  delay(100);
}


// ============================================================
//  runLapsThenFinale()
//  0s–25s : loop runRectangleLap()
//  25s–28s: idle, motors/intake off, waiting
//  28s–30s: run runRectangleLap3() once
//  30s    : matchTimeUp() hard-stops everything (safety net,
//           still enforced inside every drive/turn function)
// ============================================================
void runLapsThenFinale() {
  while (!matchTimeUp()) {
    runRectangleLap();
  }
  motorsStop();
  intakeOff();
  servoWrite(SERVO_NEUTRAL);
  Serial.println("=== MATCH TIME UP — STOPPED ===");
}
// ============================================================
//  runMission()
// ============================================================
void runMission(Zone z) {
  if (z == BOT_LEFT) {
    // T1 ends at Side1 wall → skip Side1, start from Turn1
    runTrajectory1();
    runRectangleLapFromTurn1();
    runLapsThenFinale();
    return;

  } else if (z == TOP_LEFT) {
    // T2 also ends at Side1 wall → same pattern as T1
    runTrajectory2();
    runRectangleLapFromTurn1();
    runLapsThenFinale();
    return;

  } else if (z == BOT_RIGHT) {
    // T3 ends at Side3 wall → skip Side3, start from Turn3
    runTrajectory3();
    runRectangleLapFromTurn1();
    runLapsThenFinale();
    return;

  } else if (z == TOP_RIGHT) {
    // T4 ends at Side3 wall → same pattern as T3
    runTrajectory4();
    runRectangleLapFromTurn1();
    runLapsThenFinale();
    return;

  } 
  // else {
  //       runTrajectory4();
  //   runRectangleLapFromTurn1();
  //   runLapsThenFinale();
  //   return;
  // }
  
  
  else {
    // runTrajectory1();
    // runRectangleLapFromTurn1();
    // while (true) { runRectangleLap2(); }

    runLapsThenFinale();
    return;
  }
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
  unsigned long lastPixySample = 0;
  unsigned long lastPrint      = millis();

  Zone lastSeen         = UNKNOWN;
  int  consecutiveCount = 0;
  int  missCount        = 0;                 // ← ADD: tracks consecutive UNKNOWN frames
  const int HYSTERESIS       = 2;            // frames needed to lock in a color
  const int UNKNOWN_HYST     = 10;            // frames needed to revert to white // ← ADD

  while (true) {
    unsigned long now = millis();

    // ── Live Pixy tracking every 50ms ──────────────────────
    if (now - lastPixySample >= 20) {
      lastPixySample = now;
      Zone detected = samplePurpleOnce();

      if (detected != UNKNOWN) {
        missCount = 0;                       // ← ADD: any real detection resets the miss streak

        if (detected == lastSeen) {
          consecutiveCount++;
        } else {
          lastSeen = detected;        // new zone — restart streak
          consecutiveCount = 1;
        }
        if (consecutiveCount >= HYSTERESIS) {   // ← CHANGED: was >= 1, now actually uses HYSTERESIS
          decidedZone = detected;
          setLedForZone(decidedZone);
        }
      } else {
        // ── UNKNOWN frame: count misses, revert to white after enough of them ──  // ← ADD block
        missCount++;
        consecutiveCount = 0;   // a miss breaks any in-progress color streak
        lastSeen = UNKNOWN;
        if (missCount >= UNKNOWN_HYST) {
          decidedZone = UNKNOWN;
          setLedForZone(UNKNOWN);   // ← white LED
        }
      }
    }

    // ── Switch debounce ─────────────────────────────────────
    if (isStartSwitchOn()) {
      stableHighCount++;
      if (stableHighCount >= SWITCH_DEBOUNCE_READS) {
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
  if (!found) { zone_ema_init = false; return UNKNOWN; }

  if (!zone_ema_init) {
    zone_ema_cx = bestCx;
    zone_ema_cy = bestCy;
    zone_ema_init = true;
  } else {
    if (abs(bestCx - zone_ema_cx) > ZONE_OUTLIER_JUMP ||
        abs(bestCy - zone_ema_cy) > ZONE_OUTLIER_JUMP) {
      return UNKNOWN;  // outlier — discard this frame
    }
    zone_ema_cx = ZONE_EMA_ALPHA * bestCx + (1.0f - ZONE_EMA_ALPHA) * zone_ema_cx;
    zone_ema_cy = ZONE_EMA_ALPHA * bestCy + (1.0f - ZONE_EMA_ALPHA) * zone_ema_cy;
  }
  return classifyZone((int)zone_ema_cx, (int)zone_ema_cy);
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

struct ZoneCentroid { Zone z; int cx; int cy; };

Zone classifyZone(int cx, int cy) {
  // Centroids from actual Olivia camera readings
  static const ZoneCentroid ZONE_CENTROIDS[4] = {
    { TOP_LEFT,  160,  81 },
    { BOT_LEFT,  155, 101 },
    { TOP_RIGHT, 242,  86 },
    { BOT_RIGHT, 263, 104 },
  };
  float bestDist = 1e9;
  Zone  bestZone = UNKNOWN;
  for (int i = 0; i < 4; i++) {
    float dx = cx - ZONE_CENTROIDS[i].cx;
    float dy = cy - ZONE_CENTROIDS[i].cy;
    float dist = dx*dx + dy*dy;
    if (dist < bestDist) { bestDist = dist; bestZone = ZONE_CENTROIDS[i].z; }
  }
  const float MAX_DIST_SQ = 40.0f * 40.0f;  // reject if >40px from any centroid
  if (bestDist > MAX_DIST_SQ) return UNKNOWN;
  return bestZone;
}

// ============================================================
//  TRAJECTORIES
//  T1 & T2 → end with Pixy slow-crawl to Side1 wall (0°)
//  T3 & T4 → end with TOF slow-crawl to Side3 wall (180°)
//  NOTE: these still use the ORIGINAL executeDrive / executeDriveUntilPixy
//  functions (FWD_BASE_SPEED / SIG3_FWD_SPEED) — unaffected by RECT_SPEED.
// ============================================================

void runTrajectory1() {
  Serial.println("=== TRAJECTORY 1 START ===");
  intakeOn();
  servoWrite(SERVO_LOAD); delay(500);
  executeDriveToEncoder(T1_ENC_PHASE1, 0.0, T1_SPEED);
  servoWrite(SERVO_NEUTRAL); delay(500);
  executeDriveToEncoder(T1_ENC_PHASE2, 0.0, T1_SPEED);
  motorsStop(); delay(T1_PAUSE_MS);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop(); intakeOff(); delay(200);
  // ← Pixy crawl REMOVED — rectangle turn1 takes over immediately
  Serial.println("=== TRAJECTORY 1 DONE ===");
}

void runTrajectory2() {
  Serial.println("=== TRAJECTORY 2 START ===");
  intakeOn();
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDriveToEncoder(T2_ENC_PHASE1, 0.0, T2_SPEED);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_LOAD);  delay(250);
  executeDriveToEncoder(T2_ENC_PHASE2, 0.0, T2_SPEED);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop();
  // ← Pixy crawl REMOVED
  Serial.println("=== TRAJECTORY 2 DONE ===");
}

void runTrajectory3() {
  Serial.println("=== TRAJECTORY 3 START ===");
  executeTurn(30.0); waitMs(pause_ms);
  intakeOn();
  servoWrite(SERVO_LOAD); delay(500);
  executeDriveToEncoder(T3_ENC_PHASE1, 30.0, T3_SPEED);
  servoWrite(SERVO_NEUTRAL); delay(500);
  executeDriveToEncoder(T3_ENC_PHASE2, 0.0, T3_SPEED);
  motorsStop(); delay(T3_PAUSE_MS);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop(); intakeOff(); delay(200);
  // ← Pixy crawl REMOVED
  Serial.println("=== TRAJECTORY 3 DONE ===");
}

void runTrajectory4() {
  Serial.println("=== TRAJECTORY 4 START ===");
  executeTurn(30.0); waitMs(pause_ms);
  intakeOn();
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDriveToEncoder(T4_ENC_PHASE1, 30.0, T4_SPEED);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_LOAD);  delay(250);
  executeDriveToEncoder(T4_ENC_PHASE2, 0.0, T4_SPEED);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop();
  // ← Pixy crawl REMOVED
  Serial.println("=== TRAJECTORY 4 DONE ===");
}


void runForwardPIDSpeed(float targetHeading, float &prevErr, float &integ, const int speed) {
  float heading = getHeading();
  float error   = shortestError(targetHeading, heading);
  float dt      = FWD_INTERVAL / 1000.0;

  if (abs(error) <= FWD_DEADBAND) {
    integ   = 0.0;
    prevErr = error;
    driveMotors(speed, speed);
    Serial.print("FWD STRAIGHT | H:"); Serial.println(heading, 1);
    return;
  }

  integ += error * dt;
  integ  = constrain(integ, -50, 50);

  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float correction = (FWD_KP * error) + (FWD_KI * integ) + (FWD_KD * derivative);

  // ── Scale max correction to the current base speed ──
  // At full speed (220) this still allows up to FWD_MAX_CORRECT (60).
  // At a slow speed (e.g. 60) it caps correction at a fraction of that
  // speed instead of overpowering it, preventing bang-bang oscillation.
  int scaledMaxCorrect = constrain((int)(FWD_MAX_CORRECT * ((float)speed / FWD_BASE_SPEED)), 15, FWD_MAX_CORRECT);
  correction = constrain(correction, -scaledMaxCorrect, scaledMaxCorrect);

  int leftSpeed  = constrain(speed + (int)correction, 0, 255);
  int rightSpeed = constrain(speed - (int)correction, 0, 255);

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("FWD | H:"); Serial.print(heading, 1);
  Serial.print(" E:"); Serial.print(error, 1);
  Serial.print(" Corr:"); Serial.print(correction, 1);
  Serial.print(" L:"); Serial.print(leftSpeed);
  Serial.print(" R:"); Serial.println(rightSpeed);
}
void runRectangleLap3() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  executeDriveBackward(250, 0.0);   // drive backward for 700ms, holding heading 0.0
  waitMs(pause_ms);

  executeTurn(-90.0);

  Serial.println("=== RECT: Side 2 @ -90° (time) ===");
  executeDriveRect(breadth_pause, -90.0);
  waitMs(pause_ms);

  // intakeSlowed();
  // waitMs(pause_ms);
  // servoWrite(SERVO_SHOOT);
  // waitMs(350);
  // servoWrite(SERVO_NEUTRAL);
  // intakeOn();

  Serial.println("=== RECT: Turn 2 → -135° ===");
  executeTurn(-145.0);
  waitMs(pause_ms);

  // SIDE 3 — encoder, no shoot (matches runRectangleLap exact)
  Serial.println("=== RECT: Side 3 @ 180° (encoder) ===");
  resetEncoder();
  executeDriveUntilEncRect(180.0, ENC_END_SIDE3_SLOW_COUNT, ENC_END_SIDE3_STOP_COUNT, FWD_BASE_SPEED, RECT_SPEED);
  waitMs(pause_ms);

  executeTurn(-90);
  waitMs(pause_ms);
  executeDriveRect(800, -90.0);
  waitMs(pause_ms);
  executeTurn(0);

  servoWrite(SERVO_NEUTRAL);
  waitMs(pause_ms);


  unsigned long startTime = millis();

  float prevErr = 0.0;
  float integ   = 0.0;

  unsigned long lastPIDTime = millis();

  const float TARGET_HEADING = 0.0;

  bool shootTriggered = false;

  while (!matchTimeUp()) {

    unsigned long now = millis();
    unsigned long elapsed = now - startTime;

    // ----------------------------------------------------------
    // FIRST 550 ms
    // Drive forward at SPEED 220
    // Servo remains neutral
    // ----------------------------------------------------------
    if (elapsed < 250) {

      if (now - lastPIDTime >= FWD_INTERVAL) {

        lastPIDTime = now;

        runForwardPIDSpeed(
          TARGET_HEADING,
          prevErr,
          integ,
          220
        );
      }
    }


      else if (elapsed < 650) {

      if (now - lastPIDTime >= FWD_INTERVAL) {

        lastPIDTime = now;

        runForwardPIDSpeed(
          20,
          prevErr,
          integ,
          220
        );
      }
    }

    // ----------------------------------------------------------
    // AFTER 550 ms
    // Servo -> SHOOT
    // Drive continuously at SPEED 60
    // Servo stays SHOOT until match ends
    // ----------------------------------------------------------
    else {

      // Move servo to shooting position ONCE
      if (!shootTriggered) {

        servoWrite(SERVO_SHOOT);
        shootTriggered = true;

        Serial.println("=== 550 ms REACHED ===");
        Serial.println("=== SERVO -> SHOOT ===");
        Serial.println("=== SPEED -> 60 ===");
      }

      // Continue driving forward at 60
      if (now - lastPIDTime >= FWD_INTERVAL) {

        lastPIDTime = now;

        runForwardPIDSpeed(
          20,
          prevErr,
          integ,
          70
        );
      }
    }

    // Intake NEVER stops
    intakeOn();
  }

  Serial.println("=== RECT3 COMPLETE — 50 SECOND MATCH ===");


}


void runRectangleLap() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  // SIDE 1 — Pixy sig3, shoot midway, full-speed phase ON
Serial.println("=== RECT: Side 1 @ 0° (encoder) ===");
resetEncoder();
executeDriveUntilEncRect(5.0, ENC_SIDE1_SLOW_COUNT, ENC_SIDE1_STOP_COUNT, FWD_BASE_SPEED, RECT_SPEED, false, true);
waitMs(pause_ms);
  //   // ★★★ ADD — stop exactly here if 25s has passed ★★★
  // if (phase1TimeUp()) {
  //   Serial.println("=== PHASE1 25s HIT — stopping at end of Side 3 ===");
  //   motorsStop();
  //   intakeOff();
  //   servoWrite(SERVO_NEUTRAL);
  //   return true;   // signal caller: stopped early, don't run another lap
  // }


  Serial.println("=== RECT: Turn 1 → -45° ===");
  executeTurn(-70.0, false);
  intakeOn();
  servoWrite(SERVO_NEUTRAL);
  lapCounter++;
  if (lapCounter >= 2) {
    driftCorrection += 1.2;
    lapCounter = 0;
    Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
  }
  waitMs(pause_ms);

Serial.println("=== RECT: Side 2 @ -90° (encoder) ===");
{
  long slowCount, stopCount;
  int side2Speed;
  if (firstLap || slowSide2NextLap) {
    slowCount  = ENC_SIDE2_LAP3_SLOW_COUNT;
    stopCount  = ENC_SIDE2_LAP3_STOP_COUNT;
    side2Speed = SIDE2_LAP3_SPEED;
    if (firstLap) firstLap = false;
    if (slowSide2NextLap) slowSide2NextLap = false;
  } else {
    slowCount  = ENC_SIDE2_SLOW_COUNT;
    stopCount  = ENC_SIDE2_STOP_COUNT;
    side2Speed = RECT_SPEED_2;
  }
  resetEncoder();
  executeDriveUntilEncRect(-80.0, slowCount, stopCount, FWD_BASE_SPEED, side2Speed);
}
waitMs(pause_ms);

  // intakeSlowed();
  // waitMs(pause_ms);
  // servoWrite(SERVO_SHOOT);
  // waitMs(350);
  // servoWrite(SERVO_NEUTRAL);
  // intakeOn();

  Serial.println("=== RECT: Turn 2 → -135° ===");
  executeTurn(-145.0);
  waitMs(pause_ms);

  // SIDE 3 — TOF, no shoot
  Serial.println("=== RECT: Side 3 @ 180° (TOF) ===");
  resetEncoder();
executeDriveUntilEncRect(180.0, ENC_SIDE3_SLOW_COUNT, ENC_SIDE3_STOP_COUNT, FWD_BASE_SPEED, RECT_SPEED);
  waitMs(pause_ms);


  Serial.println("=== RECT: Turn 3 → 135° ===");
  executeTurn(130.0);
  waitMs(pause_ms);

lap3Counter++;
long side4SlowCount = ENC_SIDE4_SLOW_COUNT;
long side4StopCount = ENC_SIDE4_STOP_COUNT;
if (lap3Counter >= 3) {
  side4SlowCount = 60;
  side4StopCount = 80;
  lap3Counter    = 0;
  slowSide2NextLap  = true;
  // lap3Side1NextCall = true;
  Serial.println("=== LAP 3 SPECIAL: Side 4 shortened ===");
}

Serial.println("=== RECT: Side 4 @ 90° (encoder) ===");
resetEncoder();
bool doReverseIntake = (lap3Counter != 0);  // lap3Counter was just reset to 0 on lap3
executeDriveUntilEncRect(100.0, side4SlowCount, side4StopCount, FWD_BASE_SPEED, RECT_SPEED_2, doReverseIntake);
waitMs(pause_ms);

  Serial.println("=== RECT: Turn 4 → 45° ===");
  executeTurn(30.0);
  waitMs(pause_ms);

  Serial.println("=== RECT: Lap complete ===");
    // return false;   // ★ ADD — normal completion, keep looping in Phase 1
  
}
// ============================================================
//  runRectangleLap()
//  Full lap: Side1(Pixy) → Turn1 → Side2 → Turn2 →
//            Side3(TOF)  → Turn3 → Side4 → Turn4
//  NOTE: all forward-drive calls here use the *Rect variants
//  (RECT_SPEED = 140). Turns are unaffected (PID-driven).
// ============================================================
// void runRectangleLap() {
//   intakeOn();
//   servoWrite(SERVO_NEUTRAL);

//   // SIDE 1 — Pixy sig3, shoot midway, full-speed phase ON
//   Serial.println("=== RECT: Side 1 @ 0° (Pixy stop, shoot midway) ===");
//   ema_init = false;
//   executeDriveUntilPixyRect(10.0, true, true);
//   waitMs(pause_ms);


//   Serial.println("=== RECT: Turn 1 → -45° ===");
//   executeTurn(-75.0, true);        // ← shoot fires as turn start;
//   intakeOn();              // resume forward

//   servoWrite(SERVO_NEUTRAL);   // ← release shoot after Turn 1
// lapCounter++;
// if (lapCounter >= 2) {
//   driftCorrection += 1.2;
//   lapCounter = 0;
//   Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
// }

//   waitMs(pause_ms);

//   Serial.println("=== RECT: Side 2 @ -90° (time) ===");
//   executeDriveRect(breadth_pause, -85.0);
//   waitMs(pause_ms);

//   Serial.println("=== RECT: Turn 2 → -135° ===");
//   executeTurn(-145.0);
//   waitMs(pause_ms);

//   // SIDE 3 — TOF, no shoot
//   Serial.println("=== RECT: Side 3 @ 180° (TOF) ===");
// executeDriveUntilCloseRect(180.0, STOP_DISTANCE_CM, false, true);  waitMs(pause_ms);

//   Serial.println("=== RECT: Turn 3 → 135° ===");
//   executeTurn(135.0);
//   waitMs(pause_ms);

//     lap3Counter++;                          // ← increment here
//   int side4Duration = breadth_pause;      // default
//   if (lap3Counter >= 3) {
//     side4Duration = 400;                  // shorter on every 3rd lap
//     lap3Counter = 0;                      // reset counter
//     Serial.println("=== LAP 3 SPECIAL: Side 4 shortened ===");
//   }

//   Serial.println("=== RECT: Side 4 @ 90° (time) ===");
//   executeDriveRect(side4Duration, 100.0);
//   waitMs(pause_ms);

//   Serial.println("=== RECT: Turn 4 → 45° ===");
//   executeTurn(30.0);
//   waitMs(pause_ms);

//   Serial.println("=== RECT: Lap complete ===");
// }

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
  executeTurn(-70.0, false);        // ← shoot fires as turn start;
  intakeOn();              // resume forward

  servoWrite(SERVO_NEUTRAL);   // ← add this
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.2;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);

// Serial.println("=== RECT(T1): Side 2 @ -90° (encoder) ===");
// {
//   long savedSide2Slow = ENC_SIDE2_SLOW_COUNT;
//   bool side2Slow = (firstLap || slowSide2NextLap);
//   if (side2Slow) ENC_SIDE2_SLOW_COUNT = 0;
//   int side2Speed = side2Slow ? 100 : RECT_SPEED_2;
//   if (firstLap) firstLap = false;
//   if (slowSide2NextLap) slowSide2NextLap = false;
//   resetEncoder();
//   executeDriveUntilEncRect(-85.0, ENC_SIDE2_SLOW_COUNT, ENC_SIDE2_STOP_COUNT, FWD_BASE_SPEED, side2Speed);
//   ENC_SIDE2_SLOW_COUNT = savedSide2Slow;
// }



Serial.println("=== RECT: Side 2 @ -90° (encoder) ===");
{
  long slowCount, stopCount;
  int side2Speed;
  if (firstLap || slowSide2NextLap) {
    slowCount  = ENC_SIDE2_LAP3_SLOW_COUNT;
    stopCount  = ENC_SIDE2_LAP3_STOP_COUNT;
    side2Speed = SIDE2_LAP3_SPEED;
    if (firstLap) firstLap = false;
    if (slowSide2NextLap) slowSide2NextLap = false;
  } else {
    slowCount  = ENC_SIDE2_SLOW_COUNT;
    stopCount  = ENC_SIDE2_STOP_COUNT;
    side2Speed = RECT_SPEED_2;
  }
  resetEncoder();
  executeDriveUntilEncRect(-80.0, slowCount, stopCount, FWD_BASE_SPEED, side2Speed);
}
waitMs(pause_ms);

  Serial.println("=== RECT(T1): Turn 2 → -135° ===");
  executeTurn(-145.0);
  waitMs(pause_ms);

  // SIDE 3 — TOF, no shoot
  Serial.println("=== RECT(T1): Side 3 @ 180° (TOF) ===");
resetEncoder();
executeDriveUntilEncRect(180.0, ENC_SIDE3_SLOW_COUNT, ENC_SIDE3_STOP_COUNT, FWD_BASE_SPEED, RECT_SPEED);
 waitMs(pause_ms);

  Serial.println("=== RECT(T1): Turn 3 → 135° ===");
  executeTurn(130.0);
  waitMs(pause_ms);
lap3Counter++;
long side4SlowCount = ENC_SIDE4_SLOW_COUNT;
long side4StopCount = ENC_SIDE4_STOP_COUNT;
if (lap3Counter >= 3) {
  side4SlowCount = 60;
  side4StopCount = 80;
  lap3Counter    = 0;
  slowSide2NextLap  = true;
  // lap3Side1NextCall = true;
  Serial.println("=== LAP 3 SPECIAL: Side 4 shortened ===");
}

Serial.println("=== RECT(T1): Side 4 @ 90° (encoder) ===");
resetEncoder();
bool doReverseIntake = (lap3Counter != 0);  // lap3Counter was just reset to 0 on lap3
executeDriveUntilEncRect(95.0, side4SlowCount, side4StopCount, FWD_BASE_SPEED, RECT_SPEED_2, doReverseIntake);
waitMs(pause_ms);

  Serial.println("=== RECT(T1): Turn 4 → 45° ===");
  executeTurn(30.0);
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
  executeTurn(130.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Side 4 @ 90° (time) ===");
executeDriveRect(breadth_pause, 95.0, true);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Turn 4 → 45° ===");
  executeTurn(30.0);
  waitMs(pause_ms);

  // SIDE 1 — Pixy sig3, shoot midway, full-speed phase ON
  Serial.println("=== RECT(T3): Side 1 @ 0° (Pixy stop, shoot midway) ===");
resetEncoder();
executeDriveUntilEncRect(0.0, ENC_SIDE1_SLOW_COUNT, ENC_SIDE1_STOP_COUNT, FWD_BASE_SPEED, RECT_SPEED, false, true);
  waitMs(pause_ms);

  Serial.println("=== RECT(T3): Turn 1 → -45° ===");
  executeTurn(-70.0, true);        // ← shoot fires as turn start;
  intakeOn();              // resume forward

  servoWrite(SERVO_NEUTRAL);   // ← release shoot after Turn 1
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.2;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);

Serial.println("=== RECT(T3): Side 2 @ -90° (encoder) ===");
{
  long slowCount = slowSide2NextLap ? ENC_SIDE2_LAP3_SLOW_COUNT : ENC_SIDE2_SLOW_COUNT;
  long stopCount = slowSide2NextLap ? ENC_SIDE2_LAP3_STOP_COUNT : ENC_SIDE2_STOP_COUNT;
  int side2Speed = slowSide2NextLap ? SIDE2_LAP3_SPEED : RECT_SPEED_2;
  if (slowSide2NextLap) slowSide2NextLap = false;
  resetEncoder();
  executeDriveUntilEncRect(-80.0, slowCount, stopCount, FWD_BASE_SPEED, side2Speed);
}
waitMs(pause_ms);

  Serial.println("=== RECT(T3): Turn 2 → -135° ===");
  executeTurn(-145.0);
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
resetEncoder();
executeDriveUntilEncRect(180.0, ENC_SIDE3_SLOW_COUNT, ENC_SIDE3_STOP_COUNT, FWD_BASE_SPEED, RECT_SPEED);
waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 3 → 135° ===");
  executeTurn(130.0);

  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Side 4 @ 90° (time) ===");
  executeDriveRect(breadth_pause, 95.0, true);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 4 → 45° ===");
  executeTurn(30.0);
  waitMs(pause_ms);

  // SIDE 1 — Pixy sig3, shoot midway, full-speed phase ON
  Serial.println("=== RECT(S3): Side 1 @ 0° (Pixy stop, shoot midway) ===");
resetEncoder();
executeDriveUntilEncRect(5.0, ENC_SIDE1_SLOW_COUNT, ENC_SIDE1_STOP_COUNT, FWD_BASE_SPEED, RECT_SPEED, false, true);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 1 → -45° ===");
  executeTurn(-70.0, true);        // ← shoot fires as turn start;
  intakeOn();              // resume forward

  servoWrite(SERVO_NEUTRAL);   // ← release shoot after Turn 1
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.2;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);

Serial.println("=== RECT(S3): Side 2 @ -90° (encoder) ===");
{
  long slowCount = slowSide2NextLap ? ENC_SIDE2_LAP3_SLOW_COUNT : ENC_SIDE2_SLOW_COUNT;
  long stopCount = slowSide2NextLap ? ENC_SIDE2_LAP3_STOP_COUNT : ENC_SIDE2_STOP_COUNT;
  int side2Speed = slowSide2NextLap ? SIDE2_LAP3_SPEED : RECT_SPEED_2;
  if (slowSide2NextLap) slowSide2NextLap = false;
  resetEncoder();
  executeDriveUntilEncRect(-85.0, slowCount, stopCount, FWD_BASE_SPEED, side2Speed);
}
waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 2 → -135° ===");
  executeTurn(-145.0);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Lap complete ===");
}

// ============================================================
//  executeDriveUntilPixy()  [ORIGINAL — used by runTrajectory1-4]
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
        if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }   // ★ ADD

    unsigned long now     = millis();
    unsigned long elapsed = now - startTime;

    // ── Timeout fallback ───────────────────────────────────
    if (elapsed > SIG3_TIMEOUT_MS) {
      Serial.println("PIXY STOP: TIMEOUT — stopping.");
      break;
    }

    // ── Midway shoot state machine (non-blocking) ──────────
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
//  executeDriveUntilPixyRect()  [RECTANGLE-ONLY — RECT_SPEED = 140]
//  Identical logic to executeDriveUntilPixy(), but the "full speed"
//  phase uses RECT_SPEED instead of FWD_BASE_SPEED. Only called from
//  runRectangleLap() / runRectangleLapFromTurn1() / FromTurn3() / FromSide3().
// ============================================================
void executeDriveUntilPixyRect(float targetHeading, bool shootMidway, bool useFullSpeedPhase) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  while (true) {
        if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }   // ★ ADD

    unsigned long now     = millis();
    unsigned long elapsed = now - startTime;

    // ── Timeout fallback ───────────────────────────────────
    if (elapsed > SIG3_TIMEOUT_MS) {
      Serial.println("PIXY STOP: TIMEOUT — stopping.");
      break;
    }

    // ── Midway shoot state machine (non-blocking) ──────────
// if (shootMidway && !shootTriggered && elapsed >= 500) {
//   Serial.println("Mid-drive SHOOT triggered — holding until Turn 1");
//   servoWrite(SERVO_SHOOT);
//   shootTriggered = true;
// }

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

// ── Speed: full speed → RECT_SPEED crawl → SIG3_FWD_SPEED final approach ──
int currentSpeed;
if (useFullSpeedPhase && elapsed < RECT_FULLSPEED_MS) {
  currentSpeed = FWD_BASE_SPEED;      // 0–550ms: full speed
} else {
  currentSpeed = SIG3_FWD_SPEED;      // after 550ms: existing slow Pixy-approach speed
}
runForwardPID(targetHeading, prevErr, integ, currentSpeed);
    }
  }

  motorsStop();
}

// ============================================================
//  executeDrive()  [ORIGINAL — used by runTrajectory1-4]
// ============================================================
void executeDrive(unsigned long durationMs, float targetHeading) {
  float prevErr = 0.0, integ = 0.0;
  unsigned long startTime = millis(), lastPIDTime = millis();

  while (millis() - startTime < durationMs) {
    if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }   // ★ ADD
    unsigned long now = millis();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPID(targetHeading, prevErr, integ, FWD_BASE_SPEED);
    }
  }
  motorsStop();
}

// ============================================================
//  executeDriveRect()  [RECTANGLE-ONLY — RECT_SPEED = 140]
// ============================================================
void executeDriveRect(unsigned long durationMs, float targetHeading, bool reverseIntakeAtStart) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  bool intakeSwitched = false;
  bool servoShot      = false;

  if (reverseIntakeAtStart) intakeReverseSlowed();

  while (millis() - startTime < durationMs) {
        if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }   // ★ ADD

    unsigned long now = millis();
    unsigned long elapsed = now - startTime;

    if (reverseIntakeAtStart && !servoShot && elapsed >= 50) {
      servoWrite(SERVO_SHOOT);
      servoShot = true;
      Serial.println("Side4: servo SHOOT at 50ms");
    }
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 500) {
      intakeOn();
      servoWrite(SERVO_NEUTRAL);
      intakeSwitched = true;
      Serial.println("Side4: intake ON + servo NEUTRAL at 500ms");
    }

    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPID(targetHeading, prevErr, integ, RECT_SPEED_2);
    }
  }
  motorsStop();
}

// void executeDriveUntilEncRect(float targetHeading, long slowCount, long stopCount, int fastSpeed, int slowSpeed) {
//   float prevErr = 0.0;
//   float integ   = 0.0;
//   unsigned long lastPIDTime = millis();
//   unsigned long startTime   = millis();
//   const unsigned long MAX_TIMEOUT = 4000;

//   while (true) {
//     if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }

//     unsigned long now = millis();
//     if (now - startTime > MAX_TIMEOUT) {
//       Serial.println("ENC: MAX TIMEOUT — stopping.");
//       break;
//     }

//     if (now - lastPIDTime >= FWD_INTERVAL) {
//       lastPIDTime = now;
//       long ticks = abs(encCount);
//       Serial.print("ENC | ticks: "); Serial.println(ticks);

//       if (ticks >= stopCount) {
//         Serial.println("ENC → STOP");
//         break;
//       }

//       int currentSpeed = (ticks < slowCount) ? fastSpeed : slowSpeed;
//       runForwardPID(targetHeading, prevErr, integ, currentSpeed);
//     }
//   }
//   motorsStop();
// }




void executeDriveUntilEncRect(float targetHeading, long slowCount, long stopCount,
                               int fastSpeed, int slowSpeed,
                               bool reverseIntakeAtStart, bool shootMidway) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime = millis();
  unsigned long startTime   = millis();
  const unsigned long MAX_TIMEOUT = 1500;

  bool intakeSwitched = false;
  bool servoShot      = false;

  // LOCAL shoot state — resets every call so each lap fires correctly
  bool shootTriggered     = false;
  bool shootDone          = false;
  unsigned long shootStartTime = 0;
  int  shootPhase         = 0;

  if (reverseIntakeAtStart) intakeReverseSlowed();

  while (true) {
    if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }

    unsigned long now     = millis();
    unsigned long elapsed = now - startTime;

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("ENC: MAX TIMEOUT — stopping.");
      break;
    }

    if (reverseIntakeAtStart && !servoShot && elapsed >= 50) {
      servoWrite(SERVO_SHOOT);
      servoShot = true;
    }
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 500) {
      intakeOn();
      servoWrite(SERVO_NEUTRAL);
      intakeSwitched = true;
    }

    // ── Midway shoot (Side 1) ──────────────────────────────────
    if (shootMidway && !shootDone) {
      if (!shootTriggered && elapsed >= 350) {
        intakeOn();
        servoWrite(SERVO_SHOOT);
        shootTriggered  = true;
        shootStartTime  = now;
        shootPhase      = 1;
        Serial.println("ENC: midway SHOOT triggered");
      }
      if (shootPhase == 1 && (now - shootStartTime >= 900)) {
        servoWrite(SERVO_NEUTRAL);
        intakeOn();
        shootPhase = 2;
        Serial.println("ENC: servo back to NEUTRAL");
      }
      if (shootPhase == 2 && (now - shootStartTime >= 1100)) {
        shootDone = true;
        Serial.println("ENC: midway shoot DONE");
      }
    }

    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      long ticks = abs(encCount);
      Serial.print("ENC | ticks: "); Serial.println(ticks);

      if (ticks >= stopCount) {
        Serial.println("ENC → STOP");
        break;
      }

      int currentSpeed = (ticks < slowCount) ? fastSpeed : slowSpeed;
      runForwardPID(targetHeading, prevErr, integ, currentSpeed);
    }
  }
  motorsStop();
}



// ============================================================
//  executeDriveUntilClose() — Side 3 (TOF)  [ORIGINAL — used by runTrajectory1-4]
// ============================================================
void executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 1500;

  bool intakeSwitched = false;
    bool servoShot       = false;          // ← ADD

  if (reverseIntakeAtStart) intakeReverseSlowed();   // ← only when caller asks
  else                      intakeOn();        // ← normal: keep forward


  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  while (true) {
        if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }   // ★ ADD

    unsigned long now = millis();
    unsigned long elapsed = now - startTime;   // ← ADD

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
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 500) {
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
void runForwardPIDBackward(float targetHeading, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetHeading, heading);
  float dt      = FWD_INTERVAL / 1000.0;

  if (abs(error) <= FWD_DEADBAND) {
    integ   = 0.0;
    prevErr = error;
    driveMotors(-FWD_BASE_SPEED_Slow, -FWD_BASE_SPEED_Slow);
    Serial.print("BWD STRAIGHT | H:"); Serial.println(heading, 1);
    return;
  }

  integ += error * dt;
  integ  = constrain(integ, -50, 50);

  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float correction = (FWD_KP * error) + (FWD_KI * integ) + (FWD_KD * derivative);
  correction = constrain(correction, -FWD_MAX_CORRECT, FWD_MAX_CORRECT);

  // Same correction sign as forward — steering direction relative to
  // heading error is unchanged; only base speed sign flips for reverse.
  int leftSpeed  = constrain(FWD_BASE_SPEED_Slow + (int)correction, 0, 255);
  int rightSpeed = constrain(FWD_BASE_SPEED_Slow - (int)correction, 0, 255);

  driveMotors(-leftSpeed, -rightSpeed);

  Serial.print("BWD | H:"); Serial.print(heading, 1);
  Serial.print(" E:"); Serial.print(error, 1);
  Serial.print(" L:"); Serial.print(-leftSpeed);
  Serial.print(" R:"); Serial.println(-rightSpeed);
}


void executeDriveBackward(unsigned long durationMs, float targetHeading) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  while (millis() - startTime < durationMs) {
    unsigned long now = millis();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPIDBackward(targetHeading, prevErr, integ);
    }
  }
  motorsStop();
}

// ============================================================
//  executeDriveToEncoder()
//  Drive at `speed` until `targetTicks` encoder counts.
//  Resets encoder internally — caller never needs resetEncoder().
// ============================================================
void executeDriveToEncoder(long targetTicks, float targetHeading, int speed) {
  resetEncoder();   // ← internal reset, no manual reset needed between phases
  float prevErr = 0.0, integ = 0.0;
  unsigned long lastPIDTime = millis();
  unsigned long startTime   = millis();
  const unsigned long MAX_TIMEOUT = 5000;

  while (true) {
    if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }
    if (millis() - startTime > MAX_TIMEOUT) {
      Serial.println("DTE: TIMEOUT — stopping.");
      break;
    }
    unsigned long now = millis();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      long ticks = abs(encCount);
      Serial.print("DTE | "); Serial.print(ticks);
      Serial.print("/"); Serial.println(targetTicks);
      if (ticks >= targetTicks) { Serial.println("DTE → STOP"); break; }
      runForwardPID(targetHeading, prevErr, integ, speed);
    }
  }
  motorsStop();
}
// ============================================================
//  executeDriveUntilEnc()  — used by runTrajectory1-4 only
//  Single-speed encoder drive (no slow phase — trajectory dashes are short)
// ============================================================
void executeDriveUntilEnc(float targetHeading, long stopCount, int speed) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime = millis();
  unsigned long startTime   = millis();
  const unsigned long MAX_TIMEOUT = 5000;

  while (true) {
    if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }
    unsigned long now = millis();
    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("ENC_T: MAX TIMEOUT — stopping.");
      break;
    }
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      long ticks = abs(encCount);
      Serial.print("ENC_T | ticks: "); Serial.println(ticks);
      if (ticks >= stopCount) { Serial.println("ENC_T → STOP"); break; }
      runForwardPID(targetHeading, prevErr, integ, speed);
    }
  }
  motorsStop();
}
// ============================================================
//  executeDriveUntilCloseRect() — Side 3 (TOF)  [RECTANGLE-ONLY — RECT_SPEED = 140]
// ============================================================
void executeDriveUntilCloseRect(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 1500;

  bool intakeSwitched = false;
    bool servoShot       = false;

  if (reverseIntakeAtStart) intakeReverseSlowed();
  else                      intakeOn();


  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  while (true) {
        if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }   // ★ ADD

    unsigned long now = millis();
    unsigned long elapsed = now - startTime;

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("TOF: MAX TIMEOUT — stopping.");
      break;
    }

        // ── Side3 servo shoot at 50ms ──────────────────────────
    if (reverseIntakeAtStart && !servoShot && elapsed >= 50) {
      servoWrite(SERVO_SHOOT);
      servoShot = true;
      Serial.println("Side3: servo SHOOT at 50ms");
    }

    // ── Side3 intake forward + servo neutral at 1000ms ─────
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 500) {
      intakeOn();
      servoWrite(SERVO_NEUTRAL);
      intakeSwitched = true;
      Serial.println("Side3: intake ON + servo NEUTRAL at 1000ms");
    }

    if (shootMidway && !shootDone) {
      if (!shootTriggered && (now - startTime >= 500)) {
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
      // ── Two-phase speed: full speed first, then slow crawl ──
      int currentSpeed = (elapsed < RECT_FULLSPEED_MS) ? FWD_BASE_SPEED_Slow : RECT_SPEED;
      runForwardPID(targetHeading, prevErr, integ, currentSpeed);
    }
  }
  motorsStop();
}

// ============================================================
//  executeTurn()
// ============================================================
void executeTurn(float targetAngle, bool shootDuringTurn) {
  float prevErr     = 0.0;
  float integ       = 0.0;
  int   stableCount = 0;
  unsigned long lastPIDTime = millis();

  if (shootDuringTurn) {
    intakeReverse();
    Serial.println("SHOOT triggered during Turn 1");
  }

  while (true) {
    if (matchTimeUp()) { motorsStop(); intakeOff(); servoWrite(SERVO_SHOOT); return; }

    unsigned long now = millis();
    if (now - lastPIDTime >= TRN_INTERVAL) {
      lastPIDTime = now;

      float heading = getHeading();
      float error   = shortestError(targetAngle, heading);

      if (abs(error) <= TRN_DEADBAND) {
        motorsStop();
        integ = 0.0; prevErr = error;
        stableCount++;
        Serial.print("Stable "); Serial.print(stableCount);
        Serial.print("/"); Serial.print(TRN_STABLE_COUNT);
        Serial.print(" | H:"); Serial.print(heading, 1);
        Serial.print(" E:"); Serial.println(error, 1);
        Serial.print("Turn done. H:"); Serial.println(heading, 1);
        return;   // ← exit on FIRST stable frame, exactly like Shawn
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
    Serial.println("VL53L0X not found! Continuing without TOF.");
    return;
  }
  Serial.println("VL53L0X Ready.");
}

float getDistanceCm() {
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
}

void intakeOn()  { analogWrite(INTAKE_IN1, 0); analogWrite(INTAKE_IN2, INTAKE_SPEED); }

void intakeSlowed()  { analogWrite(INTAKE_IN1, 0); analogWrite(INTAKE_IN2, 160); }
void intakeReverse()  { analogWrite(INTAKE_IN1, INTAKE_SPEED); analogWrite(INTAKE_IN2, 0); }
void intakeReverseSlowed()  { analogWrite(INTAKE_IN1, SLOW); analogWrite(INTAKE_IN2, 0); }
void intakeOff() { analogWrite(INTAKE_IN1, 0); analogWrite(INTAKE_IN2, 0); }



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
void waitMs(int ms)               { motorsStop(); delay(ms); }