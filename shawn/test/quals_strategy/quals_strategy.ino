// =========================SHAWN===================================
//  FULL RUN — ESP32-S3 + BNO085 + VL53L0X + Pixy2 + MDD 3A
//
//  BOOT FLOW:
//    1. Init all hardware
//    2. Sample Pixy2 for PIXY_SAMPLE_MS (1500ms)
//    3. Pick zone by most-seen cumulative vote:
//       BOT-LEFT  → runTrajectory1()  then rectangle from Side 1
//       TOP-LEFT  → runTrajectory2()  then rectangle from Side 1
//       BOT-RIGHT → runTrajectory3()  then rectangle from Side 3
//       TOP-RIGHT → runTrajectory4()  then rectangle from Side 3
//    4. Rectangle loop forever
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>
#include <Pixy2SPI_SS.h>

#include <FastLED.h>
#define LED_PIN   37
#define NUM_LEDS  1
CRGB leds[NUM_LEDS];
// Zone decidedZone = UNKNOWN;
float driftCorrection = 0.0;  // subtracted from heading every 2 laps
float offset1 = 15;
float offset2 = 5;
int lap3Counter = 0;   // counts laps; every 3rd lap uses shorter breadth_pause for Side 4
int   lapCounter     = 0;    // counts laps for heading drift correction
const int SLOW = 140;          // ← ADD

const int SLOWED = 55;
// bool lapCounterResetForPhase2 = false; // global flag, declared once

// ── EMA smoothing state for PURPLE ZONE cx/cy only ── ADD THESE 5 LINES
float ema_cx = -1.0f;
float ema_cy = -1.0f;
bool  ema_zone_init = false;
#define ZONE_EMA_ALPHA      0.5f
#define ZONE_OUTLIER_JUMP   40
const float SHOOT_HEADING_DEG = 30.0;

// ── Match timer ──────────────────────────────────────────
const unsigned long MATCH_DURATION_MS = 121000; // 120 seconds = 2 minutes
unsigned long matchStartTime = 0;                // set once the switch fires

// unsigned long rectPhaseStartTime = 0;
// const unsigned long PHASE1_DURATION_MS = 15000; // 15 seconds


unsigned long rectPhaseStartTime = 0;
const unsigned long PHASE1_DURATION_MS = 88000; // 15 seconds — phase 1 ends here
const unsigned long PHASE2_DURATION_MS = 108000; // was 35000 — phase 2 ends here
const unsigned long PHASE3_START_MS    = 117000; // ← ADD — phase 3 doesn't begin until here

bool lap3CounterResetForPhase2 = false; // was lapCounterResetForPhase2
// --- Referee start toggle switch ---
#define SWITCH_PWR              48
#define SWITCH_SIG              47
#define SWITCH_DEBOUNCE_READS   5
#define SWITCH_POLL_MS          10
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

// --- Trajectory 3 (Bot-Right: 60° t    urn + T1 logic + realign) ---
const unsigned long T3_PHASE1_MS = 480;
const unsigned long T3_PHASE2_MS = 300;
const unsigned long T3_PHASE3_MS = 200;
const unsigned long T3_PAUSE_MS  = 200;
const int           T3_SPEED     = 180;

// --- Trajectory 4 (Top-Right: 60° turn + T2 logic + realign) ---
const unsigned long T4_PHASE1_MS = 400;
const unsigned long T4_PHASE2_MS = 400;
const int           T4_SPEED     = 180;

// --- Rectangle ---
const float STOP_DISTANCE_CM = 14.0;
const float STOP_DISTANCE_CM_2 = 6.0;
int         breadth_pause    = 1300;
int         pause_ms         = 50;

// --- Pixy2 zone detection ---
// --- Pixy2 zone detection (SHAWN — Pixy 2.1) ---
#define SIG_PURPLE        2
#define MIN_AREA          100       // was 200 — Shawn classifier uses 100
#define ROI_TOP_Y         40        // was 55
// #define SPLIT_X           200       // was 210
// #define SPLIT_Y           77        // was 81
// #define DEAD_X            10        // was 5
// #define DEAD_Y            3         // was 5
#define PIXY_SAMPLE_MS    1500


  // #define SPLIT_X   200 //value changed - was too much to the left earlier
  // #define SPLIT_Y   90  //value changed - was too much to the bottom earlier, verify and mak e up a bit
  // #define DEAD_X    15
  // #define DEAD_Y    3

#define SPLIT_X    195    // midpoint of 147 and 231
#define SPLIT_Y     91    // midpoint of 80 and 102
#define DEAD_X      20    // left cx≈147, right cx≈231, plenty of room
#define DEAD_Y       4    // gap is only 22px, can't afford large dead zone


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

#define TOF_SDA 9
#define TOF_SCL 46

const int INTAKE_IN1   = 38;
const int INTAKE_IN2   = 39;
const int INTAKE_SPEED = 255;

#define SERVO_PIN 40
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

const int SERVO_NEUTRAL = 20;
const int SERVO_SHOOT   = 65;
const int SERVO_LOAD    = 140;

// ============================================================
//  PID CONSTANTS
// ============================================================

const float FWD_KP          = 2.6;
const float FWD_KI          = 0.0;
const float FWD_KD          = 0.5;
const int   FWD_BASE_SPEED  = 220;   // used ONLY by runTrajectory1-4 (via executeDrive/executeDriveUntilClose)
const int   FWD_BASE_SPEED_2  = 60; // RECTANGLE-ONLY slow speed — used for Side2/Side4, and the
const int   FWD_BASE_SPEED_4  = 140; // RECTANGLE-ONLY slow speed — used for Side2/Side4, and the

const int   FWD_BASE_SPEED_3  = 80; // RECTANGLE-ONLY slow speed — used for Side2/Side4, and the

                                      // second (crawl) phase of Side1/Side3 after RECT_FULLSPEED_MS
const unsigned long RECT_FULLSPEED_MS = 800; // Side1/Side3: run at FWD_BASE_SPEED for this long,
                                              // then drop to FWD_BASE_SPEED_2 until the TOF stops it
const int   FWD_MAX_CORRECT = 60;
const float FWD_DEADBAND    = 1.5;
const int   FWD_INTERVAL    = 20;

const float TRN_KP           = 6.0;
const float TRN_KI           = 0.0;
const float TRN_KD           = 0.5;
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

Zone decidedZone = UNKNOWN;   // ← add here

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================

Zone  detectPurpleBall();
Zone  classifyZone(int cx, int cy);

void  runTrajectory1();
void  runTrajectory2();
void  runTrajectory3();
void  runTrajectory4();

// void  runRectangleFromSide1();
void  runRectangleLapFromSide3();
void  runRectangleLap();

void  executeDrive(unsigned long durationMs, float targetHeading);
void  executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway = false, bool reverseIntakeAtStart = false);

// --- Rectangle-lap-only slow variants (FWD_BASE_SPEED_2 = 140) ---
// NOTE: this prototype was previously commented out, which is why the
// definition further down was never being wired into the rectangle calls.
void  executeDriveSpeed(unsigned long durationMs, float targetHeading, const int speed);
void  executeDriveUntilCloseSpeed(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart, const int speed, unsigned long shootDelayMs = 650);
void  runForwardPIDSpeed(float targetHeading, float &prevErr, float &integ, const int speed);

void  executeTurn(float targetAngle);

void  runForwardPID(float targetHeading, float &prevErr, float &integ);
void  runTurnPID(float targetAngle, float &prevErr, float &integ);
void  runRectangleLap3();
void  driveMotors(int leftSpeed, int rightSpeed);
void  turnClockwise(int speed);
void  turnAntiClockwise(int speed);
void  motorsStop();
void  setMotorPins();
void  waitMs(int ms);
void  runRectangleLap2();
void  initServo();
void  servoWrite(int angle);
void  initIntake();
void  intakeOn();
void  intakeOff();
void  intakeReverseSlowed();
void intakestore();
void  initIMU();
void  zeroIMU();
float getRawYaw();
float getHeading();
float shortestError(float target, float current);
float wrapAngle(float a);

void  initTOF();
void  initStartSwitch();
bool  isStartSwitchOn();
void  waitForStartSwitch();
float getDistanceCm();
void  executeDriveBackward(unsigned long durationMs, float targetHeading);
void  runForwardPIDBackward(float targetHeading, float &prevErr, float &integ);

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
  return millis() - matchStartTime >= MATCH_DURATION_MS;
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
  zeroIMU();           // second zero
  decidedZone = UNKNOWN;   // ← sample FIRST
  Serial.print("=== ZONE LOCKED: ");
  Serial.println(decidedZone);

  setLedForZone(UNKNOWN);        // ← LED ON after detection, during wait

  waitForStartSwitch();
  matchStartTime = millis();     // ← ADD: match clock starts the instant the switch fires

  leds[0] = CRGB::Black;             // ← LED OFF when switch fires
  FastLED.show();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

  if (decidedZone  == BOT_LEFT) {
    Serial.println("DECISION: BOT-LEFT → Trajectory 1 → Rect from Side 1");
    runTrajectory1();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 1) ===");
rectPhaseStartTime = millis();
lap3CounterResetForPhase2 = false;
while (!matchTimeUp()) {
  unsigned long elapsedPhase = millis() - rectPhaseStartTime;
  if (elapsedPhase < PHASE1_DURATION_MS) {
    runRectangleLap();                 // or runRectangleLapFromSide3() for the Side-3 branches
  } else if (elapsedPhase < PHASE2_DURATION_MS) {
    if (!lap3CounterResetForPhase2) {
      lap3Counter = 0;
      lap3CounterResetForPhase2 = true;
      Serial.println("=== PHASE 2 START: lap3Counter reset to 0 ===");
    }
    runRectangleLap2();                // or runRectangleLapFromSide3_2()
  } else if (elapsedPhase < PHASE3_START_MS) {
    // ── WAIT ── phase 2's current lap already finished (the while-loop only
    // re-checks time between full function calls), bot holds still until 35s
    motorsStop();
    intakeOff();
    delay(50);
  } else {
    runRectangleLap3();
    // matchEndStop();   // never returns — phase 3 runs exactly once, then bot stops
  }
}
    matchEndStop();

  } else if (decidedZone  == TOP_LEFT) {
    Serial.println("DECISION: TOP-LEFT → Trajectory 2 → Rect from Side 1");
    runTrajectory2();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 1) ===");
    rectPhaseStartTime = millis();
    lap3CounterResetForPhase2 = false;
while (!matchTimeUp()) {
  unsigned long elapsedPhase = millis() - rectPhaseStartTime;
  if (elapsedPhase < PHASE1_DURATION_MS) {
    runRectangleLap();                 // or runRectangleLapFromSide3() for the Side-3 branches
  } else if (elapsedPhase < PHASE2_DURATION_MS) {
    if (!lap3CounterResetForPhase2) {
      lap3Counter = 0;
      lap3CounterResetForPhase2 = true;
      Serial.println("=== PHASE 2 START: lap3Counter reset to 0 ===");
    }
    runRectangleLap2();                // or runRectangleLapFromSide3_2()
  } else if (elapsedPhase < PHASE3_START_MS) {
    // ── WAIT ── phase 2's current lap already finished (the while-loop only
    // re-checks time between full function calls), bot holds still until 35s
    motorsStop();
    intakeOff();
    delay(50);
  } else {
    runRectangleLap3();
    // matchEndStop();   // never returns — phase 3 runs exactly once, then bot stops
  }
}matchEndStop();

  } else if (decidedZone  == BOT_RIGHT) {
    Serial.println("DECISION: BOT-RIGHT → Trajectory 3 → Rect from Side 3");
    runTrajectory3();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 3) ===");
    rectPhaseStartTime = millis();
    lap3CounterResetForPhase2 = false;
while (!matchTimeUp()) {
  unsigned long elapsedPhase = millis() - rectPhaseStartTime;
  if (elapsedPhase < PHASE1_DURATION_MS) {
    runRectangleLap();                 // or runRectangleLapFromSide3() for the Side-3 branches
  } else if (elapsedPhase < PHASE2_DURATION_MS) {
    if (!lap3CounterResetForPhase2) {
      lap3Counter = 0;
      lap3CounterResetForPhase2 = true;
      Serial.println("=== PHASE 2 START: lap3Counter reset to 0 ===");
    }
    runRectangleLap2();                // or runRectangleLapFromSide3_2()
  } else if (elapsedPhase < PHASE3_START_MS) {
    // ── WAIT ── phase 2's current lap already finished (the while-loop only
    // re-checks time between full function calls), bot holds still until 35s
    motorsStop();
    intakeOff();
    delay(50);
  } else {
    runRectangleLap3();
    // matchEndStop();   // never returns — phase 3 runs exactly once, then bot stops
  }

}
matchEndStop();
  } else if (decidedZone  == TOP_RIGHT) {
    Serial.println("DECISION: TOP-RIGHT → Trajectory 4 → Rect from Side 3");
    runTrajectory4();
    Serial.println("=== ENTERING RECTANGLE LOOP (from Side 3) ===");
    rectPhaseStartTime = millis();
    lap3CounterResetForPhase2 = false;
while (!matchTimeUp()) {
  unsigned long elapsedPhase = millis() - rectPhaseStartTime;
  if (elapsedPhase < PHASE1_DURATION_MS) {
    runRectangleLap();                 // or runRectangleLapFromSide3() for the Side-3 branches
  } else if (elapsedPhase < PHASE2_DURATION_MS) {
    if (!lap3CounterResetForPhase2) {
      lap3Counter = 0;
      lap3CounterResetForPhase2 = true;
      Serial.println("=== PHASE 2 START: lap3Counter reset to 0 ===");
    }
    runRectangleLap2();                // or runRectangleLapFromSide3_2()
  } else if (elapsedPhase < PHASE3_START_MS) {
    // ── WAIT ── phase 2's current lap already finished (the while-loop only
    // re-checks time between full function calls), bot holds still until 35s
    motorsStop();
    intakeOff();
    delay(50);
  } else {
    runRectangleLap3();
    // matchEndStop();   // never returns — phase 3 runs exactly once, then bot stops
  }
}
matchEndStop();
  } else {
    Serial.println("DECISION: UNKNOWN → defaulting to Rect from Side 1");
    rectPhaseStartTime = millis();
    lap3CounterResetForPhase2 = false;
while (!matchTimeUp()) {
  unsigned long elapsedPhase = millis() - rectPhaseStartTime;
  if (elapsedPhase < PHASE1_DURATION_MS) {
    runRectangleLap();                 // or runRectangleLapFromSide3() for the Side-3 branches
  } else if (elapsedPhase < PHASE2_DURATION_MS) {
    if (!lap3CounterResetForPhase2) {
      lap3Counter = 0;
      lap3CounterResetForPhase2 = true;
      Serial.println("=== PHASE 2 START: lap3Counter reset to 0 ===");
    }
    runRectangleLap2();                // or runRectangleLapFromSide3_2()
  } else if (elapsedPhase < PHASE3_START_MS) {
    // ── WAIT ── phase 2's current lap already finished (the while-loop only
    // re-checks time between full function calls), bot holds still until 35s
    motorsStop();
    intakeOff();
    delay(50);
  } else {
    runRectangleLap3();
    // matchEndStop();   // never returns — phase 3 runs exactly once, then bot stops
  }
}
matchEndStop();
  }
}
struct ZoneCentroid { Zone z; float cx; float cy; };
const ZoneCentroid ZONE_CENTROIDS[4] = {
  { TOP_LEFT,  147, 60 },
  { BOT_LEFT,  146, 83 },
  { TOP_RIGHT, 240, 66 },
  { BOT_RIGHT, 262, 88 },
};
// ============================================================
//  detectPurpleBall()
//  Samples Pixy2 for PIXY_SAMPLE_MS. Counts votes per zone.
//  Returns zone with highest cumulative vote count.
// ============================================================
// ============================================================
//  classifyZone() — SHAWN tuned thresholds
//  TL: cx~147  cy~60   BL: cx~146  cy~83
//  TR: cx~240  cy~66   BR: cx~262  cy~88
//  SPLIT_X=200  SPLIT_Y=77
// ============================================================
// ✅ FIXED — uses SPLIT_X, SPLIT_Y, DEAD_X, DEAD_Y

Zone classifyZone(int cx, int cy) {
  float bestDist = 1e9;
  Zone  bestZone = UNKNOWN;
  for (int i = 0; i < 4; i++) {
    float dx = cx - ZONE_CENTROIDS[i].cx;
    float dy = cy - ZONE_CENTROIDS[i].cy;
    float dist = dx*dx + dy*dy;  // squared distance, no need for sqrt
    if (dist < bestDist) {
      bestDist = dist;
      bestZone = ZONE_CENTROIDS[i].z;
    }
  }
  // Optional: reject if too far from ANY known centroid (likely garbage/noise)
  const float MAX_DIST_SQ = 60.0f * 60.0f;  // tune this — reject if >60px from nearest centroid
  if (bestDist > MAX_DIST_SQ) return UNKNOWN;

  return bestZone;
}

// ↓↓↓ REPLACE THIS ENTIRE FUNCTION ↓↓↓
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
//  detectPurpleBall() — unchanged logic, updated print
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

      // ── Same debug print as zone_classifier.ino ──────────
      Serial.printf("cx=%-3d  cy=%-3d  isTop=%d  isBot=%d  isLeft=%d  isRight=%d  zone=",
                    bestCx, bestCy,
                    bestCy < (SPLIT_Y - DEAD_Y),
                    bestCy > (SPLIT_Y + DEAD_Y),
                    bestCx < (SPLIT_X - DEAD_X),
                    bestCx > (SPLIT_X + DEAD_X));

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

  // ── Vote summary ──────────────────────────────────────────
  Serial.println("─── PIXY VOTE SUMMARY ───");
  Serial.print("  TOP-LEFT : "); Serial.println(voteTL);
  Serial.print("  BOT-LEFT : "); Serial.println(voteBL);
  Serial.print("  TOP-RIGHT: "); Serial.println(voteTR);
  Serial.print("  BOT-RIGHT: "); Serial.println(voteBR);
  Serial.print("  OTHER    : "); Serial.println(voteOther);
  Serial.print("  MISSED   : "); Serial.println(missed);

  // ── Pick winner ───────────────────────────────────────────
  int best = max({voteTL, voteBL, voteTR, voteBR});
  Serial.println("Best: " + best);
  if (best == 0) {
    Serial.println("  RESULT: No valid detections → UNKNOWN");
    return UNKNOWN;
  }

  if      (voteTL == best) { Serial.println("  RESULT: TOP-LEFT");  return TOP_LEFT;  }
  else if (voteBL == best) { Serial.println("  RESULT: BOT-LEFT");  return BOT_LEFT;  }
  else if (voteTR == best) { Serial.println("  RESULT: TOP-RIGHT"); return TOP_RIGHT; }
  else                     { Serial.println("  RESULT: BOT-RIGHT"); return BOT_RIGHT; }
}


void matchEndStop() {
  motorsStop();
  intakeOff();
  // servoWrite(SERVO_NEUTRAL);
  Serial.println("=== MATCH TIME UP — STOPPING ===");
  while (true) {
    // halt forever; nothing more should run after the match ends
    delay(1000);
  }
}
// ============================================================
//  TRAJECTORY 1 — Bot-Left (Purple → Orange)
//
//  Bot starts facing 0°, ball is bottom-left.
//  1. Servo → LOAD
//  2. Forward phase 1
//  3. Servo → SHOOT, delay
//  4. Forward phase 2, intake ON
//  5. Pause
//  6. Servo → NEUTRAL
//  7. Forward phase 3
//  8. Stop, intake OFF
//  9. Final SHOOT
//  → Hands off to rectangle from Side 1 (heading 0°)
//
//  NOTE: uses the ORIGINAL executeDrive()/executeDriveUntilClose()
//  (FWD_BASE_SPEED = 220) — unaffected by the rectangle slowdown.
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
//  runRectangleLap3()
//  Phase-3 (35s+) trajectory. Follows the same movement as Side 1
//  of runRectangleLap() — drive to the Side1 wall (TOF), then turn
//  to the shoot heading — but the servo is held at SERVO_SHOOT the
//  entire time instead of toggling to NEUTRAL. Called repeatedly by
//  the phase-3 branch in loop(), for every zone (BOT_LEFT, TOP_LEFT,
//  BOT_RIGHT, TOP_RIGHT, UNKNOWN alike) — it's independent of which
//  side the bot started from.
// ============================================================
void runRectangleLap3() {

  Serial.println("========================================");
  Serial.println("=== RECT3: TIME-BASED CONTINUOUS DRIVE ===");
  Serial.println("=== 0-550ms  : SPEED 220              ===");
  Serial.println("=== 550ms-END: SPEED 60 + SERVO SHOOT ===");
  Serial.println("=== NO TOF / NO DISTANCE SENSOR       ===");
  Serial.println("========================================");

  // Intake ON continuously
  intakeOn();

  // Start servo in neutral
  servoWrite(SERVO_NEUTRAL);

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
    if (elapsed < 650) {

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
          TARGET_HEADING,
          prevErr,
          integ,
          60
        );
      }
    }

    // Intake NEVER stops
    intakeOn();
  }

  Serial.println("=== RECT3 COMPLETE — 50 SECOND MATCH ===");
}
// ============================================================
//  TRAJECTORY 2 — Top-Left (Orange → Purple)
//
//  Bot starts facing 0°, ball is top-left.
//  1. Servo → NEUTRAL
//  2. Forward phase 1, intake ON
//  3. Stop
//  4. SHOOT: servo → SHOOT
//  5. Servo → LOAD
//  6. Forward phase 2, intake ON
//  7. Servo → NEUTRAL
//  8. Stop, intake OFF
//  → Hands off to rectangle from Side 1 (heading 0°)
// ============================================================
void runTrajectory2() {
  Serial.println("=== TRAJECTORY 2 START (Top-Left: Orange → Purple) ===");

  intakeOn();

  Serial.println("T2 S1: Servo → NEUTRAL");
  servoWrite(SERVO_SHOOT);
  delay(100);

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
//  TRAJECTORY 3 — Bot-Right (Purple → Orange, right side)
//
//  Ball is bottom-right. Bot must:
//  1. Turn 60° clockwise to face the ball
//  2. Run same intake/shoot logic as Trajectory 1 (at 60° heading)
//  3. Turn to 180° (face Side 3 direction)
//  4. Hand off to rectangle starting from Side 3
// ============================================================
void runTrajectory3() {
  Serial.println("=== TRAJECTORY 3 START (Bot-Right: Purple → Orange) ===");

  // Step 1: Turn 60° right to face the ball
  Serial.println("T3 S1: Turn → 60°");
  executeTurn(30.0);
  waitMs(pause_ms);

  intakeOn();

  // Step 2: Servo → LOAD
  Serial.println("T3 S2: Servo → LOAD");
  servoWrite(SERVO_LOAD);
  delay(500);

  // Step 3: Forward phase 1 (heading locked at 60°)
  Serial.println("T3 S3: Forward phase 1 @ 60°");
  executeDrive(T3_PHASE1_MS, 30.0);

  // Step 4: Servo → SHOOT
  Serial.println("T3 S4: Servo → SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(500);

  // Step 5: Forward phase 2, intake ON
  Serial.println("T3 S5: Forward phase 2 @ 60°");
  executeDrive(T3_PHASE2_MS, 30.0);

  // Step 6: Pause
  Serial.println("T3 S6: Pause");
  motorsStop();
  delay(T3_PAUSE_MS);

  // Step 7: Servo → NEUTRAL
  Serial.println("T3 S7: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);

  // Step 8: Forward phase 3
  Serial.println("T3 S8: Forward phase 3 @ 60°");
  executeDrive(T3_PHASE3_MS, 30.0);

  // Step 9: Stop, intake OFF
  Serial.println("T3 S9: Stop");
  motorsStop();
  intakeOff();
  delay(200);

  // Step 10: Final SHOOT
  Serial.println("T3 S10: Final SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(250);

  // Step 11: Turn to 180° to align with Side 3 direction
  Serial.println("T3 S11: Realign → 180°");
  executeTurn(180.0);
  waitMs(pause_ms);

  Serial.println("=== TRAJECTORY 3 DONE ===");
  waitMs(pause_ms);
}

// ============================================================
//  TRAJECTORY 4 — Top-Right (Orange → Purple, right side)
//
//  Ball is top-right. Bot must:
//  1. Turn 60° clockwise to face the ball
//  2. Run same intake/shoot logic as Trajectory 2 (at 60° heading)
//  3. Turn to 180° (face Side 3 direction)
//  4. Hand off to rectangle starting from Side 3
// ============================================================
void runTrajectory4() {
  Serial.println("=== TRAJECTORY 4 START (Top-Right: Orange → Purple) ===");

  // Step 1: Turn 60° right to face the ball
  Serial.println("T4 S1: Turn → 60°");
  executeTurn(30.0);
  waitMs(pause_ms);

  intakeOn();

  // Step 2: Servo → NEUTRAL
  Serial.println("T4 S2: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);

  // Step 3: Forward phase 1 (heading locked at 60°)
  Serial.println("T4 S3: Forward phase 1 @ 60°");
  executeDrive(T4_PHASE1_MS, 30.0);

  // Step 4: Stop
  Serial.println("T4 S4: Stop");
  motorsStop();
  delay(150);

  // Step 5: SHOOT
  Serial.println("T4 S5: SHOOT");
  servoWrite(SERVO_SHOOT);
  delay(250);

  // Step 6: Servo → LOAD
  Serial.println("T4 S6: Servo → LOAD");
  servoWrite(SERVO_LOAD);
  delay(250);

  // Step 7: Forward phase 2, intake ON
  Serial.println("T4 S7: Forward phase 2 @ 60°");
  executeDrive(T4_PHASE2_MS, 30.0);

  // Step 8: Servo → NEUTRAL
  Serial.println("T4 S8: Servo → NEUTRAL");
  servoWrite(SERVO_NEUTRAL);
  delay(250);

  // Step 9: Stop
  Serial.println("T4 S9: Stop");
  motorsStop();

  // Step 10: Turn to 180° to align with Side 3 direction
  Serial.println("T4 S10: Realign → 180°");
  executeTurn(180.0);
  waitMs(pause_ms);

  Serial.println("=== TRAJECTORY 4 DONE ===");
  waitMs(pause_ms);
}
void runRectangleLap2() {

  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  // SIDE 1 — time-based 750ms
  Serial.println("=== RECT: Side 1 @ 0° (750ms) ===");
  executeDrive(850, 0.0);
  waitMs(pause_ms);
  servoWrite(SERVO_SHOOT);

  // Intake reverse after Side 1
  intakestore();
  delay(1000);                  // hold reverse briefly before turning

  executeDriveBackward(250, 0.0);   // drive backward for 700ms, holding heading 0.0

  Serial.println("=== RECT: Turn 1 → 90° ===");
  executeTurn(85.0);           // exact 90, no offset
  servoWrite(SERVO_NEUTRAL);
  intakeOn();

  lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.0;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);



  // SIDE 2 — time-based
  Serial.println("=== RECT: Side 2 @ 90° (time, slow) ===");
  executeDriveSpeed(breadth_pause, 90.0 - offset2, FWD_BASE_SPEED_3);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 2 → 135° ===");
  executeTurn(135.0  + offset1);
  waitMs(pause_ms);

  // SIDE 3 — TOF triggered, no shoot
  Serial.println("=== RECT: Side 3 @ 180° (TOF, slow) ===");
  executeDriveUntilCloseSpeed2(180.0, STOP_DISTANCE_CM_2, false, true, FWD_BASE_SPEED_3);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 3 → -135° ===");
  executeTurn(-135.0  + offset1);
  waitMs(pause_ms);


  lap3Counter++;
  int side4Duration = breadth_pause;
  float side4Offset2 = offset2;           // ← add this
  if (lap3Counter >= 3) {
    side4Duration = 450;
    side4Offset2  = -5.0;                  // ← zero offset2 on 3rd lap
    lap3Counter = 0;
    Serial.println("=== LAP 3 SPECIAL: Side 4 shortened + offset zeroed ===");
  }

  // SIDE 4 — time-based
  Serial.println("=== RECT: Side 4 @ -90° (time, slow) ===");
    executeDriveSpeed(breadth_pause, (-90.0 - offset2),  FWD_BASE_SPEED_3);

  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 4 → -45° ===");
  executeTurn(-45.0  + offset1);
  waitMs(pause_ms);

  Serial.println("=== RECT: Lap complete ===");
}


// ============================================================
//  runRectangleLap()
//  Full rectangle lap starting from Side 1 (heading 0°).
//  Used after Trajectory 1 and 2.
//
//  Side 1 → Turn1(45°) → Side2(90°) → Turn2(135°)
//  → Side3(180°) → Turn3(-135°) → Side4(-90°) → Turn4(-45°)
//
//  ALL FOUR SIDES run at FWD_BASE_SPEED_2 (140) via the *Speed
//  functions. Turns are unaffected (PID-driven off heading error).
// ============================================================
void runRectangleLap() {

  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  // SIDE 1 — TOF triggered, shoot midway
  Serial.println("=== RECT: Side 1 @ 0° (TOF + shoot midway, slow) ===");
  executeDriveUntilCloseSpeed(0.0, STOP_DISTANCE_CM, true, false, FWD_BASE_SPEED_2);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 1 → 45° ===");
  // executeTurn(SHOOT_HEADING_DEG);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): SHOOT @ shoot heading ===");
  // servoWrite(SERVO_SHOOT);
  // delay(250);
  servoWrite(SERVO_NEUTRAL);

  Serial.println("=== RECT(S3): Turn 1B → 90° ===");
  executeTurn(45.0 + offset1);
  // servoWrite(SERVO_NEUTRAL);   // ← release shoot after Turn 1
waitMs(pause_ms);
  lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.0;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}

  waitMs(pause_ms);



  // SIDE 2 — time-based
  Serial.println("=== RECT: Side 2 @ 90° (time, slow) ===");
  executeDriveSpeed(breadth_pause, 90.0 - offset2, FWD_BASE_SPEED_3);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 2 → 135° ===");
  executeTurn(135.0  + offset1);
  waitMs(pause_ms);

  // SIDE 3 — TOF triggered, no shoot
  Serial.println("=== RECT: Side 3 @ 180° (TOF, slow) ===");
  executeDriveUntilCloseSpeed(180.0, STOP_DISTANCE_CM_2, false, true, FWD_BASE_SPEED_2);
  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 3 → -135° ===");
  executeTurn(-135.0  + offset1);
  waitMs(pause_ms);


  lap3Counter++;
  int side4Duration = breadth_pause;
  float side4Offset2 = offset2;           // ← add this
  if (lap3Counter >= 3) {
    side4Duration = 450;
    side4Offset2  = -5.0;                  // ← zero offset2 on 3rd lap
    lap3Counter = 0;
    Serial.println("=== LAP 3 SPECIAL: Side 4 shortened + offset zeroed ===");
  }

  // SIDE 4 — time-based
  Serial.println("=== RECT: Side 4 @ -90° (time, slow) ===");
    executeDriveSpeed(side4Duration, (-90.0 - side4Offset2),  FWD_BASE_SPEED_3);

  waitMs(pause_ms);

  Serial.println("=== RECT: Turn 4 → -45° ===");
  executeTurn(-45.0  + offset1);
  waitMs(pause_ms);

  Serial.println("=== RECT: Lap complete ===");
}

// ============================================================
//  runRectangleLapFromSide3()
//  Rectangle lap starting from Side 3 (heading 180°).
//  Used after Trajectory 3 and 4 — bot is already at 180°.
//
//  Side 3 → Turn3(-135°) → Side4(-90°) → Turn4(-45°)
//  → Side1(0°) → Turn1(45°) → Side2(90°) → Turn2(135°)
//
//  ALL FOUR SIDES run at FWD_BASE_SPEED_2 (140) via the *Speed
//  functions. Turns are unaffected (PID-driven off heading error).
// ============================================================
void runRectangleLapFromSide3() {

  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  // SIDE 3 — TOF triggered, no shoot (entering mid-rect)
  Serial.println("=== RECT(S3): Side 3 @ 180° (TOF, slow) ===");
  executeDriveUntilCloseSpeed(180.0, STOP_DISTANCE_CM_2, false, true, FWD_BASE_SPEED_2);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 3 → -135° ===");
  executeTurn(-135.0 + offset1);
  waitMs(pause_ms);

  lap3Counter++;
  int side4Duration = breadth_pause;
  float side4Offset2 = offset2;           // ← add this
  if (lap3Counter >= 3) {
    side4Duration = 450;
    side4Offset2  = -5.0;                  // ← zero offset2 on 3rd lap
    lap3Counter = 0;
    Serial.println("=== LAP 3 SPECIAL: Side 4 shortened + offset zeroed ===");
  }



  // SIDE 4 — time-based
  Serial.println("=== RECT(S3): Side 4 @ -90° (time, slow) ===");
  executeDriveSpeed(side4Duration, -90.0- side4Offset2,FWD_BASE_SPEED_3);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 4 → -45° ===");
  executeTurn(-45.0 + offset1);
  waitMs(pause_ms);

  // SIDE 1 — TOF triggered, shoot midway
  Serial.println("=== RECT(S3): Side 1 @ 0° (TOF + shoot midway, slow) ===");
  executeDriveUntilCloseSpeed(0.0 - offset2, STOP_DISTANCE_CM, true, false, FWD_BASE_SPEED_2);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 1 → 45° ===");
  // executeTurn(SHOOT_HEADING_DEG);
  // waitMs(pause_ms);

  Serial.println("=== RECT(S3): SHOOT @ shoot heading ===");
  // servoWrite(SERVO_SHOOT);
  // delay(250);
  servoWrite(SERVO_NEUTRAL);

  Serial.println("=== RECT(S3): Turn 1B → 90° ===");
  executeTurn(45.0+ offset1);
  waitMs(pause_ms);
lapCounter++;
if (lapCounter >= 2) {
  driftCorrection += 1.0;
  lapCounter = 0;
  Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
}
  waitMs(pause_ms);

  // SIDE 2 — time-based
  Serial.println("=== RECT(S3): Side 2 @ 90° (time, slow) ===");
  executeDriveSpeed(breadth_pause, 90.0- offset2,FWD_BASE_SPEED_3);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Turn 2 → 135° ===");
  executeTurn(135.0 + offset1);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3): Lap complete ===");
}

// ============================================================
//  executeDrive()  [ORIGINAL — used by runTrajectory1-4, FWD_BASE_SPEED=220]
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
//  executeDriveUntilClose()  [ORIGINAL — kept for reference / not
//  used by the rectangle anymore; nothing currently calls this,
//  since Side1/Side3 now go through executeDriveUntilCloseSpeed().
//  Left in place in case you need a fast-TOF approach elsewhere.]
// ============================================================
void executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart) {
    float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 4500;

  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  bool intakeSwitched = false;
  bool servoShot      = false;

  if (reverseIntakeAtStart) intakeReverseSlowed();
  else                      intakeOn();

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;    // ← ADD

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("TOF: MAX TIMEOUT — stopping.");
      break;
    }


   // ── Slowed reverse timing ──────────────────────────────
    if (reverseIntakeAtStart && !servoShot && elapsed >= 50) {
      servoWrite(SERVO_SHOOT);
      servoShot = true;
      Serial.println("Side1: servo SHOOT at 50ms");
    }
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 500) {
      intakeOn();
      servoWrite(SERVO_NEUTRAL);
      intakeSwitched = true;
      Serial.println("Side1: intake ON + servo NEUTRAL at 250ms");
    }
    // Non-blocking shoot state machine
// Non-blocking shoot — fire and hold until Turn 1
if (shootMidway && !shootTriggered && (now - startTime >= 900)) {
  Serial.println("Mid-drive SHOOT triggered — holding until Turn 1");
  servoWrite(SERVO_SHOOT);
  shootTriggered = true;
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
//  runRectangleLapFromSide3_2()
//  Phase-2 counterpart to runRectangleLapFromSide3(). Identical
//  except Side 1 uses the same time-based approach as
//  runRectangleLap2()'s Side 1, instead of the TOF-triggered one.
// ============================================================
void runRectangleLapFromSide3_2() {

  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  // SIDE 3 — TOF triggered, no shoot (entering mid-rect)
  Serial.println("=== RECT(S3-2): Side 3 @ 180° (TOF, slow) ===");
  executeDriveUntilCloseSpeed2(180.0, STOP_DISTANCE_CM_2, false, true, FWD_BASE_SPEED_2);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3-2): Turn 3 → -135° ===");
  executeTurn(-135.0 + offset1);
  waitMs(pause_ms);

  lap3Counter++;
  int side4Duration = breadth_pause;
  float side4Offset2 = offset2;
  if (lap3Counter >= 3) {
    side4Duration = 450;
    side4Offset2  = -5.0;
    lap3Counter = 0;
    Serial.println("=== LAP 3 SPECIAL: Side 4 shortened + offset zeroed ===");
  }

  // SIDE 4 — time-based
  Serial.println("=== RECT(S3-2): Side 4 @ -90° (time, slow) ===");
  executeDriveSpeed(breadth_pause, -90.0 - offset2, FWD_BASE_SPEED_3);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3-2): Turn 4 → -45° ===");
  executeTurn(-45.0 + offset1);
  waitMs(pause_ms);

  // ── SIDE 1 — swapped to time-based (matches runRectangleLap2()) ──
  Serial.println("=== RECT(S3-2): Side 1 @ 0° (600ms) ===");
  executeDrive(850, 0.0);
  waitMs(pause_ms);
  servoWrite(SERVO_SHOOT);

  intakestore();
  delay(1000);
executeDriveBackward(250, 0.0);   // drive backward for 700ms, holding heading 0.0

  Serial.println("=== RECT(S3-2): Turn 1 → 90° ===");
  executeTurn(85.0);
  servoWrite(SERVO_NEUTRAL);
  intakeOn();

  lapCounter++;
  if (lapCounter >= 2) {
    driftCorrection += 1.0;
    lapCounter = 0;
    Serial.print("=== DRIFT CORRECTION: "); Serial.println(driftCorrection, 1);
  }
  waitMs(pause_ms);

  // SIDE 2 — time-based
  Serial.println("=== RECT(S3-2): Side 2 @ 90° (time, slow) ===");
  executeDriveSpeed(breadth_pause, 90.0 - offset2, FWD_BASE_SPEED_3);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3-2): Turn 2 → 135° ===");
  executeTurn(135.0 + offset1);
  waitMs(pause_ms);

  Serial.println("=== RECT(S3-2): Lap complete ===");
}



// ============================================================
//  executeDriveBackward()  [ORIGINAL-STYLE — same signature as
//  executeDrive(), but drives in reverse while still tracking
//  targetHeading. Uses FWD_BASE_SPEED, same as executeDrive().]
// ============================================================
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
//  executeDriveUntilCloseSpeed()  [RECTANGLE-ONLY — wired into
//  Side 1 and Side 3 of both runRectangleLap() and
//  runRectangleLapFromSide3().
//
//  TWO-PHASE SPEED:
//    elapsed < RECT_FULLSPEED_MS (600ms) → FWD_BASE_SPEED (220, full speed)
//    elapsed >= RECT_FULLSPEED_MS        → `speed` param (FWD_BASE_SPEED_2 = 140)
//  The TOF stop check runs the whole time; in practice it crosses
//  the trigger distance during the slow crawl phase.
// ============================================================
void executeDriveUntilCloseSpeed(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart, const int speed, unsigned long shootDelayMs) {
    float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 2000;

  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  bool intakeSwitched = false;
  bool servoShot      = false;

  if (reverseIntakeAtStart) intakeReverseSlowed();
  else                      intakeOn();

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;    // ← ADD

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("TOF: MAX TIMEOUT — stopping.");
      break;
    }


   // ── Slowed reverse timing ──────────────────────────────
    if (reverseIntakeAtStart && !servoShot && elapsed >= 50) {
      servoWrite(SERVO_SHOOT);
      servoShot = true;
      Serial.println("Side1: servo SHOOT at 50ms");
    }
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 500) {
      intakeOn();
      servoWrite(SERVO_NEUTRAL);
      intakeSwitched = true;
      Serial.println("Side1: intake ON + servo NEUTRAL at 250ms");
    }
    // Non-blocking shoot state machine
// Non-blocking shoot — fire and hold until Turn 1
// Non-blocking shoot state machine
// Non-blocking shoot — fire and hold until Turn 1
if (shootMidway && !shootTriggered && (now - startTime >= shootDelayMs)) {
  Serial.print("Mid-drive SHOOT triggered at ");
  Serial.print(shootDelayMs);
  Serial.println("ms — holding");
  servoWrite(SERVO_SHOOT);
  shootTriggered = true;
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
      // ── Two-phase speed: full speed first, then slow crawl ──
      int currentSpeed = (elapsed < RECT_FULLSPEED_MS) ? FWD_BASE_SPEED : speed;
      runForwardPIDSpeed(targetHeading, prevErr, integ, currentSpeed);
    }
  }

  motorsStop();
}


void executeDriveUntilCloseSpeed2(float targetHeading, float stopDistCm, bool shootMidway, bool reverseIntakeAtStart, const int speed) {
    float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 2000;

  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  bool intakeSwitched = false;
  bool servoShot      = false;

  if (reverseIntakeAtStart) intakeReverseSlowed();
  else                      intakeOn();

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;    // ← ADD

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("TOF: MAX TIMEOUT — stopping.");
      break;
    }


   // ── Slowed reverse timing ──────────────────────────────
    if (reverseIntakeAtStart && !servoShot && elapsed >= 50) {
      servoWrite(SERVO_SHOOT);
      servoShot = true;
      Serial.println("Side1: servo SHOOT at 50ms");
    }
    if (reverseIntakeAtStart && !intakeSwitched && elapsed >= 500) {
      intakeOn();
      servoWrite(SERVO_NEUTRAL);
      intakeSwitched = true;
      Serial.println("Side1: intake ON + servo NEUTRAL at 250ms");
    }
    // Non-blocking shoot state machine
// Non-blocking shoot — fire and hold until Turn 1
if (shootMidway && !shootTriggered && (now - startTime >= 650)) {
  Serial.println("Mid-drive SHOOT triggered — holding until Turn 1");
  servoWrite(SERVO_SHOOT);
  shootTriggered = true;
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
      // ── Two-phase speed: full speed first, then slow crawl ──
      int currentSpeed = (elapsed < RECT_FULLSPEED_MS) ? FWD_BASE_SPEED_4 : speed;
      runForwardPIDSpeed(targetHeading, prevErr, integ, currentSpeed);
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
        Serial.print("Turn done. H:"); Serial.print(heading, 1); Serial.println("deg");
        return;
      } else {
        stableCount = 0;
        runTurnPID(targetAngle, prevErr, integ);
      }
    }
  }
}

// ============================================================
//  executeDriveSpeed()  [RECTANGLE-ONLY — Side2/Side4, FWD_BASE_SPEED_2]
// ============================================================
void executeDriveSpeed(unsigned long durationMs, float targetHeading, const int speed) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  while (millis() - startTime < durationMs) {
    unsigned long now = millis();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPIDSpeed(targetHeading, prevErr, integ, speed);
    }
  }
  motorsStop();
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
// ============================================================
//  runForwardPIDBackward()  [mirrors runForwardPID(), but drives
//  the wheels in reverse — heading correction logic unchanged.]
// ============================================================
void runForwardPIDBackward(float targetHeading, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetHeading, heading);
  float dt      = FWD_INTERVAL / 1000.0;

  if (abs(error) <= FWD_DEADBAND) {
    integ   = 0.0;
    prevErr = error;
    driveMotors(-FWD_BASE_SPEED, -FWD_BASE_SPEED);
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
  int leftSpeed  = constrain(FWD_BASE_SPEED + (int)correction, 0, 255);
  int rightSpeed = constrain(FWD_BASE_SPEED - (int)correction, 0, 255);

  driveMotors(-leftSpeed, -rightSpeed);

  Serial.print("BWD | H:"); Serial.print(heading, 1);
  Serial.print(" E:"); Serial.print(error, 1);
  Serial.print(" L:"); Serial.print(-leftSpeed);
  Serial.print(" R:"); Serial.println(-rightSpeed);
}
// ============================================================
//  runForwardPID()  [ORIGINAL — used by executeDrive/executeDriveUntilClose,
//  i.e. runTrajectory1-4. Hardcodes FWD_BASE_SPEED = 220.]
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
// correction to be updated for left and right --- leftspeed to be the highest and right speed to be [255 - 2(correction)]
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
void initStartSwitch() {
  pinMode(SWITCH_PWR, OUTPUT);
  digitalWrite(SWITCH_PWR, HIGH);
  pinMode(SWITCH_SIG, INPUT_PULLDOWN);
  Serial.println("Start switch ready.");
}


bool isStartSwitchOn() {
  return digitalRead(SWITCH_SIG) == HIGH;
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
}// ============================================================
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

void intakeReverseSlowed() { analogWrite(INTAKE_IN1, SLOW); analogWrite(INTAKE_IN2, 0); }
void intakestore() { analogWrite(INTAKE_IN1, SLOWED); analogWrite(INTAKE_IN2, 0); }

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

float getHeading() { return wrapAngle(getRawYaw() - yawOffset - driftCorrection); }

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
