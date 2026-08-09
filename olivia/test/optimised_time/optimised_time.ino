// ============================================================
//  FULL RUN — ESP32-S3 + BNO085 + VL53L0X + Pixy2 + ESP-NOW
//  OLIVIA / SHAWN — identical code flashed on both bots
//
//  BOOT FLOW:
//    1. Init all hardware (~1 second total)
//    2. Sample Pixy2 — early exit at 5 confident votes
//    3. Broadcast zone to partner over ESP-NOW
//    4. Wait for partner's zone (500ms timeout)
//    5. Decide role:
//       SAW PURPLE   → STORER  → run storing trajectory → rectangle
//       SAW NOTHING  → SHOOTER → flip zone → run shoot trajectory → rectangle
//
//  MAC Address format:
//  uint8_t partnerMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>
#include <Pixy2SPI_SS.h>
#include <WiFi.h>
#include <esp_now.h>


// ============================================================
//  ★ SET THIS TO true WHEN TESTING WITHOUT SHAWN ★
//  SET TO false WHEN BOTH BOTS ARE PRESENT
// ============================================================
#define SOLO_MODE true

// ============================================================
//  ★ FILL IN PARTNER MAC AFTER GETTING IT FROM SERIAL ★
// ============================================================
uint8_t partnerMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ============================================================
//  ★ TUNE THESE ★
// ============================================================

// --- Trajectory 1 — STORE — Bot-Left ---
const unsigned long T1_PHASE1_MS = 450;
const unsigned long T1_PHASE2_MS = 300;
const unsigned long T1_PHASE3_MS = 200;
const unsigned long T1_PAUSE_MS  = 200;
const int           T1_SPEED     = 180;

// --- Trajectory 2 — STORE — Top-Left ---
const unsigned long T2_PHASE1_MS = 400;
const unsigned long T2_PHASE2_MS = 200;
const int           T2_SPEED     = 180;

// --- Trajectory 3 — STORE — Bot-Right ---
const unsigned long T3_PHASE1_MS = 450;
const unsigned long T3_PHASE2_MS = 300;
const unsigned long T3_PHASE3_MS = 200;
const unsigned long T3_PAUSE_MS  = 200;
const int           T3_SPEED     = 180;

// --- Trajectory 4 — STORE — Top-Right ---
const unsigned long T4_PHASE1_MS = 400;
const unsigned long T4_PHASE2_MS = 200;
const int           T4_SPEED     = 180;

// --- Shoot Trajectories (time-based, same positions as store) ---
const unsigned long S1_DRIVE_MS  = 400;   // Shoot for BOT-LEFT  (partner saw BOT-RIGHT → flip → BOT-LEFT)
const unsigned long S2_DRIVE_MS  = 400;   // Shoot for TOP-LEFT  (partner saw TOP-RIGHT → flip → TOP-LEFT)
const unsigned long S3_DRIVE_MS  = 400;   // Shoot for BOT-RIGHT (partner saw BOT-LEFT  → flip → BOT-RIGHT)
const unsigned long S4_DRIVE_MS  = 400;   // Shoot for TOP-RIGHT (partner saw TOP-LEFT  → flip → TOP-RIGHT)
const int           SHOOT_SPEED  = 180;

// --- Rectangle ---
const float          STOP_DISTANCE_CM = 12.0;
const unsigned long  BREADTH_PAUSE    = 500;
const int            PAUSE_MS         = 50;

// --- Pixy ---
#define SIG_PURPLE          2
#define MIN_AREA            200
#define ROI_TOP_Y           55
#define SPLIT_X             210
#define SPLIT_Y             81
#define DEAD_X              5
#define DEAD_Y              5
#define PIXY_CONFIDENCE     5      // votes needed for early exit
#define PIXY_MAX_MS         800    // max sample window (down from 1500)
#define ESPNOW_TIMEOUT_MS   500    // wait for partner data

// ============================================================
//  HARDWARE PINS
// ============================================================

const int M1A = 6;
const int M1B = 7;
const int M2A = 15;
const int M2B = 16;

#define BNO08X_SDA 18
#define BNO08X_SCL 17
#define BNO08X_RST 12

#define TOF_SDA 46
#define TOF_SCL 9

const int INTAKE_IN1   = 3;
const int INTAKE_IN2   = 8;
const int INTAKE_SPEED = 255;

#define SERVO_PIN 4
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

const int SERVO_NEUTRAL = 0;
const int SERVO_SHOOT   = 50;
const int SERVO_LOAD    = 140;

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

// Zone enum — value matches packet uint8_t directly
enum Zone : uint8_t {
  UNKNOWN   = 0,
  TOP_LEFT  = 1,
  BOT_LEFT  = 2,
  TOP_RIGHT = 3,
  BOT_RIGHT = 4
};

// ESP-NOW packet
typedef struct {
  uint8_t zone;       // Zone enum value
  bool    sawPurple;  // true = I saw purple
} ZonePacket;

ZonePacket myPacket;
ZonePacket partnerPacket;
volatile bool partnerDataReceived = false;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================

// ESP-NOW
void  initESPNow();
void  sendMyZone(Zone z, bool sawPurple);
bool  waitForPartner(Zone myZone, bool iSawPurple, unsigned long timeoutMs);
void  onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void  onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
Zone  flipZone(Zone z);

// Pixy
Zone  detectPurpleBall();
Zone  classifyZone(int cx, int cy);

// Storing trajectories (I see purple)
void  runStoreTrajectory(Zone z);
void  runStore1();   // BOT-LEFT
void  runStore2();   // TOP-LEFT
void  runStore3();   // BOT-RIGHT
void  runStore4();   // TOP-RIGHT

// Shooting trajectories (partner sees purple, I shoot)
void  runShootTrajectory(Zone z);
void  runShoot1();   // BOT-LEFT  (partner saw BOT-RIGHT)
void  runShoot2();   // TOP-LEFT  (partner saw TOP-RIGHT)
void  runShoot3();   // BOT-RIGHT (partner saw BOT-LEFT)
void  runShoot4();   // TOP-RIGHT (partner saw TOP-LEFT)

// Rectangle
void  runRectangleLap();
void  runRectangleLapFromSide3();

// Motion
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

// Hardware
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

// ============================================================
//  SETUP — optimized to ~1 second total
// ============================================================



void setup() {
  Serial.begin(115200);

  setMotorPins();
  motorsStop();
  initIntake();
  initServo();
  initIMU();
  initTOF();

  if (!SOLO_MODE) {
    initESPNow();
  } else {
    // Still print MAC so you can note it down for later
    WiFi.mode(WIFI_STA);
    Serial.print("MY MAC (note this down): ");
    Serial.println(WiFi.macAddress());
  }

  pixy.init();
  pixy.changeProg("color_connected_components");
  delay(100);

  zeroIMU();
  Serial.println("=== ALL HARDWARE READY ===");
}


// void setup() {
//   Serial.begin(115200);

//   // Motors first — bot should never twitch on boot
//   setMotorPins();
//   motorsStop();

//   // Intake off
//   initIntake();

//   // Servo — reduced delay (200ms is enough to reach neutral)
//   initServo();

//   // IMU — reduced internal delay
//   initIMU();

//   // TOF
//   initTOF();

//   // ESP-NOW — init before Pixy so radio is ready
//   initESPNow();

//   // Pixy
//   pixy.init();
//   pixy.changeProg("color_connected_components");
//   delay(100);   // reduced from 500ms — pixy just needs to switch prog

//   // Final IMU zero right before action
//   zeroIMU();

//   Serial.println("=== ALL HARDWARE READY ===");
// }

// ============================================================
//  LOOP
// ============================================================




void loop() {

  // ── STEP 1: Pixy sampling ────────────────────────────────
  Zone myZone     = detectPurpleBall();
  bool iSawPurple = (myZone != UNKNOWN);

  Serial.print("MY ZONE: "); Serial.print(myZone);
  Serial.print(" | SAW PURPLE: "); Serial.println(iSawPurple ? "YES" : "NO");

  if (!SOLO_MODE) {
    // ── STEP 2: Broadcast + wait for partner ───────────────
    sendMyZone(myZone, iSawPurple);
    waitForPartner(myZone, iSawPurple, 2000);
  } else {
    Serial.println("SOLO MODE — skipping ESP-NOW");
  }

  // ── STEP 3: Decide role ──────────────────────────────────
  if (iSawPurple) {
    Serial.println("ROLE: STORER");
    runStoreTrajectory(myZone);
    if (myZone == BOT_LEFT || myZone == TOP_LEFT) {
      while (true) { runRectangleLap(); }
    } else {
      while (true) { runRectangleLapFromSide3(); }
    }
  } else {
    // In solo mode this won't happen since purple is guaranteed
    Serial.println("SOLO MODE: No purple seen — going to rectangle");
    while (true) { runRectangleLap(); }
  }
}



// void loop() {

//   // ── STEP 1: Sample Pixy — early exit on confidence ───────
//   Zone myZone = detectPurpleBall();
//   bool iSawPurple = (myZone != UNKNOWN);

//   Serial.print("MY ZONE: "); Serial.print(myZone);
//   Serial.print(" | SAW PURPLE: "); Serial.println(iSawPurple ? "YES" : "NO");

//   // ── STEP 2: Broadcast my result to partner ────────────────
//   sendMyZone(myZone, iSawPurple);

//   // ── STEP 3: Wait for partner's result ────────────────────
//   bool partnerResponded = waitForPartner(ESPNOW_TIMEOUT_MS);

//   // ── STEP 4: Decide role and act ──────────────────────────
//   if (iSawPurple) {

//     // ── I AM THE STORER ──────────────────────────────────
//     Serial.println("ROLE: STORER");
//     runStoreTrajectory(myZone);

//     // Rectangle entry depends on which trajectory ran
//     if (myZone == BOT_LEFT || myZone == TOP_LEFT) {
//       Serial.println("=== RECTANGLE LOOP (from Side 1) ===");
//       while (true) { runRectangleLap(); }
//     } else {
//       Serial.println("=== RECTANGLE LOOP (from Side 3) ===");
//       while (true) { runRectangleLapFromSide3(); }
//     }

//   } else {

//     // ── I AM THE SHOOTER ─────────────────────────────────
//     Serial.println("ROLE: SHOOTER");

//     if (partnerResponded && (Zone)partnerPacket.zone != UNKNOWN) {
//       // Flip partner's zone — that's where opponent's purple is from my view
//       Zone flipped = flipZone((Zone)partnerPacket.zone);
//       Serial.print("Partner zone: "); Serial.print(partnerPacket.zone);
//       Serial.print(" → Flipped: "); Serial.println(flipped);
//       runShootTrajectory(flipped);
//     } else {
//       // Partner didn't respond — skip shoot, go straight to rectangle
//       Serial.println("No partner data — skipping shoot, going to rectangle");
//     }

//     // Shooter always starts rectangle from Side 1
//     Serial.println("=== RECTANGLE LOOP (from Side 1) ===");
//     while (true) { runRectangleLap(); }
//   }
// }

// ============================================================
//  ESP-NOW
// ============================================================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "DELIVERED" : "FAILED");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&partnerPacket, data, sizeof(partnerPacket));
  partnerDataReceived = true;
  Serial.print("ESP-NOW recv | Partner zone: "); Serial.print(partnerPacket.zone);
  Serial.print(" sawPurple: "); Serial.println(partnerPacket.sawPurple ? "YES" : "NO");
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  Serial.print("MY MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED"); while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, partnerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer"); while (true) delay(1000);
  }
  Serial.println("ESP-NOW ready.");
}

void sendMyZone(Zone z, bool sawPurple) {
  myPacket.zone      = (uint8_t)z;
  myPacket.sawPurple = sawPurple;
  esp_now_send(partnerMAC, (uint8_t *)&myPacket, sizeof(myPacket));
}

bool waitForPartner(Zone myZone, bool iSawPurple, unsigned long timeoutMs) {
  unsigned long start    = millis();
  unsigned long lastSend = millis();

  while (!partnerDataReceived) {
    // Resend every 200ms in case packet dropped
    if (millis() - lastSend > 200) {
      sendMyZone(myZone, iSawPurple);
      lastSend = millis();
      Serial.println("ESP-NOW: Resending...");
    }
    if (millis() - start > timeoutMs) {
      Serial.println("ESP-NOW: Partner timeout — acting alone");
      return false;
    }
    delay(10);
  }
  return true;
}

// ── Zone flip — both axes ────────────────────────────────────
Zone flipZone(Zone z) {
  switch (z) {
    case BOT_LEFT:  return TOP_RIGHT;
    case BOT_RIGHT: return TOP_LEFT;
    case TOP_LEFT:  return BOT_RIGHT;
    case TOP_RIGHT: return BOT_LEFT;
    default:        return UNKNOWN;
  }
}

// ============================================================
//  detectPurpleBall() — optimized with early exit
// ============================================================
Zone detectPurpleBall() {
  int voteTL = 0, voteBL = 0, voteTR = 0, voteBR = 0;

  Serial.println("=== PIXY: Sampling... ===");
  unsigned long start = millis();

  while (millis() - start < PIXY_MAX_MS) {
    pixy.ccc.getBlocks();

    int      bestCx = 0, bestCy = 0;
    uint32_t bestArea = 0;
    bool     found = false;

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
      switch (z) {
        case TOP_LEFT:  voteTL++; break;
        case BOT_LEFT:  voteBL++; break;
        case TOP_RIGHT: voteTR++; break;
        case BOT_RIGHT: voteBR++; break;
        default: break;
      }
      Serial.print("cx="); Serial.print(bestCx);
      Serial.print(" cy="); Serial.print(bestCy);
      Serial.print(" | TL="); Serial.print(voteTL);
      Serial.print(" BL="); Serial.print(voteBL);
      Serial.print(" TR="); Serial.print(voteTR);
      Serial.print(" BR="); Serial.println(voteBR);

      // ── Early exit on confidence ────────────────────────
      if (voteTL >= PIXY_CONFIDENCE) { Serial.println("CONFIDENT: TOP-LEFT");  return TOP_LEFT;  }
      if (voteBL >= PIXY_CONFIDENCE) { Serial.println("CONFIDENT: BOT-LEFT");  return BOT_LEFT;  }
      if (voteTR >= PIXY_CONFIDENCE) { Serial.println("CONFIDENT: TOP-RIGHT"); return TOP_RIGHT; }
      if (voteBR >= PIXY_CONFIDENCE) { Serial.println("CONFIDENT: BOT-RIGHT"); return BOT_RIGHT; }
    }

    delay(30);   // reduced from 50ms
  }

  // Window expired — pick highest vote
  Serial.println("─── PIXY FINAL VOTE ───");
  Serial.print("TL="); Serial.print(voteTL);
  Serial.print(" BL="); Serial.print(voteBL);
  Serial.print(" TR="); Serial.print(voteTR);
  Serial.print(" BR="); Serial.println(voteBR);

  int best = max({voteTL, voteBL, voteTR, voteBR});
  if (best == 0)          return UNKNOWN;
  if (voteTL == best)     return TOP_LEFT;
  if (voteBL == best)     return BOT_LEFT;
  if (voteTR == best)     return TOP_RIGHT;
  return BOT_RIGHT;
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
//  STORE TRAJECTORIES — dispatch
// ============================================================
void runStoreTrajectory(Zone z) {
  switch (z) {
    case BOT_LEFT:  runStore1(); break;
    case TOP_LEFT:  runStore2(); break;
    case BOT_RIGHT: runStore3(); break;
    case TOP_RIGHT: runStore4(); break;
    default: break;
  }
}

// ============================================================
//  SHOOT TRAJECTORIES — dispatch
// ============================================================
void runShootTrajectory(Zone z) {
  switch (z) {
    case BOT_LEFT:  runShoot1(); break;
    case TOP_LEFT:  runShoot2(); break;
    case BOT_RIGHT: runShoot3(); break;
    case TOP_RIGHT: runShoot4(); break;
    default: break;
  }
}

// ============================================================
//  runStore1 — BOT-LEFT (Purple → Orange)
// ============================================================
void runStore1() {
  Serial.println("=== STORE 1: BOT-LEFT ===");
  intakeOn();
  servoWrite(SERVO_LOAD);  delay(500);
  executeDrive(T1_PHASE1_MS, 0.0);
  servoWrite(SERVO_SHOOT); delay(500);
  executeDrive(T1_PHASE2_MS, 0.0);
  motorsStop(); delay(T1_PAUSE_MS);
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDrive(T1_PHASE3_MS, 0.0);
  motorsStop(); intakeOff(); delay(200);
  servoWrite(SERVO_SHOOT); delay(250);
  Serial.println("=== STORE 1 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  runStore2 — TOP-LEFT (Orange → Purple)
// ============================================================
void runStore2() {
  Serial.println("=== STORE 2: TOP-LEFT ===");
  intakeOn();
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDrive(T2_PHASE1_MS, 0.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_LOAD);  delay(250);
  executeDrive(T2_PHASE2_MS, 0.0);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop();
  Serial.println("=== STORE 2 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  runStore3 — BOT-RIGHT (60° turn + Store1 logic + realign)
// ============================================================
void runStore3() {
  Serial.println("=== STORE 3: BOT-RIGHT ===");
  executeTurn(60.0); waitMs(PAUSE_MS);
  intakeOn();
  servoWrite(SERVO_LOAD);  delay(500);
  executeDrive(T3_PHASE1_MS, 60.0);
  servoWrite(SERVO_SHOOT); delay(500);
  executeDrive(T3_PHASE2_MS, 60.0);
  motorsStop(); delay(T3_PAUSE_MS);
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDrive(T3_PHASE3_MS, 60.0);
  motorsStop(); intakeOff(); delay(200);
  servoWrite(SERVO_SHOOT); delay(250);
  executeTurn(180.0); waitMs(PAUSE_MS);
  Serial.println("=== STORE 3 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  runStore4 — TOP-RIGHT (60° turn + Store2 logic + realign)
// ============================================================
void runStore4() {
  Serial.println("=== STORE 4: TOP-RIGHT ===");
  executeTurn(60.0); waitMs(PAUSE_MS);
  intakeOn();
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeDrive(T4_PHASE1_MS, 60.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_LOAD);  delay(250);
  executeDrive(T4_PHASE2_MS, 60.0);
  servoWrite(SERVO_NEUTRAL); delay(250);
  motorsStop();
  executeTurn(180.0); waitMs(PAUSE_MS);
  Serial.println("=== STORE 4 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  runShoot1 — Shoot at BOT-LEFT
//  (partner saw BOT-RIGHT → flipped → BOT-LEFT)
// ============================================================
void runShoot1() {
  Serial.println("=== SHOOT 1: targeting BOT-LEFT ===");
  executeDrive(S1_DRIVE_MS, 0.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_NEUTRAL); delay(250);
  Serial.println("=== SHOOT 1 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  runShoot2 — Shoot at TOP-LEFT
//  (partner saw TOP-RIGHT → flipped → TOP-LEFT)
// ============================================================
void runShoot2() {
  Serial.println("=== SHOOT 2: targeting TOP-LEFT ===");
  executeDrive(S2_DRIVE_MS, 0.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_NEUTRAL); delay(250);
  Serial.println("=== SHOOT 2 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  runShoot3 — Shoot at BOT-RIGHT
//  (partner saw BOT-LEFT → flipped → BOT-RIGHT)
// ============================================================
void runShoot3() {
  Serial.println("=== SHOOT 3: targeting BOT-RIGHT ===");
  executeTurn(60.0); waitMs(PAUSE_MS);
  executeDrive(S3_DRIVE_MS, 60.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeTurn(0.0); waitMs(PAUSE_MS);
  Serial.println("=== SHOOT 3 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  runShoot4 — Shoot at TOP-RIGHT
//  (partner saw TOP-LEFT → flipped → TOP-RIGHT)
// ============================================================
void runShoot4() {
  Serial.println("=== SHOOT 4: targeting TOP-RIGHT ===");
  executeTurn(60.0); waitMs(PAUSE_MS);
  executeDrive(S4_DRIVE_MS, 60.0);
  motorsStop(); delay(150);
  servoWrite(SERVO_SHOOT); delay(250);
  servoWrite(SERVO_NEUTRAL); delay(250);
  executeTurn(0.0); waitMs(PAUSE_MS);
  Serial.println("=== SHOOT 4 DONE ===");
  waitMs(PAUSE_MS);
}

// ============================================================
//  RECTANGLE — from Side 1
// ============================================================
void runRectangleLap() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  Serial.println("=== RECT: Side 1 @ 0° (TOF + shoot midway) ===");
  executeDriveUntilClose(0.0, STOP_DISTANCE_CM, true);
  waitMs(PAUSE_MS);

  Serial.println("=== RECT: Turn 1 → 45° ===");
  executeTurn(45.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT: Side 2 @ 90° (time) ===");
  executeDrive(BREADTH_PAUSE, 90.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT: Turn 2 → 135° ===");
  executeTurn(135.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT: Side 3 @ 180° (TOF) ===");
  executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false);
  waitMs(PAUSE_MS);

  Serial.println("=== RECT: Turn 3 → -135° ===");
  executeTurn(-135.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT: Side 4 @ -90° (time) ===");
  executeDrive(BREADTH_PAUSE, -90.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT: Turn 4 → -45° ===");
  executeTurn(-45.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT: Lap complete ===");
}

// ============================================================
//  RECTANGLE — from Side 3
// ============================================================
void runRectangleLapFromSide3() {
  intakeOn();
  servoWrite(SERVO_NEUTRAL);

  Serial.println("=== RECT(S3): Side 3 @ 180° (TOF) ===");
  executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false);
  waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Turn 3 → -135° ===");
  executeTurn(-135.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Side 4 @ -90° (time) ===");
  executeDrive(BREADTH_PAUSE, -90.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Turn 4 → -45° ===");
  executeTurn(-45.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Side 1 @ 0° (TOF + shoot midway) ===");
  executeDriveUntilClose(0.0, STOP_DISTANCE_CM, true);
  waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Turn 1 → 45° ===");
  executeTurn(45.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Side 2 @ 90° (time) ===");
  executeDrive(BREADTH_PAUSE, 90.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Turn 2 → 135° ===");
  executeTurn(135.0); waitMs(PAUSE_MS);

  Serial.println("=== RECT(S3): Lap complete ===");
}

// ============================================================
//  executeDrive()
// ============================================================
void executeDrive(unsigned long durationMs, float targetHeading) {
  float prevErr = 0.0, integ = 0.0;
  unsigned long startTime = millis(), lastPIDTime = millis();
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
  float prevErr = 0.0, integ = 0.0;
  unsigned long lastPIDTime = millis(), startTime = millis();
  const unsigned long MAX_TIMEOUT = 2000;
  bool shootDone = false, shootTriggered = false;
  unsigned long shootStartTime = 0;
  int shootPhase = 0;

  while (true) {
    unsigned long now = millis();
    if (now - startTime > MAX_TIMEOUT) { Serial.println("TOF: TIMEOUT"); break; }

    if (shootMidway && !shootDone) {
      if (!shootTriggered && (now - startTime >= 600)) {
        servoWrite(SERVO_SHOOT); shootTriggered = true;
        shootStartTime = now; shootPhase = 1;
        Serial.println("Mid-drive SHOOT triggered");
      }
      if (shootPhase == 1 && (now - shootStartTime >= 250)) {
        servoWrite(SERVO_NEUTRAL); shootPhase = 2;
      }
      if (shootPhase == 2 && (now - shootStartTime >= 500)) {
        shootDone = true; Serial.println("Mid-drive SHOOT done");
      }
    }

    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      float dist = getDistanceCm();
      Serial.print("TOF: "); Serial.print(dist, 1); Serial.print("cm");
      if (dist > 0 && dist < stopDistCm) {
        Serial.println(" → STOP"); motorsStop(); break;
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
  float prevErr = 0.0, integ = 0.0;
  int stableCount = 0;
  unsigned long lastPIDTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - lastPIDTime >= TRN_INTERVAL) {
      lastPIDTime = now;
      float heading = getHeading();
      float error   = shortestError(targetAngle, heading);
      if (abs(error) <= TRN_DEADBAND) {
        motorsStop();
        Serial.print("Turn done @ "); Serial.print(heading, 1); Serial.println("°");
        return;
      }
      runTurnPID(targetAngle, prevErr, integ);
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
    integ = 0.0; prevErr = error;
    driveMotors(FWD_BASE_SPEED, FWD_BASE_SPEED);
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
}

// ============================================================
//  TOF
// ============================================================
void initTOF() {
  Wire1.begin(TOF_SDA, TOF_SCL);
  if (!lox.begin(0x29, false, &Wire1)) {
    Serial.println("VL53L0X not found!"); while (1) delay(10);
  }
  Serial.println("TOF ready.");
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
  delay(200);   // reduced from 500ms
  Serial.println("Servo ready.");
}

void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  ledcWrite(SERVO_PIN, map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY));
}

// ============================================================
//  INTAKE
// ============================================================
void initIntake() {
  pinMode(INTAKE_IN1, OUTPUT); pinMode(INTAKE_IN2, OUTPUT);
  analogWrite(INTAKE_IN1, 0);  analogWrite(INTAKE_IN2, 0);
}

void intakeOn()  { analogWrite(INTAKE_IN1, 0); analogWrite(INTAKE_IN2, INTAKE_SPEED); }
void intakeOff() { analogWrite(INTAKE_IN1, 0); analogWrite(INTAKE_IN2, 0); }

// ============================================================
//  IMU
// ============================================================
void initIMU() {
  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not found!"); while (1) delay(10);
  }
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  delay(100);   // reduced from 200ms
  Serial.println("IMU ready.");
}

void zeroIMU() {
  for (int i = 0; i < 10; i++) {   // reduced from 20 samples
    bno.getSensorEvent(&imuData);
    delay(10);
  }
  yawOffset = getRawYaw();
  Serial.print("IMU zeroed @ "); Serial.print(yawOffset, 2); Serial.println("°");
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

float getHeading()                           { return wrapAngle(getRawYaw() - yawOffset); }
float shortestError(float t, float c)        { return wrapAngle(wrapAngle(t) - wrapAngle(c)); }
float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ============================================================
//  MOTORS
// ============================================================
void driveMotors(int l, int r) {
  analogWrite(M1A, l > 0 ? 0 : (l < 0 ? -l : 0));
  analogWrite(M1B, l > 0 ? l : 0);
  analogWrite(M2A, r > 0 ? r : (r < 0 ? 0 : 0));
  analogWrite(M2B, r > 0 ? 0 : (r < 0 ? -r : 0));
}

void turnClockwise(int s)    { analogWrite(M1A,0); analogWrite(M1B,s); analogWrite(M2A,0); analogWrite(M2B,s); }
void turnAntiClockwise(int s){ analogWrite(M1A,s); analogWrite(M1B,0); analogWrite(M2A,s); analogWrite(M2B,0); }
void motorsStop()            { analogWrite(M1A,0); analogWrite(M1B,0); analogWrite(M2A,0); analogWrite(M2B,0); }
void setMotorPins()          { pinMode(M1A,OUTPUT); pinMode(M1B,OUTPUT); pinMode(M2A,OUTPUT); pinMode(M2B,OUTPUT); }
void waitMs(int ms)          { motorsStop(); delay(ms); }