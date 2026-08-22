// ============================================================
//  Pixy2 Zone Calibration Tool  v2
//  Works for both Olivia and Shawn — pick one below
//
//  Usage:
//    1. Select your robot (comment/uncomment line below)
//    2. Upload → Open Serial Monitor at 115200 baud
//    3. Place bot at each corner when prompted → press ENTER
//    4. Copy the 4 #define lines into your main code
// ============================================================

// ── ROBOT SELECTOR ──────────────────────────────────────────
#define ROBOT_SHAWN       // <─ comment this out for OLIVIA
// #define ROBOT_OLIVIA   // <─ uncomment this for OLIVIA
// ────────────────────────────────────────────────────────────

#ifdef ROBOT_SHAWN
  #define ROBOT_NAME    "SHAWN"
  #define SIG2_MIN_AREA  100     // Shawn's threshold
  #define ROI_TOP_Y       40     // Shawn's ROI filter
#else
  #define ROBOT_NAME    "OLIVIA"
  #define SIG2_MIN_AREA  200     // Olivia's threshold
  #define ROI_TOP_Y       55     // Olivia's ROI filter
#endif

#include <Pixy2SPI_SS.h>

Pixy2SPI_SS pixy;

#define SAMPLE_COUNT    30        // samples averaged per position (more = more stable)

// Position order: TOP_LEFT=0, BOT_LEFT=1, TOP_RIGHT=2, BOT_RIGHT=3
const char* LABELS[4]  = { "TOP_LEFT", "BOT_LEFT", "TOP_RIGHT", "BOT_RIGHT" };
int rec_cx[4] = {-1, -1, -1, -1};
int rec_cy[4] = {-1, -1, -1, -1};
int step = 0;
bool done = false;

// ── Helpers ──────────────────────────────────────────────────

// Returns the largest sig2 blob that passes area & ROI filters
bool readBestSig2(int &cx, int &cy) {
  pixy.ccc.getBlocks();
  uint32_t bestArea = 0;
  cx = -1; cy = -1;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature != 2) continue;
    if (b.m_y < ROI_TOP_Y) continue;           // ROI filter
    uint32_t area = (uint32_t)b.m_width * b.m_height;
    if (area < SIG2_MIN_AREA || area <= bestArea) continue;
    bestArea = area;
    cx = b.m_x;
    cy = b.m_y;
  }
  return (cx >= 0);
}

// Collect SAMPLE_COUNT valid readings and return their average
bool samplePosition(int &avg_cx, int &avg_cy) {
  long sumX = 0, sumY = 0;
  int count = 0;
  Serial.print("  Sampling");
  unsigned long giveUp = millis() + 8000;   // 8 s timeout
  while (count < SAMPLE_COUNT) {
    if (millis() > giveUp) {
      Serial.println();
      Serial.println("  TIMEOUT — purple not visible long enough. Try again.");
      return false;
    }
    int cx, cy;
    if (readBestSig2(cx, cy)) {
      sumX += cx; sumY += cy;
      count++;
      if (count % 5 == 0) Serial.print(".");   // dot every 5 samples
    }
    delay(40);
  }
  Serial.println(" done");
  avg_cx = (int)(sumX / count);
  avg_cy = (int)(sumY / count);
  return true;
}

void printResults() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("  CALIBRATION RESULTS — " ROBOT_NAME);
  Serial.println("======================================");
  for (int i = 0; i < 4; i++) {
    Serial.print("  "); Serial.print(LABELS[i]);
    Serial.print("  cx="); Serial.print(rec_cx[i]);
    Serial.print("  cy="); Serial.println(rec_cy[i]);
  }

  // Indices: TOP_LEFT=0, BOT_LEFT=1, TOP_RIGHT=2, BOT_RIGHT=3
  int left_cx_avg  = (rec_cx[0] + rec_cx[1]) / 2;  // TL + BL
  int right_cx_avg = (rec_cx[2] + rec_cx[3]) / 2;  // TR + BR
  int top_cy_avg   = (rec_cy[0] + rec_cy[2]) / 2;  // TL + TR
  int bot_cy_avg   = (rec_cy[1] + rec_cy[3]) / 2;  // BL + BR

  int SPLIT_X = (left_cx_avg + right_cx_avg) / 2;
  int SPLIT_Y = (top_cy_avg  + bot_cy_avg)   / 2;

  // DEAD zones: ~1/3 of the half-gap, minimum 3 pixels
  int raw_dead_x = (right_cx_avg - left_cx_avg) / 2;
  int raw_dead_y = abs(bot_cy_avg - top_cy_avg) / 2;
  int DEAD_X = max(3, raw_dead_x / 3);
  int DEAD_Y = max(3, raw_dead_y / 3);

  Serial.println();
  Serial.println("  ── Computed values ──────────────────");
  Serial.print("  #define SPLIT_X   "); Serial.println(SPLIT_X);
  Serial.print("  #define SPLIT_Y   "); Serial.println(SPLIT_Y);
  Serial.print("  #define DEAD_X    "); Serial.println(DEAD_X);
  Serial.print("  #define DEAD_Y    "); Serial.println(DEAD_Y);
  Serial.println("  ─────────────────────────────────────");
  Serial.println();

  // Quick sanity check
  bool ok = true;
  if (left_cx_avg >= right_cx_avg) {
    Serial.println("  WARNING: left avg cx >= right avg cx — check corner positions!");
    ok = false;
  }
  if (bot_cy_avg <= top_cy_avg) {
    Serial.println("  WARNING: bot avg cy <= top avg cy — Pixy Y increases downward, check positions!");
    ok = false;
  }
  if (ok) Serial.println("  Sanity check PASSED.");

  Serial.println();
  Serial.println("  Copy the 4 #define lines into your main code.");
  Serial.println("  Reset board to run calibration again.");
  Serial.println("======================================");
}

// ── Setup / Loop ─────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(600);
  pixy.init();
  pixy.changeProg("color_connected_components");
  delay(300);

  Serial.println();
  Serial.println("======================================");
  Serial.print  ("  PIXY2 ZONE CALIBRATION — ");
  Serial.println(ROBOT_NAME);
  Serial.println("======================================");
  Serial.println("  ROI_TOP_Y filter : " + String(ROI_TOP_Y));
  Serial.println("  SIG2 min area    : " + String(SIG2_MIN_AREA));
  Serial.println("  Samples/position : " + String(SAMPLE_COUNT));
  Serial.println();
  Serial.println("  Place the robot at each corner when asked,");
  Serial.println("  hold it still, then press ENTER to record.");
  Serial.println();
  Serial.print("  Step 1/4 — Place bot at TOP_LEFT, then press ENTER...");
}

void loop() {
  if (done) return;

  // Live cx/cy feed so you can see if Pixy is tracking
  int cx, cy;
  if (readBestSig2(cx, cy)) {
    Serial.print("\r  sig2: cx="); Serial.print(cx);
    Serial.print("  cy="); Serial.print(cy);
    Serial.print("   ");
  } else {
    Serial.print("\r  sig2: (not detected)                ");
  }

  // Wait for ENTER
  if (Serial.available() > 0) {
    while (Serial.available()) Serial.read();   // flush input
    Serial.println();

    int avg_cx, avg_cy;
    Serial.print("  Recording "); Serial.print(LABELS[step]); Serial.println("...");

    if (!samplePosition(avg_cx, avg_cy)) {
      // samplePosition already printed the error
      Serial.println();
      Serial.print("  Step "); Serial.print(step + 1); Serial.print("/4");
      Serial.print(" — Re-place bot at "); Serial.print(LABELS[step]);
      Serial.print(", then press ENTER...");
      return;
    }

    rec_cx[step] = avg_cx;
    rec_cy[step] = avg_cy;
    Serial.print("  Saved: cx="); Serial.print(avg_cx);
    Serial.print("  cy="); Serial.println(avg_cy);

    step++;
    if (step >= 4) {
      printResults();
      done = true;
    } else {
      Serial.println();
      Serial.print("  Step "); Serial.print(step + 1); Serial.print("/4");
      Serial.print(" — Place bot at "); Serial.print(LABELS[step]);
      Serial.print(", then press ENTER...");
    }
  }

  delay(40);
}