// ============================================================
//  Pixy2 Zone Calibration Tool
//  Usage: Open Serial Monitor at 115200 baud
//         Place bot at each position and press ENTER to record
// ============================================================

#include <Pixy2SPI_SS.h>

Pixy2SPI_SS pixy;

#define SIG2_MIN_AREA   100    // min purple blob area to accept
#define SAMPLE_COUNT    20     // samples averaged per position

// Recorded positions — order: BOT_LEFT, BOT_RIGHT, TOP_LEFT, TOP_RIGHT
const char* LABELS[4] = { "BOT_LEFT", "BOT_RIGHT", "TOP_LEFT", "TOP_RIGHT" };
int rec_cx[4] = {-1, -1, -1, -1};
int rec_cy[4] = {-1, -1, -1, -1};
int step = 0;   // which position we're waiting to record (0-3)
bool done = false;

// ── helpers ──────────────────────────────────────────────────

bool readBestSig2(int &cx, int &cy) {
  pixy.ccc.getBlocks();
  uint32_t bestArea = 0;
  cx = -1; cy = -1;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature != 2) continue;
    if (b.m_y < 40) continue;          // ← add this (ROI_TOP_Y filter)
    uint32_t area = (uint32_t)b.m_width * b.m_height;
    if (area < SIG2_MIN_AREA || area <= bestArea) continue;
    bestArea = area;
    cx = b.m_x;
    cy = b.m_y;
  }
  return (cx >= 0);
}

// Average SAMPLE_COUNT valid readings
bool samplePosition(int &avg_cx, int &avg_cy) {
  long sumX = 0, sumY = 0;
  int count = 0;
  Serial.print("  Sampling");
  while (count < SAMPLE_COUNT) {
    int cx, cy;
    if (readBestSig2(cx, cy)) {
      sumX += cx; sumY += cy;
      count++;
      Serial.print(".");
    }
    delay(50);
  }
  Serial.println();
  if (count == 0) return false;
  avg_cx = (int)(sumX / count);
  avg_cy = (int)(sumY / count);
  return true;
}

void printResults() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("  CALIBRATION RESULTS");
  Serial.println("======================================");
  for (int i = 0; i < 4; i++) {
    Serial.print("  "); Serial.print(LABELS[i]);
    Serial.print("  cx="); Serial.print(rec_cx[i]);
    Serial.print("  cy="); Serial.println(rec_cy[i]);
  }

  // SPLIT_X = midpoint between left and right cx averages
  int left_cx_avg  = (rec_cx[0] + rec_cx[2]) / 2;  // BL + TL
  int right_cx_avg = (rec_cx[1] + rec_cx[3]) / 2;  // BR + TR
  int SPLIT_X = (left_cx_avg + right_cx_avg) / 2;

  // SPLIT_Y = midpoint between top and bottom cy averages
  int top_cy_avg = (rec_cy[2] + rec_cy[3]) / 2;    // TL + TR
  int bot_cy_avg = (rec_cy[0] + rec_cy[1]) / 2;    // BL + BR
  int SPLIT_Y = (top_cy_avg + bot_cy_avg) / 2;

  // DEAD_X = half the gap between left and right, minus a small margin
  int raw_dead_x = (right_cx_avg - left_cx_avg) / 2;
  int DEAD_X = max(2, raw_dead_x / 4);  // conservative: 1/4 of half-gap

  // DEAD_Y = half the gap between top and bottom, minus a small margin
  int raw_dead_y = abs(bot_cy_avg - top_cy_avg) / 2;
  int DEAD_Y = max(2, raw_dead_y / 4);

  Serial.println();
  Serial.println("  ── Computed values ──────────────────");
  Serial.print("  #define SPLIT_X   "); Serial.println(SPLIT_X);
  Serial.print("  #define SPLIT_Y   "); Serial.println(SPLIT_Y);
  Serial.print("  #define DEAD_X    "); Serial.println(DEAD_X);
  Serial.print("  #define DEAD_Y    "); Serial.println(DEAD_Y);
  Serial.println("  ─────────────────────────────────────");
  Serial.println();
  Serial.println("  Copy these 4 lines into your main code.");
  Serial.println("  Reset board to calibrate again.");
  Serial.println("======================================");
}

// ── setup / loop ─────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  pixy.init();
  pixy.changeProg("color_connected_components");
  delay(300);

  Serial.println();
  Serial.println("======================================");
  Serial.println("  PIXY2 ZONE CALIBRATION");
  Serial.println("======================================");
  Serial.println("  Place bot at each position when asked,");
  Serial.println("  then press ENTER (send any key) to record.");
  Serial.println();
  Serial.print("  Step 1/4 — Place bot at BOT_LEFT, then press ENTER...");
}

void loop() {
  if (done) return;

  // ── live feed while waiting ──────────────────────────────
  int cx, cy;
  if (readBestSig2(cx, cy)) {
    Serial.print("\r  sig2: cx="); Serial.print(cx);
    Serial.print("  cy="); Serial.print(cy);
    Serial.print("   ");   // clear trailing chars
  } else {
    Serial.print("\r  sig2: (not detected)           ");
  }

  // ── check for ENTER keypress ──────────────────────────────
  if (Serial.available() > 0) {
    while (Serial.available()) Serial.read();  // flush
    Serial.println();  // newline after live feed line

    int avg_cx, avg_cy;
    Serial.print("  Recording "); Serial.print(LABELS[step]); Serial.println("...");
    if (!samplePosition(avg_cx, avg_cy)) {
      Serial.println("  ERROR: no sig2 detected! Make sure purple ball is visible. Try again.");
      Serial.print("  Press ENTER when ready...");
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