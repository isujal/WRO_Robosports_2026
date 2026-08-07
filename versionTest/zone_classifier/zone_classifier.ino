#include <Pixy2SPI_SS.h>

#define SIG_PURPLE  2
#define MIN_AREA    200
#define ROI_TOP_Y   55

// ── Tuned to YOUR mat readings ─────────────────────────────
// Left  cx: ~150-158   Right cx: ~263-302
// Top   cy: ~68        Bottom cy: ~93-96
// Midpoint X = (158+263)/2 = 210
// Midpoint Y = (68+95)/2   = 81
#define SPLIT_X     210
#define SPLIT_Y     81
#define DEAD_X      5
#define DEAD_Y      5

#define FOCAL_PX    180.0f
#define PURPLE_DIAM 5.0f

enum Zone { UNKNOWN, TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT };

Pixy2SPI_SS pixy;

const char* zoneName(Zone z) {
  switch(z) {
    case TOP_LEFT:  return "TOP-LEFT";
    case TOP_RIGHT: return "TOP-RIGHT";
    case BOT_LEFT:  return "BOT-LEFT";
    case BOT_RIGHT: return "BOT-RIGHT";
    default:        return "UNKNOWN";
  }
}

Zone classifyZone(int cx, int cy) {
  bool isLeft  = cx < (SPLIT_X - DEAD_X);   // cx < 205
  bool isRight = cx > (SPLIT_X + DEAD_X);   // cx > 215
  bool isTop   = cy < (SPLIT_Y - DEAD_Y);   // cy < 76
  bool isBot   = cy > (SPLIT_Y + DEAD_Y);   // cy > 86

  if (isTop && isLeft)  return TOP_LEFT;
  if (isTop && isRight) return TOP_RIGHT;
  if (isBot && isLeft)  return BOT_LEFT;
  if (isBot && isRight) return BOT_RIGHT;
  return UNKNOWN;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pixy.init();
  pixy.changeProg("color_connected_components");

  Serial.println("═══════════════════════════════════════");
  Serial.println("  Purple Zone Tracker — tuned splits");
  Serial.println("═══════════════════════════════════════");
  Serial.printf("  SPLIT_X=%d  (left<%-3d  right>%d)\n", SPLIT_X, SPLIT_X-DEAD_X, SPLIT_X+DEAD_X);
  Serial.printf("  SPLIT_Y=%d   (top <%-3d  bot  >%d)\n", SPLIT_Y, SPLIT_Y-DEAD_Y, SPLIT_Y+DEAD_Y);
  Serial.println("  Expected ranges:");
  Serial.println("  TL: cx~150-158  cy~68");
  Serial.println("  TR: cx~263-302  cy~68");
  Serial.println("  BL: cx~150-158  cy~93-96");
  Serial.println("  BR: cx~263-302  cy~93-96");
  Serial.println("═══════════════════════════════════════\n");
}

void loop() {
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
    Serial.printf("PURPLE → cx=%-3d  cy=%-3d  zone=%-12s",
                  bestCx, bestCy, zoneName(z));

    // Extra debug — show which side of each split it fell on
    if (z == UNKNOWN) {
      Serial.printf("  [cx%s%d  cy%s%d]",
        bestCx < SPLIT_X ? "<" : ">", SPLIT_X,
        bestCy < SPLIT_Y ? "<" : ">", SPLIT_Y);
    }
    Serial.println();
  } else {
    Serial.println("PURPLE → NOT DETECTED");
  }

  // delay(150);
}