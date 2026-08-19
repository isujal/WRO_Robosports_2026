#include <SPI.h>
#include <Pixy2SPI_SS.h>

#define SIG_PURPLE  2
#define MIN_AREA    100
#define ROI_TOP_Y   40

// ── Tuned to SHAWN mat readings (Pixy 2.1) ─────────────────
// TOP-LEFT  cx~147  cy~60
// BOT-LEFT  cx~146  cy~83
// TOP-RIGHT cx~240  cy~66
// BOT-RIGHT cx~262  cy~88
// Midpoint X = (146 + 251) / 2 = 200
// Midpoint Y = (63  + 85)  / 2 = 74
#define SPLIT_X     200
#define SPLIT_Y     77
#define DEAD_X      10
#define DEAD_Y      3

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

// Zone classifyZone(int cx, int cy) {
//   bool isLeft  = cx < (SPLIT_X - DEAD_X);
//   bool isRight = cx > (SPLIT_X + DEAD_X);

//   // Left side top/bot threshold
//   bool isTopLeft  = cy < 76;
//   bool isBotLeft  = cy > 80;

//   // Right side top/bot threshold — right sits higher in frame
//   bool isTopRight = cy < 80;
//   bool isBotRight = cy > 83;

//   if (isLeft  && isTopLeft)  return TOP_LEFT;
//   if (isLeft  && isBotLeft)  return BOT_LEFT;
//   if (isRight && isTopRight) return TOP_RIGHT;
//   if (isRight && isBotRight) return BOT_RIGHT;
//   return UNKNOWN;
// }

Zone classifyZone(int cx, int cy) {

  // ── LEFT SIDE (cx < 200) ─────────────────────────────────
  if (cx < 200) {
    if (cy < 78)  return TOP_LEFT;   // cy 60-72 → TOP-LEFT
    if (cy >= 78) return BOT_LEFT;   // cy 83-88 → BOT-LEFT
  }

  // ── RIGHT SIDE (cx >= 200) ───────────────────────────────
  if (cx >= 200) {
    if (cy < 82)  return TOP_RIGHT;  // cy 60-76 → TOP-RIGHT
    if (cy >= 82) return BOT_RIGHT;  // cy 87-89 → BOT-RIGHT
  }

  return UNKNOWN;
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  SPI.begin(12, 13, 11, 10);   // SCK, MISO, MOSI, CS
  pixy.init();
  pixy.changeProg("color_connected_components");

  Serial.println("═══════════════════════════════════════");
  Serial.println("  Purple Zone Tracker — SHAWN Pixy 2.1");
  Serial.println("═══════════════════════════════════════");
  Serial.printf("  SPLIT_X=%d  (left<%-3d  right>%d)\n", SPLIT_X, SPLIT_X-DEAD_X, SPLIT_X+DEAD_X);
  Serial.printf("  SPLIT_Y=%d   (top <%-3d  bot  >%d)\n", SPLIT_Y, SPLIT_Y-DEAD_Y, SPLIT_Y+DEAD_Y);
  Serial.println("  Expected ranges:");
  Serial.println("  TL: cx~147  cy~60");
  Serial.println("  TR: cx~240  cy~66");
  Serial.println("  BL: cx~146  cy~83");
  Serial.println("  BR: cx~262  cy~88");
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
    Serial.printf("cx=%-3d  cy=%-3d  isTop=%d  isBot=%d  isLeft=%d  isRight=%d  zone=%s\n",
                  bestCx, bestCy,
                  bestCy < (SPLIT_Y - DEAD_Y),
                  bestCy > (SPLIT_Y + DEAD_Y),
                  bestCx < (SPLIT_X - DEAD_X),
                  bestCx > (SPLIT_X + DEAD_X),
                  zoneName(z));
  } else {
    Serial.println("PURPLE → NOT DETECTED");
  }

  delay(150);
}