#include <Pixy2SPI_SS.h>
#include <Adafruit_NeoPixel.h>

// ── Pixy settings ────────────────────────────────────────────
#define SIG_PURPLE   2
#define MIN_AREA     200
#define ROI_TOP_Y    55
#define SPLIT_X      210
#define SPLIT_Y      81
#define DEAD_X       5
#define DEAD_Y       5

// ── Hardware ─────────────────────────────────────────────────
#define LED_PIN      48
#define TOGGLE_PIN   20

Pixy2SPI_SS pixy;
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── Zone enum ────────────────────────────────────────────────
enum Zone { UNKNOWN, TOP_LEFT, BOT_LEFT, TOP_RIGHT, BOT_RIGHT };

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

Zone scanPixy() {
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
    if (area > bestArea) {
      bestArea = area;
      bestCx   = b.m_x;
      bestCy   = b.m_y;
      found    = true;
    }
  }

  if (!found) return UNKNOWN;
  return classifyZone(bestCx, bestCy);
}

void setLedForZone(Zone z) {
  switch (z) {
    case BOT_LEFT:  led.setPixelColor(0, led.Color(255, 0,   0));   break; // RED
    case TOP_LEFT:  led.setPixelColor(0, led.Color(148, 0,   211)); break; // PURPLE
    case BOT_RIGHT: led.setPixelColor(0, led.Color(0,   255, 0));   break; // GREEN
    case TOP_RIGHT: led.setPixelColor(0, led.Color(255, 80,  0));   break; // ORANGE
    default:        led.setPixelColor(0, led.Color(255, 255, 255)); break; // WHITE
  }
  led.show();
}

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(TOGGLE_PIN, INPUT_PULLDOWN);

  led.begin();
  led.setBrightness(80);
  led.setPixelColor(0, led.Color(255, 255, 255));
  led.show();

  pixy.init();
  pixy.changeProg("color_connected_components");
  delay(200);
}

// ============================================================
void loop() {

  // Scan pixy and update LED
  Zone z = scanPixy();
  setLedForZone(z);

  // Check toggle
  if (digitalRead(TOGGLE_PIN) == HIGH) {
    Serial.println("BOT ON");
  }

  delay(100);
}