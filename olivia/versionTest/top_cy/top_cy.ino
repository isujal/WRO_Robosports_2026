#include <Pixy2SPI_SS.h>

Pixy2SPI_SS pixy;

#define SIG3_MIN_AREA    500
#define SIG3_EMA_ALPHA   0.7f

float ema_top_cy = -1.0f;
bool  ema_init   = false;

void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.changeProg("color_connected_components");
  Serial.println("Ready. Watching sig3 top_cy...");
}

void loop() {
  pixy.ccc.getBlocks();

  int      best_raw = -1;
  uint32_t bestArea = 0;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature != 3) continue;
    uint32_t area = (uint32_t)b.m_width * b.m_height;
    if (area < SIG3_MIN_AREA) continue;
    if (area > bestArea) {
      bestArea  = area;
      best_raw  = b.m_y - b.m_height / 2;
    }
  }

  if (best_raw >= 0) {
    if (!ema_init) {
      ema_top_cy = (float)best_raw;
      ema_init   = true;
    } else {
      if (abs(best_raw - ema_top_cy) < 30) {
        ema_top_cy = SIG3_EMA_ALPHA * best_raw + (1.0f - SIG3_EMA_ALPHA) * ema_top_cy;
      } else {
        Serial.println("outlier rejected");
      }
    }
    Serial.print("raw="); Serial.print(best_raw);
    Serial.print("  ema="); Serial.println((int)ema_top_cy);
  } else {
    Serial.println("sig3 not detected");
  }

  delay(50);
}