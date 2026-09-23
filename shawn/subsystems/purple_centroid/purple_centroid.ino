#include <Pixy2SPI_SS.h>

Pixy2SPI_SS pixy;

#define SIG_PURPLE  1
#define MIN_AREA    75
#define ROI_TOP_Y   55   // ← blobs with cy < this are ignored

void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.changeProg("color_connected_components");
  delay(500);
  Serial.println("=== PIXY CX/CY LOGGER — move ball to each zone ===");
}

void loop() {
  pixy.ccc.getBlocks();

  uint32_t bestArea = 0;
  int bestCx = 0, bestCy = 0;
  bool found = false;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature != SIG_PURPLE) continue;
    if (b.m_y < ROI_TOP_Y) continue;   // ← ignore blobs too high in frame
    uint32_t area = (uint32_t)b.m_width * b.m_height;
    if (area < MIN_AREA) continue;
    if (area > bestArea) {
      bestArea = area;
      bestCx = b.m_x;
      bestCy = b.m_y;
      found = true;
    }
  }

  if (found) {
    Serial.print("cx="); Serial.print(bestCx);
    Serial.print("  cy="); Serial.print(bestCy);
    Serial.print("  area="); Serial.println(bestArea);
  } else {
    Serial.println("NOT DETECTED");
  }

  // delay(100);
}