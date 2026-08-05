// ============================================================
//  Pixy2 Color Block Detection Test — ESP32-S3
//  Interface: SPI
//
//  Pixy2 Wiring:
//    MISO → GPIO 38
//    MOSI → GPIO 40
//    SCLK → GPIO 39
//    CS   → GPIO 41
//    5V   → 5V
//    GND  → GND
//
//  Prints detected block info to Serial Monitor:
//    Signature, X, Y, Width, Height, Age
//
//  Signature 1 = trained ping pong ball color
// ============================================================

#include <Pixy2.h>
#include <SPI.h>

// -------- Pixy2 SPI Pins --------
#define PIXY_CS   41
#define PIXY_MISO 38
#define PIXY_MOSI 40
#define PIXY_SCLK 39

Pixy2 pixy;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Init SPI with custom pins
  SPI.begin(PIXY_SCLK, PIXY_MISO, PIXY_MOSI, PIXY_CS);

  Serial.println("Initializing Pixy2...");
  pixy.init();
  Serial.println("Pixy2 Ready.");
  Serial.println("Waiting for blocks...");
  Serial.println("----------------------------");
}

void loop() {
  // Get color connected components (blocks)
  int numBlocks = pixy.ccc.getBlocks();

  if (numBlocks > 0) {
    Serial.print("Blocks found: ");
    Serial.println(numBlocks);

    for (int i = 0; i < numBlocks; i++) {
      Serial.print("  Block "); Serial.print(i + 1); Serial.print(": ");
      Serial.print("Sig=");    Serial.print(pixy.ccc.blocks[i].m_signature);
      Serial.print(" X=");     Serial.print(pixy.ccc.blocks[i].m_x);
      Serial.print(" Y=");     Serial.print(pixy.ccc.blocks[i].m_y);
      Serial.print(" W=");     Serial.print(pixy.ccc.blocks[i].m_width);
      Serial.print(" H=");     Serial.print(pixy.ccc.blocks[i].m_height);
      Serial.print(" Age=");   Serial.println(pixy.ccc.blocks[i].m_age);
    }

    // ---- Largest block (most likely the ball) ----
    // Pixy2 returns blocks sorted by size — index 0 is largest
    Serial.print("  >> Ball center: X=");
    Serial.print(pixy.ccc.blocks[0].m_x);
    Serial.print(", Y=");
    Serial.println(pixy.ccc.blocks[0].m_y);

    // ---- Distance estimate from block size ----
    // Larger block = closer ball. Rough estimate only.
    int area = pixy.ccc.blocks[0].m_width * pixy.ccc.blocks[0].m_height;
    Serial.print("  >> Block area: ");
    Serial.print(area);
    Serial.println(" px²");

  } else {
    Serial.println("No ball detected.");
  }

  Serial.println("----------------------------");
  delay(100);
}