#include <Pixy2SPI_SS.h>
#include <PIDLoop.h>

Pixy2SPI_SS pixy;
PIDLoop panLoop(400, 0, 400, true);
PIDLoop tiltLoop(500, 0, 500, true);

void setup()
{
  Serial.begin(115200);
  delay(1000); // give Pixy2 time to fully boot before init
  Serial.println("\n==============================");
  Serial.println("  Pixy2 Init Verification");
  Serial.println("==============================");

  int result = pixy.init();
  Serial.printf("pixy.init() returned: %d\n", result); // 0 = success

  if (result == 0) {
    Serial.println("✅ Pixy2 connected!");

    // Print firmware version
    Serial.printf("   Hardware ver : %d.%d\n",
      pixy.version->hardware >> 8,
      pixy.version->hardware & 0xff);
    Serial.printf("   Firmware ver : %d.%d.%d\n",
      pixy.version->firmwareMajor,
      pixy.version->firmwareMinor,
      pixy.version->firmwareBuild);
    Serial.printf("   Firmware type: %s\n", pixy.version->firmwareType);
    Serial.printf("   Frame size   : %d x %d\n",
      pixy.frameWidth, pixy.frameHeight);

    Serial.println("\nSwitching to color_connected_components...");
    pixy.changeProg("color_connected_components");
    Serial.println("✅ Program changed. Ready to detect objects.");

  } else {
    Serial.println("❌ Pixy2 init failed!");
    Serial.println("   Checklist:");
    Serial.println("   • Patched Pixy2SPI_SS.h saved in libraries folder?");
    Serial.println("   • Arduino IDE Clean Build done? (Sketch → Clean Build Folder)");
    Serial.println("   • Pixy2 powered and SPI with SS set in PixyMon?");
  }
}

void loop()
{
  if (pixy.ccc.getBlocks())
  {
    Serial.printf("Blocks found: %d\n", pixy.ccc.numBlocks);
    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
      Serial.printf("  [%d] sig=%d  x=%d  y=%d  w=%d  h=%d\n",
        i,
        pixy.ccc.blocks[i].m_signature,
        pixy.ccc.blocks[i].m_x,
        pixy.ccc.blocks[i].m_y,
        pixy.ccc.blocks[i].m_width,
        pixy.ccc.blocks[i].m_height);
    }
  }
  delay(200);
}