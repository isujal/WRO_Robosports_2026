// ============================================================
//  VISION + ESP-NOW — ZONE BROADCASTER
//  Same code flashed on both Olivia and Shawn
//
//  STEP 1: Flash with zero MAC → note down MY MAC from Serial
//  STEP 2: Fill partnerMAC[] with partner's MAC → flash again
//
//  MAC format: {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
// ============================================================

#include <WiFi.h>
#include <esp_now.h>
#include <Pixy2SPI_SS.h>

// ============================================================
//  ★ FILL THIS IN WITH YOUR PARTNER'S MAC ★
// ============================================================
uint8_t partnerMAC[] = {0x14, 0xC1, 0x9F, 0x2D, 0x13, 0xC4}; // shawn
// uint8_t partnerMAC[] = {0x14, 0xC1, 0x9F, 0x2E, 0xC6, 0x80}; // olivia 

// 14:c1:9f:2d:13:c4

// ============================================================
//  ZONE ENUM — must be declared before anything uses it
// ============================================================
enum Zone : uint8_t {
  UNKNOWN   = 0,
  TOP_LEFT  = 1,
  BOT_LEFT  = 2,
  TOP_RIGHT = 3,
  BOT_RIGHT = 4
};

// ============================================================
//  DATA PACKET — same struct on both bots
// ============================================================
typedef struct {
  uint8_t zone;
  bool    sawPurple;
} ZonePacket;

ZonePacket myPacket;
ZonePacket partnerPacket;

// ============================================================
//  PIXY SETTINGS
// ============================================================
#define SIG_PURPLE  2
#define MIN_AREA    200
#define ROI_TOP_Y   55
#define SPLIT_X     210
#define SPLIT_Y     81
#define DEAD_X      5
#define DEAD_Y      5

Pixy2SPI_SS pixy;

// ============================================================
//  HELPERS
// ============================================================
const char* zoneName(Zone z) {
  switch (z) {
    case TOP_LEFT:  return "TOP-LEFT";
    case BOT_LEFT:  return "BOT-LEFT";
    case TOP_RIGHT: return "TOP-RIGHT";
    case BOT_RIGHT: return "BOT-RIGHT";
    default:        return "UNKNOWN";
  }
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

// ============================================================
//  ESP-NOW CALLBACKS
// ============================================================

// SDK 3.x send callback signature
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // silent — prints handled in loop
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&partnerPacket, data, sizeof(partnerPacket));
  Serial.println("──────────────────────────────");
  Serial.println("RECEIVED FROM PARTNER:");
  Serial.print("  sawPurple : "); Serial.println(partnerPacket.sawPurple ? "YES" : "NO");
  Serial.print("  zone      : "); Serial.println(zoneName((Zone)partnerPacket.zone));
  Serial.println("──────────────────────────────");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Print own MAC — note this down
  WiFi.mode(WIFI_STA);
  Serial.println("══════════════════════════════");
  Serial.print("MY MAC: "); Serial.println(WiFi.macAddress());
  Serial.println("══════════════════════════════");

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW FAILED");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Register partner as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, partnerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (true) delay(1000);
  }

  // Pixy init
  pixy.init();
  pixy.changeProg("color_connected_components");
  delay(200);

  Serial.println("Ready — scanning and broadcasting...\n");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

  Zone z = scanPixy();

  myPacket.zone      = (uint8_t)z;
  myPacket.sawPurple = (z != UNKNOWN);

  // Send to partner
  esp_now_send(partnerMAC, (uint8_t *)&myPacket, sizeof(myPacket));

  // Print what I see
  Serial.print("ME → zone: ");      Serial.print(zoneName(z));
  Serial.print("  sawPurple: ");    Serial.println(myPacket.sawPurple ? "YES" : "NO");

  delay(200);
}