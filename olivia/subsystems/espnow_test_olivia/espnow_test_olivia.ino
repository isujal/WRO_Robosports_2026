// ============================================================
//  ESP-NOW TEST — OLIVIA
//  Sends "Hello from Olivia" to Shawn every 2 seconds
//  Receives acknowledgement from Shawn
//
//  MAC Address format:
//  uint8_t partnerMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
//  Get MAC by running this in setup: Serial.println(WiFi.macAddress());
// ============================================================

#include <WiFi.h>
#include <esp_now.h>

// ── Shawn's MAC address — fill this in ──────────────────────
uint8_t shawnMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ── Message structs ──────────────────────────────────────────
typedef struct {
  char message[32];
  int  messageID;
} OliviaPacket;

typedef struct {
  char message[32];
  int  messageID;
} ShawnPacket;

OliviaPacket outgoing;
ShawnPacket  incoming;

int msgCount = 0;

// ── Send callback — did Shawn receive it? ────────────────────
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("OLIVIA → SHAWN | Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✓ DELIVERED" : "✗ FAILED");
}

// ── Receive callback — message from Shawn ───────────────────
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  Serial.println("─────────────────────────────────");
  Serial.println("OLIVIA ← SHAWN | Message received:");
  Serial.print("  Text : "); Serial.println(incoming.message);
  Serial.print("  MsgID: "); Serial.println(incoming.messageID);
  Serial.println("─────────────────────────────────");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Print own MAC so you can copy it into Shawn's code
  WiFi.mode(WIFI_STA);
  Serial.print("OLIVIA MAC: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    while (true) delay(1000);
  }

  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Register Shawn as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, shawnMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Shawn as peer");
    while (true) delay(1000);
  }

  Serial.println("ESP-NOW ready. Olivia will send every 2 seconds.");
}

void loop() {
  // Send message to Shawn every 2 seconds
  msgCount++;
  snprintf(outgoing.message, sizeof(outgoing.message), "Hello from Olivia");
  outgoing.messageID = msgCount;

  esp_err_t result = esp_now_send(shawnMAC, (uint8_t *)&outgoing, sizeof(outgoing));

  Serial.println("═════════════════════════════════");
  Serial.print("OLIVIA → SHAWN | Sending msg #"); Serial.println(msgCount);
  Serial.print("  Text : "); Serial.println(outgoing.message);
  Serial.print("  Send call: ");
  Serial.println(result == ESP_OK ? "OK" : "ERROR");
  Serial.println("═════════════════════════════════");

  delay(2000);
}