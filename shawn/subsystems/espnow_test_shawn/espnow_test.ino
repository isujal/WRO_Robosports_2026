// ============================================================
//  ESP-NOW TEST — SHAWN
//  Receives "Hello from Olivia"
//  Sends back "Roger that, Shawn here" as acknowledgement
//
//  MAC Address format:
//  uint8_t partnerMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
//  Get MAC by running this in setup: Serial.println(WiFi.macAddress());
// ============================================================

#include <WiFi.h>
#include <esp_now.h>

// ── Olivia's MAC address — fill this in ─────────────────────
uint8_t oliviaMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ── Message structs — must match Olivia exactly ──────────────
typedef struct {
  char message[32];
  int  messageID;
} OliviaPacket;

typedef struct {
  char message[32];
  int  messageID;
} ShawnPacket;

OliviaPacket incoming;
ShawnPacket  outgoing;

int replyCount = 0;

// ── Send callback — did Olivia receive reply? ────────────────
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("SHAWN → OLIVIA | Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✓ DELIVERED" : "✗ FAILED");
}

// ── Receive callback — got message from Olivia ───────────────
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));

  Serial.println("─────────────────────────────────");
  Serial.println("SHAWN ← OLIVIA | Message received:");
  Serial.print("  Text : "); Serial.println(incoming.message);
  Serial.print("  MsgID: "); Serial.println(incoming.messageID);
  Serial.println("─────────────────────────────────");

  // Send reply immediately on receive
  replyCount++;
  snprintf(outgoing.message, sizeof(outgoing.message), "Roger that, Shawn here");
  outgoing.messageID = replyCount;

  esp_err_t result = esp_now_send(oliviaMAC, (uint8_t *)&outgoing, sizeof(outgoing));

  Serial.println("═════════════════════════════════");
  Serial.print("SHAWN → OLIVIA | Replying msg #"); Serial.println(replyCount);
  Serial.print("  Text : "); Serial.println(outgoing.message);
  Serial.print("  Send call: ");
  Serial.println(result == ESP_OK ? "OK" : "ERROR");
  Serial.println("═════════════════════════════════");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Print own MAC so you can copy it into Olivia's code
  WiFi.mode(WIFI_STA);
  Serial.print("SHAWN MAC: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    while (true) delay(1000);
  }

  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Register Olivia as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, oliviaMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Olivia as peer");
    while (true) delay(1000);
  }

  Serial.println("ESP-NOW ready. Shawn is listening...");
}

void loop() {
  // Shawn only replies on receive, nothing to do here
  delay(100);
}