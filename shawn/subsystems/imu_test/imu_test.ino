// ============================================================
//  IMU Diagnostic Test — Mirrors dist_based_rect_trajectory
//  Prints RAW yaw and HEADING (offset-corrected) continuously
//  BNO085 | ESP32-S3 | I2C Bus 0
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ── Pins ─────────────────────────────────────────────────────
#define BNO08X_SDA  17
#define BNO08X_SCL  16
#define BNO08X_RST  12

// ── IMU objects ──────────────────────────────────────────────
Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;

float yawOffset = 0.0;

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("==============================================");
  Serial.println("  IMU Diagnostic — RAW vs HEADING pipeline   ");
  Serial.println("==============================================");

  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  initIMU();
  zeroIMU();

  Serial.println();
  Serial.println("  RAW_YAW(°)  |  HEADING(°)");
  Serial.println("  -----------   -----------");
}

// ============================================================
void loop() {

  // ── IMU auto-reset watchdog ──────────────────────────────
  if (bno.wasReset()) {
    Serial.println("[RESET] BNO085 auto-reset — re-enabling & re-zeroing.");
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    delay(200);
    zeroIMU();
  }

  // ── Poll for fresh reading ───────────────────────────────
  if (bno.getSensorEvent(&imuData)) {
    if (imuData.sensorId == SH2_GAME_ROTATION_VECTOR) {

      float qw = imuData.un.gameRotationVector.real;
      float qx = imuData.un.gameRotationVector.i;
      float qy = imuData.un.gameRotationVector.j;
      float qz = imuData.un.gameRotationVector.k;

      // Same math as getRawYaw() in trajectory code
      float rawYaw = -degrees(atan2(2.0f * (qw * qz + qx * qy),
                                    1.0f - 2.0f * (qy * qy + qz * qz)));

      // Same as getHeading() in trajectory code
      float heading = wrapAngle(rawYaw - yawOffset);

      // Print both side by side
      Serial.print("  ");
      Serial.print(rawYaw, 2);
      Serial.print("\t\t");
      Serial.println(heading, 2);
    }
  }
}

// ============================================================
void initIMU() {
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("[ERROR] BNO085 not found!");
    while (1) delay(10);
  }
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  delay(200);
  Serial.println("[INIT] BNO085 ready.");
}

// ============================================================
//  zeroIMU() — flushes stale data then captures offset
//  Identical to dist_based_rect_trajectory setup()
// ============================================================
void zeroIMU() {
  for (int i = 0; i < 20; i++) {
    bno.getSensorEvent(&imuData);
    delay(10);
  }
  yawOffset = getRawYaw();
  Serial.print("[ZERO] Offset = ");
  Serial.print(yawOffset, 4);
  Serial.println(" deg  →  HEADING now reads 0.00");
}

// ============================================================
//  getRawYaw() — blocks until fresh event, same as trajectory
// ============================================================
float getRawYaw() {
  if (bno.wasReset()) bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  while (true) {
    if (bno.getSensorEvent(&imuData)) {
      if (imuData.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float qw = imuData.un.gameRotationVector.real;
        float qx = imuData.un.gameRotationVector.i;
        float qy = imuData.un.gameRotationVector.j;
        float qz = imuData.un.gameRotationVector.k;
        return -degrees(atan2(2.0f * (qw * qz + qx * qy),
                              1.0f - 2.0f * (qy * qy + qz * qz)));
      }
    }
  }
}

// ============================================================
float wrapAngle(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}