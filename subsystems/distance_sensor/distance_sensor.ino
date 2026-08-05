// ============================================================
//  Dual I2C Test — BNO085 + VL53L0X — ESP32-S3
//
//  I2C Bus 0 (Wire)  → BNO085 IMU
//    SDA = GPIO 18, SCL = GPIO 17
//
//  I2C Bus 1 (Wire1) → VL53L0X Distance Sensor
//    SDA = GPIO 13, SCL = GPIO 14
//
//  Distance output in CM
//  Heading zeroed at startup
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>

// -------- IMU I2C (Bus 0) --------
#define BNO08X_SDA 18
#define BNO08X_SCL 17
#define BNO08X_RST 12

// -------- VL53L0X I2C (Bus 1) --------
#define TOF_SDA 13
#define TOF_SCL 14

// -------- Objects --------
Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;
Adafruit_VL53L0X  lox;

float yawOffset = 0.0;

// -------- Function Prototypes --------
float getRawYaw();
float getHeading();
float wrapAngle(float a);
void  zeroIMU();

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // ---- Init IMU I2C (Bus 0 — Wire) ----
  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not found!");
    while (1) delay(10);
  }
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  delay(200);
  Serial.println("BNO085 Ready.");

  // ---- Init VL53L0X I2C (Bus 1 — Wire1) ----
  Wire1.begin(TOF_SDA, TOF_SCL);
  if (!lox.begin(0x29, false, &Wire1)) {
    Serial.println("VL53L0X not found! Check wiring.");
    while (1) delay(10);
  }
  Serial.println("VL53L0X Ready.");

  // ---- Zero IMU at startup ----
  zeroIMU();

  Serial.println("Heading(deg) | Distance(cm)");
}

// ============================================================
void loop() {
  // ---- Read IMU heading (zeroed) ----
  float heading = getHeading();

  // ---- Read VL53L0X distance ----
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  // ---- Print both ----
  Serial.print("Heading: "); Serial.print(heading, 1);
  Serial.print("deg  |  Distance: ");
  if (measure.RangeStatus != 4) {
    float distanceCm = measure.RangeMilliMeter / 10.0;  // mm → cm
    Serial.print(distanceCm, 1);
    Serial.println(" cm");
  } else {
    Serial.println("Out of range");
  }
}

// ============================================================
//  getRawYaw()
//  Negated so CW = positive. Blocks until fresh data.
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
        return -degrees(atan2(2.0f*(qw*qz + qx*qy),
                              1.0f - 2.0f*(qy*qy + qz*qz)));
      }
    }
  }
}

// ============================================================
//  getHeading()
//  Returns heading relative to startup zero. CW positive.
// ============================================================
float getHeading() {
  return wrapAngle(getRawYaw() - yawOffset);
}

// ============================================================
//  zeroIMU()
//  Captures current yaw as offset — heading reads 0 from here.
// ============================================================
void zeroIMU() {
  for (int i = 0; i < 20; i++) {
    bno.getSensorEvent(&imuData);
    delay(10);
  }
  yawOffset = getRawYaw();
  Serial.print("IMU Zeroed at: ");
  Serial.print(yawOffset, 2);
  Serial.println("deg");
}

// ============================================================
float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}