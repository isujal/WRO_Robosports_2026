  // ============================================================
  // BNO085 IMU — Get Yaw, Pitch, Roll on ESP32-S3 (I2C)
  // Library: Adafruit BNO08x
  // ============================================================

  #include <Wire.h>
  #include <Adafruit_BNO08x.h>

  // --- ESP32-S3 Pin Definitions ---
  #define BNO08X_SDA  18   // Default ESP32-S3 Hardware SDA
  #define BNO08X_SCL  17   // Default ESP32-S3 Hardware SCL
  #define BNO08X_RST  12  // Required for hardware reset

  // --- Struct defined GLOBALLY before any function ---
  struct Euler {
    float yaw, pitch, roll;
  };

  // Initialize BNO08x object with the Reset Pin
  Adafruit_BNO08x  bno(BNO08X_RST);
  sh2_SensorValue_t imuData;

  // ============================================================
  Euler quaternionToEuler(float qw, float qx, float qy, float qz) {
    Euler e;
    e.yaw   = atan2(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
    e.pitch = asin(2.0f*(qw*qy - qz*qx));
    e.roll  = atan2(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
    return e;
  }

  // ============================================================
  void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial Monitor on native USB

    // Initialize I2C with the custom ESP32-S3 pins
    Wire.begin(BNO08X_SDA, BNO08X_SCL);

    // FIXED: Uses the correct default hexadecimal address (0x4A) and standard Wire pointer
    if (!bno.begin_I2C(0x4A, &Wire)) {
      Serial.println("BNO085 not found! Check wiring or try address 0x4B.");
      while (1) { delay(10); }
    }

    // Enable the rotation vector report
    bno.enableReport(SH2_ROTATION_VECTOR, 10000);
    
    Serial.println("BNO085 Ready");
    Serial.println("Yaw\t\tPitch\t\tRoll");
  }

  // ============================================================
  void loop() {
    // Handle any background system reset signals from the IMU
    if (bno.wasReset()) {
      bno.enableReport(SH2_ROTATION_VECTOR, 10000);
    }

    if (bno.getSensorEvent(&imuData)) {
      if (imuData.sensorId == SH2_ROTATION_VECTOR) {

        float qw = imuData.un.rotationVector.real;
        float qx = imuData.un.rotationVector.i;
        float qy = imuData.un.rotationVector.j;
        float qz = imuData.un.rotationVector.k;

        Euler e = quaternionToEuler(qw, qx, qy, qz);

        Serial.print(degrees(e.yaw),   2);  Serial.print("\t\t");
        Serial.print(degrees(e.pitch), 2);  Serial.print("\t\t");
        Serial.println(degrees(e.roll), 2);
      }
    }
  }


















  // // ============================================================
  // // BNO085 IMU — Get Yaw, Pitch, Roll on Arduino Mega (I2C)
  // // Library: Adafruit BNO08x
  // // ============================================================

  // #include <Adafruit_BNO08x.h>

  // // --- Struct defined GLOBALLY before any function ---
  // struct Euler {
  //   float yaw, pitch, roll;
  // };

  // Adafruit_BNO08x  bno;
  // sh2_SensorValue_t imuData;

  // // ============================================================
  // Euler quaternionToEuler(float qw, float qx, float qy, float qz) {
  //   Euler e;
  //   e.yaw   = atan2(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
  //   e.pitch = asin(2.0f*(qw*qy - qz*qx));
  //   e.roll  = atan2(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
  //   return e;
  // }

  // // ============================================================
  // void setup() {
  //   Serial.begin(115200);
  //   Wire.begin();

  //   if (!bno.begin_I2C()) {
  //     Serial.println("BNO085 not found! Check wiring.");
  //     while (1);
  //   }

  //   bno.enableReport(SH2_ROTATION_VECTOR, 10000);
  //   Serial.println("BNO085 Ready");
  //   Serial.println("Yaw\t\tPitch\t\tRoll");
  // }

  // // ============================================================
  // void loop() {
  //   if (bno.getSensorEvent(&imuData)) {
  //     if (imuData.sensorId == SH2_ROTATION_VECTOR) {

  //       float qw = imuData.un.rotationVector.real;
  //       float qx = imuData.un.rotationVector.i;
  //       float qy = imuData.un.rotationVector.j;
  //       float qz = imuData.un.rotationVector.k;

  //       Euler e = quaternionToEuler(qw, qx, qy, qz);

  //       Serial.print(degrees(e.yaw),   2);  Serial.print("\t\t");
  //       Serial.print(degrees(e.pitch), 2);  Serial.print("\t\t");
  //       Serial.println(degrees(e.roll), 2);
  //     }
  //   }
  // }
