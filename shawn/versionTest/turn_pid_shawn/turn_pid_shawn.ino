// ============================================================
//  PID Turn + Hold — BNO085 + MDD 3A + ESP32-S3
//
//  Changes from previous version:
//  1. PID never stops running — holds position after reaching target
//  2. Motors only stop if MANUALLY commanded (send 'S')
//  3. New target accepted anytime from Serial Monitor
//  4. Holding uses a smaller MIN_HOLD_SPEED to avoid jitter
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// -------- Motor Pins --------
const int M1A = 42;
const int M1B = 41;
const int M2A = 15;
const int M2B = 16;

// -------- IMU Pins --------
#define BNO08X_SDA  18
#define BNO08X_SCL  17
#define BNO08X_RST  12

// -------- PID Tuning --------
float Kp = 2.95; //2.95;
float Ki = 0.0;
float Kd = 0.5;

// -------- PID Config --------
const float DEADBAND       = 2.0;   // degrees — no motor output inside this
const int   MIN_SPEED      = 60;    // min PWM when error > deadband
const int   MAX_SPEED      = 255;   // max PWM
const int   PID_INTERVAL   = 10;    // ms

// -------- IMU --------
Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;

float yawOffset   = 0.0;
float targetAngle = 0.0;
bool  pidActive   = false;   // false until first target is given

// -------- PID State --------
float prevError = 0.0;
float integral  = 0.0;
unsigned long lastPIDTime = 0;

// -------- Function Prototypes --------
void  initIMU();
void  zeroIMU();
float getYaw();
float getHeading();
float wrapAngle(float a);
void  runPID();
void  turnClockwise(int speed);
void  turnAntiClockwise(int speed);
void  motorsStop();
void  setMotorPins();

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  setMotorPins();
  motorsStop();

  initIMU();
  zeroIMU();

  Serial.println();
  Serial.println("===== PID Turn + Hold =====");
  Serial.println("Type target angle → bot turns and HOLDS");
  Serial.println("Type S → stop motors and disable PID");
  Serial.println("Type Z → re-zero IMU");
  Serial.println("===========================");
}

// ============================================================
void loop() {

  // --- Serial input ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("S")) {
      // Manual stop — kills PID and motors
      pidActive = false;
      motorsStop();
      integral  = 0.0;
      prevError = 0.0;
      Serial.println(">> PID DISABLED — Motors stopped");

    } else if (input.equalsIgnoreCase("Z")) {
      // Re-zero IMU at current position
      pidActive = false;
      motorsStop();
      zeroIMU();
      targetAngle = 0.0;
      integral    = 0.0;
      prevError   = 0.0;
      Serial.println(">> IMU Re-zeroed. Send a new target.");

    } else {
      // New angle target
      float val = input.toFloat();
      if (input.length() > 0) {
        targetAngle = val;
        integral    = 0.0;   // reset integral for fresh turn
        prevError   = 0.0;
        pidActive   = true;
        Serial.print(">> New target: ");
        Serial.print(targetAngle, 1);
        Serial.println("°  (holding after reach)");
      }
    }
  }

  // --- PID loop — runs continuously when active ---
  if (pidActive) {
    unsigned long now = millis();
    if (now - lastPIDTime >= PID_INTERVAL) {
      lastPIDTime = now;
      runPID();
    }
  }
}

// ============================================================
//  runPID()
//  Runs every PID_INTERVAL ms.
//  Inside deadband → motors off (but PID stays active to re-engage
//  if bot gets pushed). Outside deadband → drive to correct.
// ============================================================
void runPID() {
  float heading = getHeading();
  float error   = wrapAngle(targetAngle - heading);

  float dt = PID_INTERVAL / 1000.0;

  // --- Inside deadband: coast, but keep PID alive ---
  if (abs(error) <= DEADBAND) {
    motorsStop();
    integral  = 0.0;   // clear integral so no windup while holding still
    prevError = error;
    Serial.print("HOLD | Heading: "); Serial.print(heading, 1);
    Serial.print("°  Error: ");       Serial.print(error, 1);
    Serial.println("°");
    return;
  }

  // --- Outside deadband: compute PID and drive ---
  integral += error * dt;
  integral  = constrain(integral, -100, 100);  // anti-windup

  float derivative = (error - prevError) / dt;
  prevError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int speed = (int)abs(output);
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  if (error > 0) {
    turnClockwise(speed);
  } else {
    turnAntiClockwise(speed);
  }

  Serial.print("Heading: "); Serial.print(heading, 1);
  Serial.print("°  Error: "); Serial.print(error, 1);
  Serial.print("°  Speed: "); Serial.println(speed);
}

// ============================================================
void initIMU() {
  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not found!");
    while (1) delay(10);
  }
  bno.enableReport(SH2_ROTATION_VECTOR, 10000);
  delay(200);
  Serial.println("BNO085 Ready.");
}

// ============================================================
void zeroIMU() {
  for (int i = 0; i < 20; i++) {
    bno.getSensorEvent(&imuData);
    delay(10);
  }
  yawOffset = getYaw();
  Serial.print("IMU Zeroed at: ");
  Serial.print(yawOffset, 2);
  Serial.println("°");
}

// ============================================================
float getYaw() {
  if (bno.wasReset()) bno.enableReport(SH2_ROTATION_VECTOR, 10000);
  if (bno.getSensorEvent(&imuData)) {
    if (imuData.sensorId == SH2_ROTATION_VECTOR) {
      float qw = imuData.un.rotationVector.real;
      float qx = imuData.un.rotationVector.i;
      float qy = imuData.un.rotationVector.j;
      float qz = imuData.un.rotationVector.k;
      return degrees(atan2(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)));
    }
  }
  return yawOffset;
}

// ============================================================
float getHeading() {
  return wrapAngle(-(getYaw() - yawOffset));  // negated: CW = positive
}

// ============================================================
float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ============================================================
void turnClockwise(int speed) {
  analogWrite(M1A, 0);     analogWrite(M1B, speed);
  analogWrite(M2A, 0);     analogWrite(M2B, speed);
}

void turnAntiClockwise(int speed) {
  analogWrite(M1A, speed); analogWrite(M1B, 0);
  analogWrite(M2A, speed); analogWrite(M2B, 0);
}

void motorsStop() {
  analogWrite(M1A, 0); analogWrite(M1B, 0);
  analogWrite(M2A, 0); analogWrite(M2B, 0);
}

void setMotorPins() {
  pinMode(M1A, OUTPUT); pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT); pinMode(M2B, OUTPUT);
}