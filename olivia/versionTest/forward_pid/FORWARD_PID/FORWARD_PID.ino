// ============================================================
//  Drive Straight with PID Heading Hold — BNO085 + ESP32-S3
//
//  Bot drives forward while PID keeps heading at 0° (zeroed
//  at startup). If it drifts left/right, PID corrects by
//  speeding up one side and slowing the other.
//
//  Serial Commands:
//    F → Drive straight (forward)
//    S → Stop
//    B → Drive straight (backward)
//    Z → Re-zero IMU at current position
//
//  Motor Pins:
//    M1A=6, M1B=7   → Left motor
//    M2A=15, M2B=16 → Right motor
//
//  IMU: BNO085 on SDA=18, SCL=17, RST=12
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// -------- Motor Pins --------
const int M1A = 42;
const int M1B = 41;
const int M2A = 15;
const int M2B = 16;

// -------- IMU Pins --------
#define BNO08X_SDA 18
#define BNO08X_SCL 17
#define BNO08X_RST 12

// -------- Drive Speed --------
// Base speed for both motors. Start low, increase once confirmed straight.
const int BASE_SPEED = 50;   // 0–255. Start here, tune up slowly.

// -------- PID Tuning --------
// Kp: how hard to correct. Start with 1.0, increase if drifting too much.
// Ki: leave at 0 to start.
// Kd: helps reduce overcorrection wobble. Add after Kp is good.
float Kp = 2.6;
float Ki = 0.0;
float Kd = 0.5;

// -------- PID Config --------
const float DEADBAND     = 1.5;   // degrees — no correction inside this
const int   MAX_CORRECT  = 60;    // max PWM correction applied to either side
const int   PID_INTERVAL = 20;    // ms between PID updates

// -------- State --------
enum DriveState { STOPPED, FORWARD, BACKWARD };
DriveState driveState = STOPPED;

// -------- IMU --------
Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;
float yawOffset = 0.0;

// -------- PID State --------
float prevError      = 0.0;
float integral       = 0.0;
unsigned long lastPIDTime = 0;

// -------- Function Prototypes --------
void  initIMU();
void  zeroIMU();
float getYaw();
float getHeading();
float wrapAngle(float a);
void  runDrivePID();
void  driveMotors(int leftSpeed, int rightSpeed);
void  motorsStop();
void  setMotorPins();
void  printMenu();

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  setMotorPins();
  
  motorsStop();

  initIMU();
  zeroIMU();   // current heading = 0°, bot will hold this while driving

  printMenu();
}

// ============================================================
void loop() {

  // --- Serial Commands ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    char cmd = toupper(input.charAt(0));

    switch (cmd) {
      case 'F':
        driveState = FORWARD;
        integral   = 0.0;
        prevError  = 0.0;
        Serial.println(">> FORWARD — PID heading hold active");
        break;

      case 'B':
        driveState = BACKWARD;
        integral   = 0.0;
        prevError  = 0.0;
        Serial.println(">> BACKWARD — PID heading hold active");
        break;

      case 'S':
        driveState = STOPPED;
        motorsStop();
        integral  = 0.0;
        prevError = 0.0;
        Serial.println(">> STOPPED");
        break;

      case 'Z':
        driveState = STOPPED;
        motorsStop();
        zeroIMU();
        integral  = 0.0;
        prevError = 0.0;
        Serial.println(">> IMU Re-zeroed. Send F to drive.");
        break;

      default:
        Serial.println(">> Unknown. Use F / B / S / Z");
        break;
    }
  }

  // --- PID Loop — only runs while driving ---
  if (driveState != STOPPED) {
    unsigned long now = millis();
    if (now - lastPIDTime >= PID_INTERVAL) {
      lastPIDTime = now;
      runDrivePID();
    }
  }
}

// ============================================================
//  runDrivePID()
//
//  Reads heading error from zero.
//  Computes PID correction value.
//  Applies correction:
//    Drifting RIGHT (positive error) → slow right, speed up left
//    Drifting LEFT  (negative error) → slow left, speed up right
//
//  Left and right speeds are clamped to valid PWM range.
// ============================================================
void runDrivePID() {
  float heading = getHeading();
  float error   = wrapAngle(0.0 - heading);  // target is always 0°

  float dt = PID_INTERVAL / 1000.0;

  // --- Inside deadband: drive straight, no correction ---
  if (abs(error) <= DEADBAND) {
    integral  = 0.0;  // don't accumulate while straight
    prevError = error;

    if (driveState == FORWARD)
      driveMotors(BASE_SPEED, BASE_SPEED);
    else
      driveMotors(-BASE_SPEED, -BASE_SPEED);

    Serial.print("STRAIGHT | Heading: "); Serial.print(heading, 1); Serial.println("°");
    return;
  }

  // --- PID Calculation ---
  integral += error * dt;
  integral  = constrain(integral, -50, 50);   // anti-windup

  float derivative = (error - prevError) / dt;
  prevError = error;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -MAX_CORRECT, MAX_CORRECT);

  // --- Apply correction to left/right speeds ---
  // Positive error = drifted right = need to turn left
  //   → reduce right motor, increase left motor
  // Negative error = drifted left = need to turn right
  //   → reduce left motor, increase right motor
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  // Clamp both to valid range
  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  if (driveState == BACKWARD) {
    leftSpeed  = -leftSpeed;
    rightSpeed = -rightSpeed;
  }

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("Heading: "); Serial.print(heading, 1);
  Serial.print("°  Error: "); Serial.print(error, 1);
  Serial.print("°  Correction: "); Serial.print(correction, 1);
  Serial.print("  L: "); Serial.print(abs(leftSpeed));
  Serial.print("  R: "); Serial.println(abs(rightSpeed));
}

// ============================================================
//  driveMotors()
//
//  Accepts signed speed for each side:
//    Positive → forward
//    Negative → backward
//    0        → stop that side
//
//  This is the only place that touches motor pins directly.
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // --- Left Motor (M1) ---
  if (leftSpeed > 0) {
    analogWrite(M1A, 0);          analogWrite(M1B, leftSpeed);
  } else if (leftSpeed < 0) {
    analogWrite(M1A, -leftSpeed); analogWrite(M1B, 0);
  } else {
    analogWrite(M1A, 0);          analogWrite(M1B, 0);
  }

  // --- Right Motor (M2) ---
  if (rightSpeed > 0) {
    analogWrite(M2A, rightSpeed); analogWrite(M2B, 0);
  } else if (rightSpeed < 0) {
    analogWrite(M2A, 0);          analogWrite(M2B, -rightSpeed);
  } else {
    analogWrite(M2A, 0);          analogWrite(M2B, 0);
  }
}

// ============================================================
void initIMU() {
  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not found! Check wiring.");
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
  return wrapAngle(-(getYaw() - yawOffset));  // CW = positive
}

// ============================================================
float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ============================================================
void motorsStop() {
  analogWrite(M1A, 0); analogWrite(M1B, 0);
  analogWrite(M2A, 0); analogWrite(M2B, 0);
}

void setMotorPins() {
  pinMode(M1A, OUTPUT); pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT); pinMode(M2B, OUTPUT);
}

void printMenu() {
  Serial.println();
  Serial.println("===== Drive Straight PID =====");
  Serial.println("  F → Forward (heading locked to 0°)");
  Serial.println("  B → Backward (heading locked to 0°)");
  Serial.println("  S → Stop");
  Serial.println("  Z → Re-zero IMU");
  Serial.println("==============================");
  Serial.println("Tune BASE_SPEED and Kp first.");
  Serial.println("Start: BASE_SPEED=100, Kp=1.5, Ki=0, Kd=0.3");
  Serial.println("==============================");
}