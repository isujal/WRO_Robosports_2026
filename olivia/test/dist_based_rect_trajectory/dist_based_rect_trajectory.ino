// ============================================================
//  Rectangle Trajectory + Intake + Shooter Servo + VL53L0X
//  BNO085 + MDD 3A + ESP32-S3
//  MODIFIED: Long sides use TOF distance trigger (<12cm) to stop
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>

int pause_ms = 50;
int breadth_pause = 500;
int reset_pause = 400;

const float STOP_DISTANCE_CM = 12.0;  // Stop threshold on long sides

// -------- Drive Motor Pins --------
const int M1A = 6;
const int M1B = 7;
const int M2A = 15;
const int M2B = 16;

// -------- IMU Pins (Wire — Bus 0) --------
#define BNO08X_SDA 18
#define BNO08X_SCL 17
#define BNO08X_RST 12
 We are using the pixie button bot We are using the pixie button bottom points based a freedom to the qualification for exactly defined exactly note because it's called alternates testing and directions female to female jumper
// -------- VL53L0X Pins (Wire1 — Bus 1) --------
#define TOF_SDA 46
#define TOF_SCL 9

// -------- Intake Motor Pins --------
const int INTAKE_IN1   = 3;
const int INTAKE_IN2   = 8;
const int INTAKE_SPEED = 255;

// -------- Servo (14-bit LEDC) --------
#define SERVO_PIN 4
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

const int SERVO_INTAKE_POS = 0;
const int SERVO_SHOOT_POS  = 50;

// ============================================================
//  FORWARD PID CONSTANTS
// ============================================================
const float FWD_KP          = 2.6;
const float FWD_KI          = 0.0;
const float FWD_KD          = 0.5;
const int   FWD_BASE_SPEED  = 220;   // Slowed down so TOF has time to react
const int   FWD_MAX_CORRECT = 60;
const float FWD_DEADBAND    = 1.5;
const int   FWD_INTERVAL    = 20;

// ============================================================
//  TURN PID CONSTANTS
// ============================================================
const float TRN_KP           = 5; //3.25;
const float TRN_KI           = 0.0;
const float TRN_KD           = 0.8;
const int   TRN_MIN_SPEED    = 60;
const int   TRN_MAX_SPEED    = 255;
const float TRN_DEADBAND     = 8.0;
const int   TRN_INTERVAL     = 20;
const int   TRN_STABLE_COUNT = 8;

// -------- IMU --------
Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;
float yawOffset = 0.0;

// -------- VL53L0X --------
Adafruit_VL53L0X lox;

// -------- Function Prototypes --------
void     initIMU();
void     zeroIMU();
void     initTOF();
float    getDistanceCm();
float    getRawYaw();
float    getHeading();
float    shortestError(float target, float current);
float    wrapAngle(float a);
void     executeDrive(unsigned long durationMs, float targetHeading);
void     executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway = false);
void     executeTurn(float targetAngle);
void     runForwardPID(float targetHeading, float &prevErr, float &integ);
void     driveMotors(int leftSpeed, int rightSpeed);
void     turnClockwise(int speed);
void     turnAntiClockwise(int speed);
void     motorsStop();
void     setMotorPins();
void     waitMs(int ms);
void     initIntake();
void     intakeOn();
void     intakeOff();
void     initServo();
void     servoWrite(int angle);
void     servoShoot();

// ============================================================
void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(10);

  setMotorPins();
  motorsStop();

  initIntake();
  initServo();

  initIMU();
  zeroIMU();

  initTOF();

  Serial.println("Ready. Starting in 2 seconds...");
  delay(2000);
        zeroIMU();


}

// ============================================================
void loop() {



  intakeOn();
  Serial.println("Intake ON");

  servoWrite(SERVO_INTAKE_POS);
  Serial.println("Servo at 90deg");

  // ============================================================
  //  RECTANGLE SEQUENCE
  // ============================================================

  // SIDE 1: Drive until wall is <12cm (distance-triggered stop)
  Serial.println("=== SIDE 1: Forward until <12cm @ 0deg ===");
    executeDriveUntilClose(0.0, STOP_DISTANCE_CM, true);

  waitMs(pause_ms);

  // Serial.println("=== SHOOTER: 135deg → 250ms → 90deg ===");
  // servoShoot();
  // waitMs(pause_ms);

  Serial.println("=== TURN 1: To 90deg ===");
  executeTurn(45.0);
  waitMs(pause_ms);

  // SIDE 2: Short side, time-based
  Serial.println("=== SIDE 2: Forward 300ms @ 90deg ===");
  executeDrive(breadth_pause, 90.0);
  waitMs(pause_ms);

  Serial.println("=== TURN 2: To 180deg ===");
  executeTurn(135.0);
  waitMs(pause_ms);

  // SIDE 3: Drive until wall is <12cm (distance-triggered stop)
  Serial.println("=== SIDE 3: Forward until <12cm @ 180deg ===");
  executeDriveUntilClose(180.0, STOP_DISTANCE_CM, false);
  waitMs(pause_ms);

  Serial.println("=== TURN 3: To -90deg ===");
  executeTurn(-135.0);
  waitMs(pause_ms);

  // SIDE 4: Short side, time-based
  Serial.println("=== SIDE 4: Forward 300ms @ -90deg ===");
  executeDrive(breadth_pause, -90.0);
  waitMs(pause_ms);

  Serial.println("=== TURN 4: Back to 0deg ===");
  executeTurn(-45.0);
  waitMs(pause_ms);

  // executeDrive(reset_pause, 0.0);
  // waitMs(pause_ms);


  Serial.println("=== RECTANGLE COMPLETE — RESTARTING ===");
}

// ============================================================
//  executeDriveUntilClose()
//  Drives forward with heading PID until TOF reads < stopDistCm.
//  Falls back to a max timeout (5 seconds) as a safety net.
// ============================================================

void executeDriveUntilClose(float targetHeading, float stopDistCm, bool shootMidway) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long lastPIDTime  = millis();
  unsigned long startTime    = millis();
  const unsigned long MAX_TIMEOUT = 2000;

  // ── Non-blocking shoot state ────────────────────────────
  bool shootDone      = false;
  bool shootTriggered = false;
  unsigned long shootStartTime = 0;
  // Phase 0 = not started, 1 = moving to SHOOT_POS, 2 = moving back to INTAKE_POS
  int shootPhase = 0;
  // ────────────────────────────────────────────────────────

  while (true) {
    unsigned long now = millis();

    if (now - startTime > MAX_TIMEOUT) {
      Serial.println("TOF drive: MAX TIMEOUT reached, stopping.");
      break;
    }

    // ── Non-blocking servo shoot state machine ───────────
    if (shootMidway && !shootDone) {

      // Trigger: 500ms into drive
      if (!shootTriggered && (now - startTime >= 600)) {
        Serial.println("Mid-drive SHOOT triggered");
        servoWrite(SERVO_SHOOT_POS);   // command servo, no delay
        shootTriggered = true;
        shootStartTime = now;
        shootPhase = 1;
      }

      // Phase 1 → 2: servo has had 250ms to reach SHOOT_POS, now return
      if (shootPhase == 1 && (now - shootStartTime >= 250)) {
        servoWrite(SERVO_INTAKE_POS);
        shootPhase = 2;
      }

      // Phase 2 → done: servo has had 250ms to return to INTAKE_POS
      if (shootPhase == 2 && (now - shootStartTime >= 500)) {
        shootDone = true;
        Serial.println("Mid-drive SHOOT complete");
      }
    }
    // ────────────────────────────────────────────────────────

    // ── PID runs every FWD_INTERVAL ms, uninterrupted ────
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;

      float dist = getDistanceCm();

      Serial.print("TOF Drive | Dist:");
      if (dist >= 0) { Serial.print(dist, 1); Serial.print("cm"); }
      else             Serial.print("OOR");

      if (dist > 0 && dist < stopDistCm) {
        Serial.println(" → STOP (wall detected)");
        motorsStop();
        break;
      }

      Serial.println();
      runForwardPID(targetHeading, prevErr, integ);
    }
  }

  motorsStop();
}





// void executeDriveUntilClose(float targetHeading, float stopDistCm) {
//   float prevErr = 0.0;
//   float integ   = 0.0;
//   unsigned long lastPIDTime = millis();
//   unsigned long startTime   = millis();
//   const unsigned long MAX_TIMEOUT = 2000;  // 5s safety fallback

//   while (true) {
//     // Safety timeout — never run forever
//     if (millis() - startTime > MAX_TIMEOUT) {
//       Serial.println("TOF drive: MAX TIMEOUT reached, stopping.");
//       break;
//     }

//     unsigned long now = millis();
//     if (now - lastPIDTime >= FWD_INTERVAL) {
//       lastPIDTime = now;

//       // Check distance FIRST before moving
//       float dist = getDistanceCm();

//       Serial.print("TOF Drive | Dist:");
//       if (dist >= 0) {
//         Serial.print(dist, 1);
//         Serial.print("cm");
//       } else {
//         Serial.print("OOR");
//       }

//       // Stop condition: valid reading AND below threshold
//       if (dist > 0 && dist < stopDistCm) {
//         Serial.println(" → STOP (wall detected)");
//         motorsStop();
//         break;
//       }

//       Serial.println();

//       // Drive straight with heading PID
//       runForwardPID(targetHeading, prevErr, integ);
//     }
//   }

//   motorsStop();
// }

// ============================================================
//  initTOF()
// ============================================================
void initTOF() {
  Wire1.begin(TOF_SDA, TOF_SCL);
  if (!lox.begin(0x29, false, &Wire1)) {
    Serial.println("VL53L0X not found! Check wiring.");
    while (1) delay(10);
  }
  Serial.println("VL53L0X Ready.");
}

// ============================================================
//  getDistanceCm()
// ============================================================
float getDistanceCm() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter / 10.0;
  }
  return -1.0;
}

// ============================================================
//  SERVO FUNCTIONS
// ============================================================
void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t duty = map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
  ledcWrite(SERVO_PIN, duty);
}

void initServo() {
  ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_RES);
  servoWrite(SERVO_INTAKE_POS);
  delay(500);
  Serial.println("Servo ready at 90deg.");
}

void servoShoot() {
  Serial.println("Servo: 90 → 135deg");
  servoWrite(SERVO_SHOOT_POS);
  delay(250);
  Serial.println("Servo: 135 → 90deg");
  servoWrite(SERVO_INTAKE_POS);
  delay(250);
}

// ============================================================
//  INTAKE FUNCTIONS
// ============================================================
void initIntake() {
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, 0);
}

void intakeOn() {
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, INTAKE_SPEED);
}

void intakeOff() {
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, 0);
}

// ============================================================
//  IMU FUNCTIONS
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

float getHeading() {
  return wrapAngle(getRawYaw() - yawOffset);
}

float shortestError(float targetDeg, float currentDeg) {
  float error = wrapAngle(targetDeg) - wrapAngle(currentDeg);
  return wrapAngle(error);
}

void executeDrive(unsigned long durationMs, float targetHeading) {
  float prevErr = 0.0;
  float integ   = 0.0;
  unsigned long startTime   = millis();
  unsigned long lastPIDTime = millis();

  while (millis() - startTime < durationMs) {
    unsigned long now = millis();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPID(targetHeading, prevErr, integ);
    }
  }
  motorsStop();
}

void executeTurn(float targetAngle) {
  float prevErr     = 0.0;
  float integ       = 0.0;
  int   stableCount = 0;
  unsigned long lastPIDTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - lastPIDTime >= TRN_INTERVAL) {
      lastPIDTime = now;

      float heading = getHeading();
      float error   = shortestError(targetAngle, heading);

      if (abs(error) <= TRN_DEADBAND) {
        motorsStop();
        integ   = 0.0;
        prevErr = error;
        stableCount++;

        Serial.print("Stable "); Serial.print(stableCount);
        Serial.print("/"); Serial.print(TRN_STABLE_COUNT);
        Serial.print(" | H:"); Serial.print(heading, 1);
        Serial.print(" E:"); Serial.println(error, 1);

        // if (stableCount >= TRN_STABLE_COUNT) {
          Serial.print("Turn done. H:"); Serial.print(heading, 1); Serial.println("deg");
          return;
        // }
      } else {
        stableCount = 0;
        runTurnPID(targetAngle, prevErr, integ);
      }
    }
  }
}

// ============================================================
//  runForwardPID()
// ============================================================
void runForwardPID(float targetHeading, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetHeading, heading);
  float dt      = FWD_INTERVAL / 1000.0;

  if (abs(error) <= FWD_DEADBAND) {
    integ   = 0.0;
    prevErr = error;
    driveMotors(FWD_BASE_SPEED, FWD_BASE_SPEED);
    Serial.print("FWD STRAIGHT | H:"); Serial.println(heading, 1);
    return;
  }

  integ += error * dt;
  integ  = constrain(integ, -50, 50);

  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float correction = (FWD_KP * error) + (FWD_KI * integ) + (FWD_KD * derivative);
  correction = constrain(correction, -FWD_MAX_CORRECT, FWD_MAX_CORRECT);

  int leftSpeed  = constrain(FWD_BASE_SPEED + (int)correction, 0, 255);
  int rightSpeed = constrain(FWD_BASE_SPEED - (int)correction, 0, 255);

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("FWD | H:"); Serial.print(heading, 1);
  Serial.print(" E:"); Serial.print(error, 1);
  Serial.print(" Corr:"); Serial.print(correction, 1);
  Serial.print(" L:"); Serial.print(leftSpeed);
  Serial.print(" R:"); Serial.println(rightSpeed);
}

// ============================================================
//  runTurnPID()
// ============================================================
void runTurnPID(float targetAngle, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetAngle, heading);
  float dt      = TRN_INTERVAL / 1000.0;

  integ += error * dt;
  integ  = constrain(integ, -100, 100);

  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float output = (TRN_KP * error) + (TRN_KI * integ) + (TRN_KD * derivative);

  int speed = (int)abs(output);
  speed = constrain(speed, TRN_MIN_SPEED, TRN_MAX_SPEED);

  if (error > 0) turnClockwise(speed);
  else           turnAntiClockwise(speed);

  Serial.print("TRN | H:"); Serial.print(heading, 1);
  Serial.print(" Target:"); Serial.print(targetAngle, 1);
  Serial.print(" E:"); Serial.print(error, 1);
  Serial.print(" Spd:"); Serial.println(speed);
}

// ============================================================
float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

void waitMs(int ms) {
  motorsStop();
  delay(ms);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(M1A, 0);          analogWrite(M1B, leftSpeed);
  } else if (leftSpeed < 0) {
    analogWrite(M1A, -leftSpeed); analogWrite(M1B, 0);
  } else {
    analogWrite(M1A, 0);          analogWrite(M1B, 0);
  }
  if (rightSpeed > 0) {
    analogWrite(M2A, rightSpeed); analogWrite(M2B, 0);
  } else if (rightSpeed < 0) {
    analogWrite(M2A, 0);          analogWrite(M2B, -rightSpeed);
  } else {
    analogWrite(M2A, 0);          analogWrite(M2B, 0);
  }
}

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

void initIMU() {
  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not found!");
    while (1) delay(10);
  }
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  delay(200);
  Serial.println("BNO085 Ready.");
}

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