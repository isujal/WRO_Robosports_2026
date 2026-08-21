#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L1X.h>
#include <Pixy2SPI_SS.h>

const int LEFT_A = 42;
const int LEFT_B = 41;
const int RIGHT_A = 15;
const int RIGHT_B = 16;
const int INTAKE_A = 38;
const int INTAKE_B = 39;
const int BNO_SDA = 18;
const int BNO_SCL = 17;
const int BNO_RST = 12;
const int TOF_SDA = 6;
const int TOF_SCL = 9;
const int SERVO_PIN = 4;
const int ORANGE = 1;
const int PURPLE = 2;
const int MIN_AREA = 100;
const int PIXY_CENTRE = 158;
const int BALL_CLOSE_AREA = 2300;
const int PURPLE_SHOT_AREA = 350;
const int DRIVE_SPEED = 180;
const int APPROACH_SPEED = 90;
const int SLOW_SPEED = 75;
const int TURN_MIN_SPEED = 60;
const int TURN_MAX_SPEED = 180;
const int INTAKE_SPEED = 255;
const int SERVO_NEUTRAL = 0;
const int SERVO_SHOOT = 65;
const int SERVO_LOAD = 140;
const int PIXY_SAMPLE_MS = 1500;
const int PID_INTERVAL = 20;
const unsigned long T1_PHASE1_MS = 400;
const unsigned long T1_PHASE2_MS = 300;
const unsigned long T1_PHASE3_MS = 200;
const unsigned long T2_PHASE1_MS = 400;
const unsigned long T2_PHASE2_MS = 200;
const unsigned long T3_PHASE1_MS = 450;
const unsigned long T3_PHASE2_MS = 300;
const unsigned long T3_PHASE3_MS = 200;
const unsigned long T4_PHASE1_MS = 400;
const unsigned long T4_PHASE2_MS = 200;
const int ROUTE_PAUSE_MS = 200;
const int BREADTH_TIME_MS = 500;
const float STOP_DISTANCE_CM = 15.0;
const float STOP_DISTANCE_CM_2 = 6.0;
const int ROI_TOP_Y = 40;
const int SPLIT_X = 200;
const int SPLIT_Y = 77;
const float OPENING_RIGHT_HEADING = 12.0;
const float OPENING_FORWARD_CM = 32.0;
const float LONG_SIDE_CM = 225.0;
const float LONG_SIDE_HEADING = 0.0;
const float DRIVE_CM_PER_SECOND = 70.0;
const float FWD_KP = 2.6;
const float FWD_KI = 0.0;
const float FWD_KD = 0.5;
const float FWD_KF = 1.0;
const int FWD_MAX_CORRECTION = 60;
const float TURN_KP = 5.0;
const float TURN_KD = 0.5;
const float TURN_DEADBAND = 5.0;
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t SERVO_PWM_RES = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500 * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

Adafruit_BNO08x bno(BNO_RST);
Adafruit_VL53L1X lox;
Pixy2SPI_SS pixy;
sh2_SensorValue_t imuData;
float yawOffset = 0;
float coordinateX = 0;
float coordinateY = 0;
bool purpleStored = false;
bool orangeHeld = false;

enum Zone {
  UNKNOWN,
  TOP_LEFT,
  BOT_LEFT,
  TOP_RIGHT,
  BOT_RIGHT
};

float WrapAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float ShortestError(float target, float current) {
  return WrapAngle(target - current);
}

void DriveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  if (leftSpeed > 0) {
    analogWrite(LEFT_A, 0);
    analogWrite(LEFT_B, leftSpeed);
  }
  else if (leftSpeed < 0) {
    analogWrite(LEFT_A, -leftSpeed);
    analogWrite(LEFT_B, 0);
  }
  else {
    analogWrite(LEFT_A, 0);
    analogWrite(LEFT_B, 0);
  }
  if (rightSpeed > 0) {
    analogWrite(RIGHT_A, rightSpeed);
    analogWrite(RIGHT_B, 0);
  }
  else if (rightSpeed < 0) {
    analogWrite(RIGHT_A, 0);
    analogWrite(RIGHT_B, -rightSpeed);
  }
  else {
    analogWrite(RIGHT_A, 0);
    analogWrite(RIGHT_B, 0);
  }
}

void StopDrive() {
  DriveMotors(0, 0);
}

void TurnClockwise(int speed) {
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, speed);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, speed);
}

void TurnAntiClockwise(int speed) {
  analogWrite(LEFT_A, speed);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, speed);
  analogWrite(RIGHT_B, 0);
}

void IntakeOn() {
  analogWrite(INTAKE_A, 0);
  analogWrite(INTAKE_B, INTAKE_SPEED);
}

void IntakeOff() {
  analogWrite(INTAKE_A, 0);
  analogWrite(INTAKE_B, 0);
}

void ServoWrite(int angle) {
  uint16_t duty = map(constrain(angle, 0, 180), 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
  ledcWrite(SERVO_PIN, duty);
}

float GetRawYaw() {
  if (bno.wasReset()) bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  while (true) {
    if (bno.getSensorEvent(&imuData) && imuData.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = imuData.un.gameRotationVector.real;
      float qx = imuData.un.gameRotationVector.i;
      float qy = imuData.un.gameRotationVector.j;
      float qz = imuData.un.gameRotationVector.k;
      return -degrees(atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)));
    }
  }
}

float GetHeading() {
  return WrapAngle(GetRawYaw() - yawOffset);
}

void ZeroIMU() {
  for (int i = 0; i < 20; i++) {
    bno.getSensorEvent(&imuData);
    delay(10);
  }
  yawOffset = GetRawYaw();
}

void RunForwardPIDF(float target, int base, float &previous, float &integral) {
  float heading = GetHeading();
  float error = ShortestError(target, heading);
  float dt = PID_INTERVAL / 1000.0f;
  integral = constrain(integral + error * dt, -50.0f, 50.0f);
  float derivative = (error - previous) / dt;
  previous = error;
  float correction = FWD_KP * error + FWD_KI * integral + FWD_KD * derivative;
  correction = constrain(correction, -(float)FWD_MAX_CORRECTION, (float)FWD_MAX_CORRECTION);
  int feedForward = base * FWD_KF;
  DriveMotors(constrain(feedForward + (int)correction, 0, 255), constrain(feedForward - (int)correction, 0, 255));
}

void UpdateCoordinates(int speed, float heading, float dt) {
  float distance = DRIVE_CM_PER_SECOND * ((float)speed / DRIVE_SPEED) * dt;
  float radians = heading * 0.0174532925f;
  coordinateX += distance * sinf(radians);
  coordinateY += distance * cosf(radians);
}

void DriveDistancePIDF(float distance, int speed, float headingTarget) {
  float previous = 0;
  float integral = 0;
  float travelled = 0;
  unsigned long last = millis();
  unsigned long start = millis();
  while (travelled < distance && millis() - start < 5000) {
    unsigned long now = millis();
    if (now - last < PID_INTERVAL) continue;
    float dt = (now - last) / 1000.0f;
    last = now;
    RunForwardPIDF(headingTarget, speed, previous, integral);
    float heading = GetHeading();
    travelled += DRIVE_CM_PER_SECOND * ((float)speed / DRIVE_SPEED) * dt;
    UpdateCoordinates(speed, heading, dt);
  }
  StopDrive();
}

void TurnTo(float target) {
  float previous = 0;
  int stable = 0;
  unsigned long last = millis();
  unsigned long start = millis();
  while (millis() - start < 4500) {
    unsigned long now = millis();
    if (now - last < PID_INTERVAL) continue;
    last = now;
    float error = ShortestError(target, GetHeading());
    if (fabsf(error) <= TURN_DEADBAND) {
      StopDrive();
      if (++stable >= 8) return;
      continue;
    }
    stable = 0;
    float derivative = (error - previous) / (PID_INTERVAL / 1000.0f);
    previous = error;
    int speed = constrain((int)fabsf(TURN_KP * error + TURN_KD * derivative), TURN_MIN_SPEED, TURN_MAX_SPEED);
    if (error > 0) TurnClockwise(speed);
    else TurnAntiClockwise(speed);
  }
  StopDrive();
}

bool FindBall(int signature, int &x, long &area) {
  pixy.ccc.getBlocks();
  bool found = false;
  long largest = 0;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature != signature) continue;
    long current = (long)pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
    if (current < MIN_AREA) continue;
    if (current > largest) {
      largest = current;
      x = pixy.ccc.blocks[i].m_x;
      area = current;
      found = true;
    }
  }
  return found;
}

Zone ClassifyZone(int x, int y) {
  if (x < SPLIT_X) {
    if (y < 78) return TOP_LEFT;
    return BOT_LEFT;
  }
  if (y < 82) return TOP_RIGHT;
  return BOT_RIGHT;
}

Zone DetectPurpleZone() {
  int votesTopLeft = 0;
  int votesBotLeft = 0;
  int votesTopRight = 0;
  int votesBotRight = 0;
  unsigned long start = millis();

  while (millis() - start < PIXY_SAMPLE_MS) {
    pixy.ccc.getBlocks();
    int bestX = 0;
    int bestY = 0;
    long bestArea = 0;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature != PURPLE) continue;
      if (pixy.ccc.blocks[i].m_y < ROI_TOP_Y) continue;

      long area = (long)pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;

      if (area < MIN_AREA) continue;

      if (area > bestArea) {
        bestArea = area;
        bestX = pixy.ccc.blocks[i].m_x;
        bestY = pixy.ccc.blocks[i].m_y;
      }
    }

    if (bestArea > 0) {
      Zone zone = ClassifyZone(bestX, bestY);

      if (zone == TOP_LEFT) votesTopLeft++;
      else if (zone == BOT_LEFT) votesBotLeft++;
      else if (zone == TOP_RIGHT) votesTopRight++;
      else if (zone == BOT_RIGHT) votesBotRight++;
    }

    StopDrive();
    delay(50);
  }

  int best = votesTopLeft;

  if (votesBotLeft > best) best = votesBotLeft;
  if (votesTopRight > best) best = votesTopRight;
  if (votesBotRight > best) best = votesBotRight;

  if (best == 0) return UNKNOWN;
  if (votesTopLeft == best) return TOP_LEFT;
  if (votesBotLeft == best) return BOT_LEFT;
  if (votesTopRight == best) return TOP_RIGHT;
  return BOT_RIGHT;
}

void ExecuteDrive(unsigned long duration, float targetHeading, int speed = DRIVE_SPEED) {
  float previous = 0;
  float integral = 0;
  unsigned long last = millis();
  unsigned long start = millis();

  while (millis() - start < duration) {
    unsigned long now = millis();

    if (now - last >= PID_INTERVAL) {
      last = now;
      RunForwardPIDF(targetHeading, speed, previous, integral);
    }
  }

  StopDrive();
}

float GetDistanceCM() {
  if (!lox.dataReady()) return -1;

  int16_t distance = lox.distance();
  lox.clearInterrupt();

  if (distance <= 0) return -1;

  return distance / 10.0f;
}

void ExecuteDriveUntilClose(float targetHeading, float stopDistance, bool shootMidway) {
  float previous = 0;
  float integral = 0;
  unsigned long last = millis();
  unsigned long start = millis();
  bool shot = false;

  while (millis() - start < 2000) {
    unsigned long now = millis();

    if (shootMidway && !shot && now - start >= 600) {
      ServoWrite(SERVO_SHOOT);
      delay(250);
      ServoWrite(SERVO_NEUTRAL);
      shot = true;
    }

    if (now - last >= PID_INTERVAL) {
      last = now;
      float distance = GetDistanceCM();

      if (distance > 0 && distance < stopDistance) break;

      RunForwardPIDF(targetHeading, 220, previous, integral);
    }
  }

  StopDrive();
}

void RunTrajectory1() {
  IntakeOn();
  ServoWrite(SERVO_LOAD);
  delay(500);
  ExecuteDrive(T1_PHASE1_MS, 0.0);
  ServoWrite(SERVO_SHOOT);
  delay(500);
  ExecuteDrive(T1_PHASE2_MS, 0.0);
  StopDrive();
  delay(ROUTE_PAUSE_MS);
  ServoWrite(SERVO_NEUTRAL);
  delay(250);
  ExecuteDrive(T1_PHASE3_MS, 0.0);
  StopDrive();
  IntakeOff();
  delay(200);
  ServoWrite(SERVO_SHOOT);
  delay(250);
}

void RunTrajectory2() {
  IntakeOn();
  ServoWrite(SERVO_NEUTRAL);
  delay(250);
  ExecuteDrive(T2_PHASE1_MS, 0.0);
  StopDrive();
  delay(150);
  ServoWrite(SERVO_SHOOT);
  delay(250);
  ServoWrite(SERVO_LOAD);
  delay(250);
  ExecuteDrive(T2_PHASE2_MS, 0.0);
  ServoWrite(SERVO_NEUTRAL);
  delay(250);
  StopDrive();
}

void RunTrajectory3() {
  TurnTo(30.0);
  delay(ROUTE_PAUSE_MS);
  IntakeOn();
  ServoWrite(SERVO_LOAD);
  delay(500);
  ExecuteDrive(T3_PHASE1_MS, 30.0);
  ServoWrite(SERVO_SHOOT);
  delay(500);
  ExecuteDrive(T3_PHASE2_MS, 30.0);
  StopDrive();
  delay(ROUTE_PAUSE_MS);
  ServoWrite(SERVO_NEUTRAL);
  delay(250);
  ExecuteDrive(T3_PHASE3_MS, 30.0);
  StopDrive();
  IntakeOff();
  delay(200);
  ServoWrite(SERVO_SHOOT);
  delay(250);
  TurnTo(180.0);
  delay(ROUTE_PAUSE_MS);
}

void RunTrajectory4() {
  TurnTo(30.0);
  delay(ROUTE_PAUSE_MS);
  IntakeOn();
  ServoWrite(SERVO_NEUTRAL);
  delay(250);
  ExecuteDrive(T4_PHASE1_MS, 30.0);
  StopDrive();
  delay(150);
  ServoWrite(SERVO_SHOOT);
  delay(250);
  ServoWrite(SERVO_LOAD);
  delay(250);
  ExecuteDrive(T4_PHASE2_MS, 30.0);
  ServoWrite(SERVO_NEUTRAL);
  delay(250);
  StopDrive();
  TurnTo(180.0);
  delay(ROUTE_PAUSE_MS);
}

void RunRectangleLap() {
  IntakeOn();
  ServoWrite(SERVO_NEUTRAL);
  ExecuteDriveUntilClose(0.0, STOP_DISTANCE_CM, true);
  delay(ROUTE_PAUSE_MS);
  TurnTo(45.0);
  delay(ROUTE_PAUSE_MS);
  ExecuteDrive(BREADTH_TIME_MS, 90.0, 220);
  delay(ROUTE_PAUSE_MS);
  TurnTo(135.0);
  delay(ROUTE_PAUSE_MS);
  ExecuteDriveUntilClose(180.0, STOP_DISTANCE_CM_2, false);
  delay(ROUTE_PAUSE_MS);
  TurnTo(-135.0);
  delay(ROUTE_PAUSE_MS);
  ExecuteDrive(BREADTH_TIME_MS, -90.0, 220);
  delay(ROUTE_PAUSE_MS);
  TurnTo(-45.0);
  delay(ROUTE_PAUSE_MS);
}

void RunRectangleLapFromSide3() {
  IntakeOn();
  ServoWrite(SERVO_NEUTRAL);
  ExecuteDriveUntilClose(180.0, STOP_DISTANCE_CM, false);
  delay(ROUTE_PAUSE_MS);
  TurnTo(-135.0);
  delay(ROUTE_PAUSE_MS);
  ExecuteDrive(BREADTH_TIME_MS, -90.0, 220);
  delay(ROUTE_PAUSE_MS);
  TurnTo(-45.0);
  delay(ROUTE_PAUSE_MS);
  ExecuteDriveUntilClose(0.0, STOP_DISTANCE_CM, true);
  delay(ROUTE_PAUSE_MS);
  TurnTo(45.0);
  delay(ROUTE_PAUSE_MS);
  ExecuteDrive(BREADTH_TIME_MS, 90.0, 220);
  delay(ROUTE_PAUSE_MS);
  TurnTo(135.0);
  delay(ROUTE_PAUSE_MS);
}

bool DriveToOpponentPurple() {
  int x = PIXY_CENTRE;
  long area = 0;
  float previous = 0;
  float integral = 0;
  unsigned long last = millis();
  unsigned long start = millis();

  while (millis() - start < 3200) {
    if (FindBall(PURPLE, x, area) && area >= PURPLE_SHOT_AREA) {
      StopDrive();
      return true;
    }

    unsigned long now = millis();

    if (now - last >= PID_INTERVAL) {
      last = now;
      RunForwardPIDF(0.0, DRIVE_SPEED, previous, integral);
    }
  }

  StopDrive();

  return FindBall(PURPLE, x, area);
}

bool AimAtOpponentPurple() {
  int x = PIXY_CENTRE;
  long area = 0;
  int stable = 0;
  unsigned long start = millis();

  while (millis() - start < 3500) {
    if (!FindBall(PURPLE, x, area)) {
      stable = 0;
      TurnClockwise(TURN_MIN_SPEED);
      delay(15);
      continue;
    }

    int error = x - PIXY_CENTRE;

    if (abs(error) <= 5) {
      StopDrive();

      if (++stable >= 8) return true;
    }
    else {
      stable = 0;
      int speed = constrain(abs(error) / 2, TURN_MIN_SPEED, 120);

      if (error > 0) TurnClockwise(speed);
      else TurnAntiClockwise(speed);
    }

    delay(15);
  }

  StopDrive();

  return false;
}

void RunNoPurpleAttack() {
  IntakeOn();
  ServoWrite(SERVO_NEUTRAL);
  delay(250);
  TurnTo(OPENING_RIGHT_HEADING);
  delay(ROUTE_PAUSE_MS);
  ExecuteDrive(450, OPENING_RIGHT_HEADING, DRIVE_SPEED);
  delay(ROUTE_PAUSE_MS);
  TurnTo(0.0);
  delay(ROUTE_PAUSE_MS);
  ExecuteDrive(350, 0.0, DRIVE_SPEED);
  StopDrive();
  delay(300);
  DriveToOpponentPurple();
  StopDrive();
  delay(150);

  if (AimAtOpponentPurple()) {
    ServoWrite(SERVO_SHOOT);
    delay(650);
    ServoWrite(SERVO_NEUTRAL);
  }

  IntakeOff();
  StopDrive();
}

void OpeningRoute() {
  Zone zone = DetectPurpleZone();

  if (zone == BOT_LEFT) {
    RunTrajectory1();

    while (true) {
      RunRectangleLap();
    }
  }

  if (zone == TOP_LEFT) {
    RunTrajectory2();

    while (true) {
      RunRectangleLap();
    }
  }

  if (zone == BOT_RIGHT) {
    RunTrajectory3();

    while (true) {
      RunRectangleLapFromSide3();
    }
  }

  if (zone == TOP_RIGHT) {
    RunTrajectory4();

    while (true) {
      RunRectangleLapFromSide3();
    }
  }

  RunNoPurpleAttack();

  while (true) {
    RunRectangleLap();
  }
}

void StopEverything() {
  StopDrive();
  IntakeOff();
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  pinMode(INTAKE_A, OUTPUT);
  pinMode(INTAKE_B, OUTPUT);
  StopEverything();
  ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_RES);
  ServoWrite(SERVO_NEUTRAL);
  Wire.begin(BNO_SDA, BNO_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    while (true) {
      StopEverything();
      delay(10);
    }
  }
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  Wire1.begin(TOF_SDA, TOF_SCL);
    Serial.print("Before begin");

  if (!lox.begin(0x29, &Wire1, false)) {
    while (true) {
      StopEverything();
      delay(10);
    }
  }
    Serial.print("After begin");

  lox.setTimingBudget(50);
  lox.startRanging();
  pixy.init();
  pixy.changeProg("color_connected_components");
  delay(500);
  ZeroIMU();
  delay(300);
  OpeningRoute();
  StopEverything();
}

void loop() {
}