// ============================================================
//  Rectangle Trajectory + Intake + Shooter Servo
//  BNO085 + MDD 3A + ESP32-S3
//  + WiFi Serial Monitor (WebSocket)
// ============================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// -------- WiFi Credentials --------
const char* WIFI_SSID = "TIS_5G";
const char* WIFI_PASS = "Wecanwewill";

AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

int pause_ms    = 150;
int long_pause  = 750;
int breadth_pause = 300;

// -------- Drive Motor Pins --------
const int M1A = 6;
const int M1B = 7;
const int M2A = 15;
const int M2B = 16;

// -------- IMU Pins --------
#define BNO08X_SDA 18
#define BNO08X_SCL 17
#define BNO08X_RST 12

// -------- Intake Motor Pins --------
const int INTAKE_IN1   = 10;
const int INTAKE_IN2   = 9;
const int INTAKE_SPEED = 255;

// -------- Servo --------
#define SERVO_PIN 4
constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

const int SERVO_INTAKE_POS = 90;
const int SERVO_SHOOT_POS  = 135;

// ============================================================
//  FORWARD PID CONSTANTS
// ============================================================
const float FWD_KP          = 2.6;
const float FWD_KI          = 0.0;
const float FWD_KD          = 0.5;
const int   FWD_BASE_SPEED  = 255;
const int   FWD_MAX_CORRECT = 60;
const float FWD_DEADBAND    = 1.5;
const int   FWD_INTERVAL    = 20;

// ============================================================
//  TURN PID CONSTANTS
// ============================================================
const float TRN_KP           = 2.95;
const float TRN_KI           = 0.0;
const float TRN_KD           = 0.8;
const int   TRN_MIN_SPEED    = 60;
const int   TRN_MAX_SPEED    = 255;
const float TRN_DEADBAND     = 2.0;
const int   TRN_INTERVAL     = 20;
const int   TRN_STABLE_COUNT = 8;

// -------- IMU --------
Adafruit_BNO08x   bno(BNO08X_RST);
sh2_SensorValue_t imuData;
float yawOffset = 0.0;

// -------- Function Prototypes --------
void     initWiFi();
void     wsPrint(const String& msg);
void     wsPrintln(const String& msg);
void     initIMU();
void     zeroIMU();
float    getRawYaw();
float    getHeading();
float    shortestError(float target, float current);
float    wrapAngle(float a);
void     executeDrive(unsigned long durationMs, float targetHeading);
void     executeTurn(float targetAngle);
void     runForwardPID(float targetHeading, float &prevErr, float &integ);
void     runTurnPID(float targetAngle, float &prevErr, float &integ);
void     driveMotors(int leftSpeed, int rightSpeed);
void     turnClockwise(int speed);
void     turnAntiClockwise(int speed);
void     motorsStop();
void     setMotorPins();
void     pause(int ms);
void     initIntake();
void     intakeOn();
void     intakeOff();
void     initServo();
void     servoWrite(int angle);
void     servoShoot();

// ============================================================
//  WIFI + WEBSOCKET
// ============================================================

void wsPrint(const String& msg) {
  Serial.print(msg);
  ws.textAll(msg);
}

void wsPrintln(const String& msg) {
  Serial.println(msg);
  ws.textAll(msg + "\n");
}

void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! Open this in your browser: http://");
  Serial.println(WiFi.localIP());

  // Serve browser terminal
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", R"rawhtml(
<!DOCTYPE html><html><head>
<title>Robot Serial Monitor</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: #111; color: #0f0; font-family: monospace; display: flex; flex-direction: column; height: 100vh; }
  #toolbar { background: #1a1a1a; padding: 8px 12px; display: flex; align-items: center; gap: 12px; border-bottom: 1px solid #333; }
  #status  { font-size: 12px; color: #888; }
  #status.on { color: #0f0; }
  button   { background: #333; color: #ccc; border: none; padding: 4px 10px; border-radius: 4px; cursor: pointer; font-family: monospace; font-size: 12px; }
  button:hover { background: #444; }
  #log     { flex: 1; overflow-y: auto; padding: 10px 12px; white-space: pre-wrap; font-size: 13px; line-height: 1.5; }
</style></head><body>
<div id="toolbar">
  <span style="color:#0f0;font-weight:bold">Robot Monitor</span>
  <span id="status">● disconnected</span>
  <button onclick="clearLog()">Clear</button>
  <button onclick="copyLog()">Copy</button>
</div>
<div id="log"></div>
<script>
  const log = document.getElementById('log');
  const status = document.getElementById('status');
  const ws = new WebSocket('ws://' + location.host + '/ws');

  ws.onopen = () => {
    status.textContent = '● connected';
    status.className = 'on';
    append('[connected to robot]\n');
  };
  ws.onclose = () => {
    status.textContent = '● disconnected';
    status.className = '';
    append('[disconnected]\n');
  };
  ws.onmessage = e => append(e.data);

  function append(text) {
    log.textContent += text;
    log.scrollTop = log.scrollHeight;
  }
  function clearLog() { log.textContent = ''; }
  function copyLog()  { navigator.clipboard.writeText(log.textContent); }
</script></body></html>
)rawhtml");
  });

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType,
                void*, uint8_t*, size_t) {});
  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server started.");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(10);

  initWiFi();       // ← WiFi first so IP prints over USB

  setMotorPins();
  motorsStop();

  initIntake();
  initServo();

  initIMU();
  zeroIMU();

  wsPrintln("Ready. Starting in 2 seconds...");
  delay(2000);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  zeroIMU();

  intakeOn();
  wsPrintln("Intake ON");

  servoWrite(SERVO_INTAKE_POS);
  wsPrintln("Servo at 90deg");

  wsPrintln("=== SIDE 1: Forward @ 0deg ===");
  executeDrive(long_pause, 0.0);
  pause(pause_ms);

  wsPrintln("=== SHOOTER: 135deg → 90deg ===");
  servoShoot();
  pause(pause_ms);

  wsPrintln("=== TURN 1: To 90deg ===");
  executeTurn(90.0);
  pause(pause_ms);

  wsPrintln("=== SIDE 2: Forward @ 90deg ===");
  executeDrive(breadth_pause, 90.0);
  pause(pause_ms);

  wsPrintln("=== TURN 2: To 180deg ===");
  executeTurn(180.0);
  pause(pause_ms);

  wsPrintln("=== SIDE 3: Forward @ 180deg ===");
  executeDrive(long_pause, 180.0);
  pause(pause_ms);

  wsPrintln("=== TURN 3: To -90deg ===");
  executeTurn(-90.0);
  pause(pause_ms);

  wsPrintln("=== SIDE 4: Forward @ -90deg ===");
  executeDrive(breadth_pause, -90.0);
  pause(pause_ms);

  wsPrintln("=== TURN 4: Back to 0deg ===");
  executeTurn(0.0);
  pause(pause_ms);

  wsPrintln("=== RECTANGLE COMPLETE — RESTARTING ===");
}

// ============================================================
//  SERVO
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
  wsPrintln("Servo ready at 90deg.");
}

void servoShoot() {
  wsPrintln("Servo: 90 → 135deg");
  servoWrite(SERVO_SHOOT_POS);
  delay(250);
  wsPrintln("Servo: 135 → 90deg");
  servoWrite(SERVO_INTAKE_POS);
  delay(250);
}

// ============================================================
//  INTAKE
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
//  IMU
// ============================================================
void initIMU() {
  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  if (!bno.begin_I2C(0x4A, &Wire)) {
    wsPrintln("BNO085 not found!");
    while (1) delay(10);
  }
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  delay(200);
  wsPrintln("BNO085 Ready.");
}

void zeroIMU() {
  for (int i = 0; i < 20; i++) {
    bno.getSensorEvent(&imuData);
    delay(10);
  }
  yawOffset = getRawYaw();
  wsPrint("IMU Zeroed at: ");
  wsPrint(String(yawOffset, 2));
  wsPrintln("deg");
}

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

float wrapAngle(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ============================================================
//  DRIVE
// ============================================================
void executeDrive(unsigned long durationMs, float targetHeading) {
  float prevErr = 0.0, integ = 0.0;
  unsigned long startTime = millis(), lastPIDTime = millis();
  while (millis() - startTime < durationMs) {
    unsigned long now = millis();
    if (now - lastPIDTime >= FWD_INTERVAL) {
      lastPIDTime = now;
      runForwardPID(targetHeading, prevErr, integ);
      ws.cleanupClients();   // prevent stale socket buildup
    }
  }
  motorsStop();
}

void executeTurn(float targetAngle) {
  float prevErr = 0.0, integ = 0.0;
  int   stableCount = 0;
  unsigned long lastPIDTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - lastPIDTime >= TRN_INTERVAL) {
      lastPIDTime = now;
      ws.cleanupClients();   // prevent stale socket buildup

      float heading = getHeading();
      float error   = shortestError(targetAngle, heading);

      if (abs(error) <= TRN_DEADBAND) {
        motorsStop();
        integ = 0.0; prevErr = error;
        stableCount++;

        wsPrint("Stable " + String(stableCount) + "/" + String(TRN_STABLE_COUNT));
        wsPrint(" | H:" + String(heading, 1));
        wsPrint(" Target:" + String(targetAngle, 1));
        wsPrintln(" E:" + String(error, 1));

        if (stableCount >= TRN_STABLE_COUNT) {
          wsPrintln("Turn done. Heading: " + String(heading, 1) + "deg");
          return;
        }
      } else {
        stableCount = 0;
        runTurnPID(targetAngle, prevErr, integ);
      }
    }
  }
}

void runForwardPID(float targetHeading, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetHeading, heading);
  float dt      = FWD_INTERVAL / 1000.0;

  if (abs(error) <= FWD_DEADBAND) {
    integ = 0.0; prevErr = error;
    driveMotors(FWD_BASE_SPEED, FWD_BASE_SPEED);
    wsPrintln("FWD STRAIGHT | H:" + String(heading, 1) + "deg");
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

  wsPrint("FWD | H:" + String(heading, 1));
  wsPrint(" E:" + String(error, 1));
  wsPrint(" Corr:" + String(correction, 1));
  wsPrint(" L:" + String(leftSpeed));
  wsPrintln(" R:" + String(rightSpeed));
}

void runTurnPID(float targetAngle, float &prevErr, float &integ) {
  float heading = getHeading();
  float error   = shortestError(targetAngle, heading);
  float dt      = TRN_INTERVAL / 1000.0;

  integ += error * dt;
  integ  = constrain(integ, -100, 100);
  float derivative = (error - prevErr) / dt;
  prevErr = error;

  float output = (TRN_KP * error) + (TRN_KI * integ) + (TRN_KD * derivative);
  int speed = constrain((int)abs(output), TRN_MIN_SPEED, TRN_MAX_SPEED);

  if (error > 0) turnClockwise(speed);
  else           turnAntiClockwise(speed);

  wsPrint("TRN | H:" + String(heading, 1));
  wsPrint(" Target:" + String(targetAngle, 1));
  wsPrint(" E:" + String(error, 1));
  wsPrintln(" Spd:" + String(speed));
}

// ============================================================
//  MOTOR HELPERS
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  if      (leftSpeed > 0) { analogWrite(M1A, 0);          analogWrite(M1B, leftSpeed);  }
  else if (leftSpeed < 0) { analogWrite(M1A, -leftSpeed); analogWrite(M1B, 0);           }
  else                    { analogWrite(M1A, 0);          analogWrite(M1B, 0);           }

  if      (rightSpeed > 0) { analogWrite(M2A, rightSpeed); analogWrite(M2B, 0);           }
  else if (rightSpeed < 0) { analogWrite(M2A, 0);          analogWrite(M2B, -rightSpeed); }
  else                     { analogWrite(M2A, 0);          analogWrite(M2B, 0);           }
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

void pause(int ms) {
  motorsStop();
  delay(ms);
}