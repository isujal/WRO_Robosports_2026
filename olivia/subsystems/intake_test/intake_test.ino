// ============================================================
//  MDD 3A - Raw Pin Wiggle Test
//  Bypasses all functions — directly drives pins HIGH/LOW
//  If motor still doesn't move, problem is hardware/wiring
//  If motor moves now, problem was in previous code logic
// ============================================================

const int INTAKE_IN1 = 3;   // M2A
const int INTAKE_IN2 = 8;  // M2B

void setup() {
  Serial.begin(115200);
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);

  Serial.println("Pin wiggle test starting...");
  Serial.println("Motor should run FORWARD for 2s, STOP 1s, BACKWARD 2s, STOP");
}

void loop() {
  // FORWARD
  Serial.println("FORWARD");
  digitalWrite(INTAKE_IN1, LOW);
  digitalWrite(INTAKE_IN2, HIGH);
  delay(2000);

  // // STOP
  // Serial.println("STOP");
  // digitalWrite(INTAKE_IN1, LOW);
  // digitalWrite(INTAKE_IN2, LOW);
  // delay(1000);

  // // BACKWARD
  // Serial.println("BACKWARD");
  // digitalWrite(INTAKE_IN1, LOW);
  // digitalWrite(INTAKE_IN2, HIGH);
  // delay(2000);

  // // STOP
  // Serial.println("STOP");
  // digitalWrite(INTAKE_IN1, LOW);
  // digitalWrite(INTAKE_IN2, LOW);
  // delay(1000);
}