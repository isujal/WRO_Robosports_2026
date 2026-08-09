// ============================================================
//  Servo Serial Monitor Test
//  Uses ledcAttach / ledcWrite — 14-bit LEDC (same as main code)
//  Enter angle (0–180) in Serial Monitor → servo moves instantly
// ============================================================

#define SERVO_PIN 4

constexpr uint32_t SERVO_PWM_FREQ = 50;
constexpr uint8_t  SERVO_PWM_RES  = 14;
constexpr uint16_t SERVO_MIN_DUTY = (uint16_t)((500  * 16384L) / 20000);
constexpr uint16_t SERVO_MAX_DUTY = (uint16_t)((2400 * 16384L) / 20000);

// ── Same servoWrite() as vision_integrated.ino ───────────────
void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t duty = map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
  ledcWrite(SERVO_PIN, duty);
}

void initServo() {
  ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_RES);
  servoWrite(0);
  delay(500);
}

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  initServo();

  Serial.println("Servo ready. Known positions:");
  Serial.println("  0   → Neutral / Intake");
  Serial.println("  50  → Shoot");
  Serial.println("  140 → Load");
  Serial.println("Enter any angle (0–180) and press Enter:");
}

// ============================================================
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int angle = input.toInt();

    if (input.length() == 0 || (angle == 0 && input != "0")) {
      Serial.println("Invalid input. Enter a number between 0 and 180.");
      return;
    }

    if (angle < 0 || angle > 180) {
      Serial.print("Out of range: ");
      Serial.print(angle);
      Serial.println("  →  Must be 0 to 180.");
      return;
    }

    servoWrite(angle);
    Serial.print("Servo → ");
    Serial.print(angle);
    Serial.println("°");
  }
}