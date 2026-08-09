// ============================================================
//  MDD 3A - Serial Monitor Controlled Intake Motor
//  Commands:
//    f        → Forward (last set speed)
//    b        → Backward (last set speed)
//    s        → Stop
//    s<0-255> → Set speed (e.g. s150)
//    f<0-255> → Forward at speed (e.g. f200)
//    b<0-255> → Backward at speed (e.g. b100)
// ============================================================

const int INTAKE_IN1 = 3;   // M2A (PWM pin)
const int INTAKE_IN2 = 8;   // M2B

int currentSpeed = 255;  // Default full speed

void setup() {
  Serial.begin(115200);
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);

  stopMotor();

  Serial.println("=============================");
  Serial.println(" Intake Motor Serial Control");
  Serial.println("=============================");
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.length() == 0) return;

    char dir = cmd.charAt(0);

    // Parse optional speed from command (e.g. f180, b200, s150)
    if (cmd.length() > 1) {
      int parsedSpeed = cmd.substring(1).toInt();
      if (parsedSpeed >= 0 && parsedSpeed <= 255) {
        currentSpeed = parsedSpeed;
      }
    }

    if (dir == 'f') {
      moveForward(currentSpeed);
      Serial.print(">> FORWARD | Speed: ");
      Serial.println(currentSpeed);

    } else if (dir == 'b') {
      moveBackward(currentSpeed);
      Serial.print(">> BACKWARD | Speed: ");
      Serial.println(currentSpeed);

    } else if (dir == 's') {
      if (cmd.length() == 1) {
        // Plain 's' = stop
        stopMotor();
        Serial.println(">> STOP");
      } else {
        // 's200' = set speed only, no direction change
        Serial.print(">> Speed set to: ");
        Serial.println(currentSpeed);
      }

    } else if (dir == 'h' || dir == '?') {
      printHelp();

    } else {
      Serial.print(">> Unknown command: '");
      Serial.print(cmd);
      Serial.println("' — type 'h' for help");
    }
  }
}

// ── Motor Control Functions ──────────────────────────────────

void moveForward(int speed) {
  analogWrite(INTAKE_IN1, 0);       // IN1 LOW
  analogWrite(INTAKE_IN2, speed);   // IN2 PWM
}

void moveBackward(int speed) {
  analogWrite(INTAKE_IN1, speed);   // IN1 PWM
  analogWrite(INTAKE_IN2, 0);       // IN2 LOW
}

void stopMotor() {
  analogWrite(INTAKE_IN1, 0);
  analogWrite(INTAKE_IN2, 0);
}

// ── Help Text ────────────────────────────────────────────────

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  f        → Forward at current speed");
  Serial.println("  b        → Backward at current speed");
  Serial.println("  s        → Stop motor");
  Serial.println("  f<speed> → Forward at speed 0-255  (e.g. f180)");
  Serial.println("  b<speed> → Backward at speed 0-255 (e.g. b100)");
  Serial.println("  s<speed> → Set speed without changing direction (e.g. s200)");
  Serial.println("  h or ?   → Show this help");
  Serial.println();
  Serial.print("Current speed: ");
  Serial.println(currentSpeed);
  Serial.println();
}