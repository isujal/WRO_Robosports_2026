// ============================================================
//  Motor Serial Monitor Control
//  Commands:
//    f        → Forward  at current speed
//    b        → Backward at current speed
//    c        → Clockwise at current speed
//    a        → AntiClockwise at current speed
//    s        → Stop
//    f<speed> → Forward  at speed 0-255  (e.g. f180)
//    b<speed> → Backward at speed 0-255  (e.g. b200)
//    c<speed> → Clockwise at speed 0-255 (e.g. c128)
//    a<speed> → AntiClockwise            (e.g. a100)
//    s<speed> → Set speed only           (e.g. s200)
// ============================================================

const int M1A = 6;
const int M1B = 7;
const int M2A = 15;
const int M2B = 16;

int currentSpeed = 128;  // Default speed

// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  motorsStop();

  Serial.println("==============================");
  Serial.println("  Motor Serial Monitor Control");
  Serial.println("==============================");
  printHelp();
}

// ============================================================
void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.length() == 0) return;

    char dir = cmd.charAt(0);

    // Parse optional speed from command (e.g. f180, b200)
    if (cmd.length() > 1) {
      int parsed = cmd.substring(1).toInt();
      if (parsed >= 0 && parsed <= 255) {
        currentSpeed = parsed;
      }
    }

    if (dir == 'f') {
      motorsForward(currentSpeed);
      Serial.print(">> FORWARD | Speed: "); Serial.println(currentSpeed);

    } else if (dir == 'b') {
      motorsBackward(currentSpeed);
      Serial.print(">> BACKWARD | Speed: "); Serial.println(currentSpeed);

    } else if (dir == 'c') {
      motorsClockwise(currentSpeed);
      Serial.print(">> CLOCKWISE | Speed: "); Serial.println(currentSpeed);

    } else if (dir == 'a') {
      motorsAntiClockwise(currentSpeed);
      Serial.print(">> ANTICLOCKWISE | Speed: "); Serial.println(currentSpeed);

    } else if (dir == 's') {
      if (cmd.length() == 1) {
        motorsStop();
        Serial.println(">> STOP");
      } else {
        Serial.print(">> Speed set to: "); Serial.println(currentSpeed);
      }

    } else if (dir == 'h' || dir == '?') {
      printHelp();

    } else {
      Serial.print(">> Unknown command: '");
      Serial.print(cmd);
      Serial.println("'  —  type h for help");
    }
  }
}

// ============================================================
//  Motor Functions
// ============================================================
void motorsForward(int speed) {
  analogWrite(M1A, 0);      analogWrite(M1B, speed);
  analogWrite(M2A, speed);  analogWrite(M2B, 0);
}

void motorsBackward(int speed) {
  analogWrite(M1A, speed);  analogWrite(M1B, 0);
  analogWrite(M2A, 0);      analogWrite(M2B, speed);
}

void motorsClockwise(int speed) {
  analogWrite(M1A, 0);      analogWrite(M1B, speed);
  analogWrite(M2A, 0);      analogWrite(M2B, speed);
}

void motorsAntiClockwise(int speed) {
  analogWrite(M1A, speed);  analogWrite(M1B, 0);
  analogWrite(M2A, speed);  analogWrite(M2B, 0);
}

void motorsStop() {
  analogWrite(M1A, 0);  analogWrite(M1B, 0);
  analogWrite(M2A, 0);  analogWrite(M2B, 0);
}

// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("Commands (set baud 115200, line ending: Newline):");
  Serial.println("  f        → Forward");
  Serial.println("  b        → Backward");
  Serial.println("  c        → Clockwise");
  Serial.println("  a        → AntiClockwise");
  Serial.println("  s        → Stop");
  Serial.println("  f<0-255> → Forward at speed    e.g. f200");
  Serial.println("  b<0-255> → Backward at speed   e.g. b150");
  Serial.println("  c<0-255> → Clockwise at speed  e.g. c128");
  Serial.println("  a<0-255> → AntiCW at speed     e.g. a100");
  Serial.println("  s<0-255> → Set speed only       e.g. s180");
  Serial.println("  h or ?   → Show this help");
  Serial.println();
  Serial.print("Current speed: "); Serial.println(currentSpeed);
  Serial.println();
}