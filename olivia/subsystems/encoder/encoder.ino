// ============================================================
//  ENCODER TEST — Single GB37 Encoder
//  Channel A → GPIO 4
//  Channel B → GPIO 5
//  Encoder VCC → 3.3V
//  Encoder GND → GND
// ============================================================

#define ENC_A  4 //4
#define ENC_B  5 //5

volatile long encCount = 0;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_B) == HIGH) encCount++;
  else                            encCount--;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.println("Encoder test started. Spin the motor...");
}

void loop() {
  static long lastCount = 0;

  if (encCount != lastCount) {
    lastCount = encCount;
    Serial.print("Count: ");
    Serial.println(encCount);
  }

  // delay(50);
}