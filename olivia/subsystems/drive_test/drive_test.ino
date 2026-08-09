const int M1A = 6;   // Left motor
const int M1B = 7;
const int M2A = 15;   // Right motor
const int M2B = 16;
const int s_m = 128;

void motorsForward(int speed) {

  analogWrite(M1A, 0);      analogWrite(M1B, speed);
  analogWrite(M2A, speed);  analogWrite(M2B, 0);

}

void motorsBackward(int speed) {

    analogWrite(M1A, speed);  analogWrite(M1B, 0);
  analogWrite(M2A, 0);      analogWrite(M2B, speed);



}

void motorsClockwise(int speed) {
  // Left motor forward, right motor backward → spins chassis clockwise
  analogWrite(M1A, 0);  analogWrite(M1B, speed);
  analogWrite(M2A, 0);  analogWrite(M2B, speed);
}

void motorsAntiClockwise(int speed) {
  analogWrite(M1A, speed);  analogWrite(M1B, 0);
  analogWrite(M2A, speed);  analogWrite(M2B, 0);
}

void motorsStop() {
  analogWrite(M1A, 0);  analogWrite(M1B, 0);
  analogWrite(M2A, 0);  analogWrite(M2B, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  motorsStop();
}

void loop() {
  Serial.println("Forward");
  motorsForward(s_m);
  delay(2000);

  Serial.println("Stop");
  motorsStop();
  delay(500);

  Serial.println("Backward");
  motorsBackward(s_m);
  delay(2000);

  Serial.println("Stop");
  motorsStop();
  delay(500);

  Serial.println("Clockwise");
  motorsClockwise(s_m);
  delay(2000);

  Serial.println("Stop");
  motorsStop();
  delay(500);

  Serial.println("AntiClockwise");
  motorsAntiClockwise(s_m);
  delay(2000);

  Serial.println("Stop");
  motorsStop();
  delay(500);
}