#include <ESP32Servo.h>

#define SERVO_PIN 4
Servo myServo;

// after incrementing the value from 90 degrees, the flapper gors back , servo rotates anticlockwise 

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
// intyake 0...... shooting 50 ....... storing 145
      myServo.write(145);


      delay(2000);


      // myServo.write(0);




  Serial.println("Servo sweep starting...");
}

void loop() {


  // intake positio 90 and shooting position 135


  delay(1000); // pause at 180° before repeating
}