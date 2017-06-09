#include <Servo.h>

byte servoPin = 41;
int signal = 0;
Servo servo;
void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);

}

void loop() {
  if(Serial.available() > 0) {
    signal = Serial.parseInt();
    if(signal == 1) {
      digitalWrite(servoPin, HIGH);
    }
    else {
      digitalWrite(servoPin, LOW);
    }
    servo.writeMicroseconds(0);
  }
  delay(1000);

}
