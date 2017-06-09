#include <Servo.h>

byte servoPin = 10;
Servo servo;
int power = 1500;


void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.writeMicroseconds(1500);
  delay(1000);

}

void loop() {
  if(Serial.available() > 0) {
    power = Serial.parseInt();
  }
  Serial.println(power, DEC);
  servo.writeMicroseconds(power);
  //delay(1000);
  //servo.writeMicroseconds(1500);

}
