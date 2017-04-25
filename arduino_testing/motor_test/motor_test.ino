#include <Servo.h>

//byte servoPin[8] = {2, 3, 4, 5, 6, 7, 8, 9};
byte servoPin = 9;
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

}
