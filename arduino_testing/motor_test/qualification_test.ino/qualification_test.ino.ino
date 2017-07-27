#include <Servo.h>

Servo servo[8];
int neutral = 1500;
int down = 1600;
int power = 1750
int pins[8] = {1, 2, 3, 4, 5, 6, 7, 10, 9};

void setup() {
  Serial.begin(9600);
  servo.attach(pins);
  int i;
  for(i = 0; i < 8; i++) {
    servo.attach(pins[i]);
    servo[i].writeMicroseconds(neutral);
  }
  delay(1000);
  for(i = 4; i < 8; i++) {
    servo[i].writeMicroseconds(power);    
  }
  delay(3000);
}

void loop() {
  servo[0].writeMicroseconds(power);
  servo[1].writeMicroseconds(power);
  servo[4].writeMicroseconds(down);
  servo[5].writeMicroseconds(down);
  servo[6].writeMicroseconds(down);
  servo[7].writeMicroseconds(down);
  delay(60000);
}
