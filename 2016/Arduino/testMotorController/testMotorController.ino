#include <Wire.h>

char forward = 'f'; //f = 102 ASCII
char reverse = 'r'; //r = 114 ASCII
int pwm = 255;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(9);
  Wire.write('f');
  Wire.write(pwm);
  Wire.endTransmission();
}
