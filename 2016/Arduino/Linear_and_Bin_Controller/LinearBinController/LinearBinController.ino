#include <ros.h>
#include <controller/controller_i2c.h>
#include <Wire.h>

#define ADDRESS 0x01
#define POWER_LA_PIN 0
#define POWER_BN_PIN 1
#define DIR_LA_PIN 2
#define DIR_BN_PIN 3

char command = ' ';
int pwm = 0;

void setup() {
  pinMode(POWER_LA_PIN, OUTPUT); // on/off linear actuators
  pinMode(POWER_BN_PIN, OUTPUT); // direction linear actuators
  pinMode(DIR_LA_PIN, OUTPUT); // on/off bin actuators
  pinMode(DIR_BN_PIN, OUTPUT); // direction bin actuators
  
  Wire.begin(ADDRESS);
  Wire.onReceive(receiveData);
}

void loop() {
  delayMicroseconds(10);
}

void receiveData(int size) {
  while(Wire.available()) {
    command = char(Wire.read());
    pwm = int(Wire.read());

    update();
  }
}

void update() {
  if(command == 'q') { // move bin actuator up
    digitalWrite(DIR_BN_PIN, HIGH);
    digitalWrite(POWER_BN_PIN, HIGH);
  }
  else if(command == 'a') { // move bin actuator down
    digitalWrite(DIR_BN_PIN, LOW);
    digitalWrite(POWER_BN_PIN, HIGH);
  }
  else if(command == 'w') { // move linear actuator up
    digitalWrite(DIR_LA_PIN, HIGH);
    digitalWrite(POWER_LA_PIN, HIGH);
  }
  else if(command == 's') { // move linear actuator down
    digitalWrite(DIR_LA_PIN, LOW);
    digitalWrite(POWER_LA_PIN, HIGH);
  }
  else if(command == ' ') { // stop movement
    digitalWrite(POWER_BN_PIN, LOW);
    digitalWrite(POWER_LA_PIN, LOW);
  }
}
