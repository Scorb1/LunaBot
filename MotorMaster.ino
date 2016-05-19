23jhX  XX#include <ros.h>
#include <controller/controller_i2c.h>
#include <Wire.h>

ros::NodeHandle  nh;

void messageCb( const controller::controller_i2c& msg)
{

  if(msg.address==0x04)
   {digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); }

  else{
  Wire.beginTransmission(msg.address);
  Wire.write(msg.direction);
  Wire.write(msg.pwn);
  Wire.endTransmission();
  }
}

ros::Subscriber<controller::controller_i2c> sub("i2c", &messageCb );

void setup()
{ 
  Wire.begin();
  nh.initNode();
  nh.subscribe(sub);
  pinMode(13, OUTPUT);
}

void loop()
{  
  nh.spinOnce();
  delay (1);
}
