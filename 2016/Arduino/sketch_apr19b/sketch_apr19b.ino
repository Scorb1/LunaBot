#include <ros.h>
#include <controller/controller_i2c.h>
#include <Wire.h>

ros::NodeHandle  nh;

void messageCb( const controller::controller_i2c& msg)
{
  Wire.beginTransmission(msg.address);
  Wire.write(msg.direction);
  Wire.write(msg.pwn);
  Wire.endTransmission();
}

ros::Subscriber<controller::controller_i2c> sub("i2c", &messageCb );

void setup()
{ 
  Wire.begin();
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay (1);
}
