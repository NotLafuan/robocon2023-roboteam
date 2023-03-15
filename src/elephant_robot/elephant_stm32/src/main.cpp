#include <Arduino.h>
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <ArduinoHardware.h>
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  SerialUSB.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(chatter);
  pinMode(PA1, OUTPUT);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1000);
  digitalWrite(PA1, HIGH);
  delay(1000);
  digitalWrite(PA1, LOW);
}
