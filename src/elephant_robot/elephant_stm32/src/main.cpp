#include <Arduino.h>

#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16.h>
#include "config.h"
#include "motor.h"
// #include "kinematics.h"

ros::NodeHandle nh;

Motor motor1(LEFT1, RIGHT1);
Motor motor2(LEFT2, RIGHT2);
Motor motor3(LEFT3, RIGHT3);
Motor motor4(LEFT4, RIGHT4);

void motor1_callback(const std_msgs::Int16 &speed);
void motor2_callback(const std_msgs::Int16 &speed);
void motor3_callback(const std_msgs::Int16 &speed);
void motor4_callback(const std_msgs::Int16 &speed);

ros::Subscriber<std_msgs::Int16> sub1("motor1", &motor1_callback);
ros::Subscriber<std_msgs::Int16> sub2("motor2", &motor2_callback);
ros::Subscriber<std_msgs::Int16> sub3("motor3", &motor3_callback);
ros::Subscriber<std_msgs::Int16> sub4("motor4", &motor4_callback);


void setup()
{
    nh.initNode();
    nh.subscribe(sub1);
    nh.subscribe(sub2);
    nh.subscribe(sub3);
    nh.subscribe(sub4);
    analogWriteResolution (16);
}

void loop()
{
    nh.spinOnce();
}

void motor1_callback(const std_msgs::Int16 &speed)
{
    motor1.setSpeed(speed.data);
}

void motor2_callback(const std_msgs::Int16 &speed)
{
    motor2.setSpeed(speed.data);
}

void motor3_callback(const std_msgs::Int16 &speed)
{
    motor3.setSpeed(speed.data);
}

void motor4_callback(const std_msgs::Int16 &speed)
{
    motor4.setSpeed(speed.data);
}