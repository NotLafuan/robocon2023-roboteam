#include <Arduino.h>

#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <config.h>
#include <motor.h>
#include <encoder.h>
#include <hmc5883l.h>

ros::NodeHandle nh;

////////// MOTOR //////////
Motor motor1(LEFT1, RIGHT1);
Motor motor2(LEFT2, RIGHT2);
Motor motor3(LEFT3, RIGHT3);
Motor motor4(LEFT4, RIGHT4);

void motor1_callback(const std_msgs::Int16 &speed);
void motor2_callback(const std_msgs::Int16 &speed);
void motor3_callback(const std_msgs::Int16 &speed);
void motor4_callback(const std_msgs::Int16 &speed);
void pn_callback(const std_msgs::Int16 &UART_ONEBIT_SAMPLING_ENABLED);

ros::Subscriber<std_msgs::Int16> sub1("motor1", &motor1_callback);
ros::Subscriber<std_msgs::Int16> sub2("motor2", &motor2_callback);
ros::Subscriber<std_msgs::Int16> sub3("motor3", &motor3_callback);
ros::Subscriber<std_msgs::Int16> sub4("motor4", &motor4_callback);
ros::Subscriber<std_msgs::Int16> subpn("pn", &pn_callback);

///////// COMPASS /////////
HMC5883L hmc5883l;

std_msgs::Float32 hmc_msg;
ros::Publisher anglePub("angle", &hmc_msg);

void setup()
{
    Wire.setSCL(SCL_PIN);
    Wire.setSDA(SDA_PIN);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    nh.initNode();
    // motor
    nh.subscribe(sub1);
    nh.subscribe(sub2);
    nh.subscribe(sub3);
    nh.subscribe(sub4);
    nh.subscribe(subpn);
    analogWriteResolution(16);
    // compass
    nh.advertise(anglePub);
    hmc5883l.begin();
}

void loop()
{
    nh.spinOnce();
    // compass
    hmc5883l.update();
    hmc_msg.data = hmc5883l.get_heading();
    anglePub.publish(&hmc_msg);

    delay(20);
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
