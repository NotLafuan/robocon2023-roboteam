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

///////// ENCODER /////////
Encoder encoder1(ENCODER1A, ENCODER1B);
Encoder encoder2(ENCODER2A, ENCODER2B);

void encoder1Update();
void encoder2Update();

std_msgs::Int16 enc_msg;
ros::Publisher encPub1("encoder1", &enc_msg);
ros::Publisher encPub2("encoder2", &enc_msg);

///////// COMPASS /////////
HMC5883L hmc5883l;

std_msgs::Float32 hmc_msg;
ros::Publisher anglePub("angle", &hmc_msg);

void setup()
{
      pinMode(PC13, OUTPUT); // LED connect to pin PC13
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
    // encoder
    nh.advertise(encPub1);
    nh.advertise(encPub2);
    attachInterrupt(digitalPinToInterrupt(ENCODER1A), encoder1Update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER1B), encoder1Update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2A), encoder2Update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2B), encoder2Update, CHANGE);
    // compass
    nh.advertise(anglePub);
    hmc5883l.begin();
}

void loop()
{
    digitalWrite(PC13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for 100mS
  digitalWrite(PC13, LOW);    // turn the LED off by making the voltage LOW
  delay(100);
    nh.spinOnce();

    // encoder
    enc_msg.data = encoder1.get_value();
    encPub1.publish(&enc_msg);
    enc_msg.data = encoder2.get_value();
    encPub2.publish(&enc_msg);
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

void encoder1Update()
{
    encoder1.encoderUpdate();
}

void encoder2Update()
{
    encoder2.encoderUpdate();
}
void pn_callback (const std_msgs::Int16 &signal)
{
    digitalWrite(PN,signal.data);
}