#include <Arduino.h>

#define USE_USBCON
#include <ros.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <config.h>
#include <motor.h>
#include <encoder.h>

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

ros::Subscriber<std_msgs::Int16> sub1("motor1", &motor1_callback);
ros::Subscriber<std_msgs::Int16> sub2("motor2", &motor2_callback);
ros::Subscriber<std_msgs::Int16> sub3("motor3", &motor3_callback);
ros::Subscriber<std_msgs::Int16> sub4("motor4", &motor4_callback);

/////////// IMU ///////////
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensor_msgs::Imu imu_data;
ros::Publisher pub1("/imu/data", &imu_data);
void send_imu_data();

///////// ENCODER /////////
Encoder encoder1(ENCODER1A, ENCODER1B);
Encoder encoder2(ENCODER2A, ENCODER2B);
geometry_msgs::Pose pose;
ros::Publisher pub2("elephant_pose", &pose);
double heading = 0;
void encoder1Update();
void encoder2Update();
void send_pose_data();
double distance_per_count = 0.157; 

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
    analogWriteResolution(16);
    // bno055
    nh.advertise(pub1);
    bno.begin();
    delay(100);
    // encoder
    nh.advertise(pub2);
    attachInterrupt(digitalPinToInterrupt(ENCODER1A), encoder1Update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER1B), encoder1Update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2A), encoder2Update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2B), encoder2Update, CHANGE);
}

void loop()
{
    nh.spinOnce();
    send_imu_data();
    send_pose_data();
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

void send_imu_data()
{
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Quaternion quat = bno.getQuat();
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    heading = orientationData.orientation.x;

    imu_data.linear_acceleration.x = acc.x();
    imu_data.linear_acceleration.y = acc.y();
    imu_data.linear_acceleration.z = acc.z();
    imu_data.orientation.w = quat.w();
    imu_data.orientation.x = quat.x();
    imu_data.orientation.y = quat.y();
    imu_data.orientation.z = quat.z();

    pub1.publish(&imu_data);
}

void encoder1Update()
{
    encoder1.encoderUpdate();
}

void encoder2Update()
{
    encoder2.encoderUpdate();
}

void send_pose_data()
{
    double encoder1_value = (double)encoder1.get_value();
    pose.position.y += pose.position.y + encoder1_value*sin(heading*0.01745329251);
    pose.position.x += pose.position.x + encoder1_value*cos(heading*0.01745329251);
    // enc_msg.data = encoder1.get_value();
    // pub2.publish(&enc_msg);
    // enc_msg.data = encoder2.get_value();
    // pub3.publish(&enc_msg);
}
