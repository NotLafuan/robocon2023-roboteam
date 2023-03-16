// #include <Arduino.h>
// #include <geometry_msgs/Twist.h>
// #include <math.h>
// #include "motor.h"

// class Kinematics
// {
// private:
//     Motor* motor1;
//     Motor* motor2;
//     Motor* motor3;
//     Motor* motor4;
//     double r;
//     double R;
//     double theta;

// public:
//     Kinematics(Motor motor1, Motor motor2, Motor motor3, Motor motor4);
//     ~Kinematics();
//     void move(geometry_msgs::Twist);
// };

// Kinematics::Kinematics(Motor motor1, Motor motor2, Motor motor3, Motor motor4)
// :
//     this->motor1 = new Motor(motor1);
//     this->motor2 = motor2;
//     this->motor3 = motor3;
//     this->motor4 = motor4;
// {}

// Kinematics::~Kinematics()
// {
// }

// void Kinematics::move(geometry_msgs::Twist twist)
// {
//     double w1 = (1 / r) * (-sin(theta + (1 * PI / 4)) * twist.linear.x + cos(theta + (1 * PI / 4)) * twist.linear.y + R * twist.angular.z);
//     double w2 = (1 / r) * (-sin(theta + (3 * PI / 4)) * twist.linear.x + cos(theta + (3 * PI / 4)) * twist.linear.y + R * twist.angular.z);
//     double w3 = (1 / r) * (-sin(theta + (5 * PI / 4)) * twist.linear.x + cos(theta + (5 * PI / 4)) * twist.linear.y + R * twist.angular.z);
//     double w4 = (1 / r) * (-sin(theta + (7 * PI / 4)) * twist.linear.x + cos(theta + (7 * PI / 4)) * twist.linear.y + R * twist.angular.z);
//     &motor1.setSpeed(w1);
//     motor2.setSpeed(w2);
//     motor3.setSpeed(w3);
//     motor4.setSpeed(w4);
// }