#include <Arduino.h>
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Empty.h>

// Pin configuration for TB6600
const int STEP_PIN = 25; // Step pin
const int DIR_PIN = 26;  // Direction pin
const int LIMIT_PIN = 27;
// Motor configuration
// const float MM_PER_REV = 72;                // Distance in millimeters per revolution
// const int STEPS_PER_REV = 200;               // Steps per revolution
// const float MM_PER_STEP = MM_PER_REV / STEPS_PER_REV;  // Distance in millimeters per step
const float MM_PER_STEP = 44.44;

// Home sequence configuration
const int HOME_SPEED = 100;    // Speed for homing movement (steps per second)
const int HOME_DIRECTION = 1; // Direction for homing movement (-1 = counterclockwise)

// Pin configuration for Cytron MD13
const int DIR_PIN1 = 12; // Direction pin
const int PWM_PIN = 13;  // PWM pin
const int MAX_SPEED = 255;

const int MIN_LIMIT_PIN = 32; // Minimum limit switch pin
const int MAX_LIMIT_PIN = 33; // Maximum limit switch pin
// Define the stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

ros::NodeHandle nh;

void feeding();
void lowerGripper();
void homeMotor();
void homeFeeder();
void moveMotor(float);

// ROS callback function to feed ring to launcher
void feedCallback(const std_msgs::Empty &msg)
{
  feeding();
}
// ROS callback function to lower gripper
void lowerGripperCallback(const std_msgs::Empty &msg)
{
  lowerGripper();
}
// ROS callback function to pickup ring
void pickupCallback(const std_msgs::Empty &msg)
{
  feeding();
}

ros::Subscriber<std_msgs::Empty> sub1("lower_gripper", lowerGripperCallback);
ros::Subscriber<std_msgs::Empty> sub2("pickup_ring", pickupCallback);
ros::Subscriber<std_msgs::Empty> sub3("feed", feedCallback);

void setup()
{
  // Set the maximum speed and acceleration for the stepper motor
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Initialize motor pins
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(MIN_LIMIT_PIN, INPUT_PULLUP);

  // Set up PWM on PWM_PIN with a frequency of 1000Hz
  ledcSetup(0, 1000, 8);
  ledcAttachPin(PWM_PIN, 0);

  // Perform homing sequence
  homeMotor();
  homeFeeder();
  // Move motor by 100 millimeters
  moveMotor(100.0);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
}

void loop()
{
  nh.spinOnce();
  delay(20);
}
void homeFeeder()
{
  // TODO
}
void homeMotor()
{
  // Set the motor speed and direction for homing
  stepper.setSpeed(HOME_SPEED * HOME_DIRECTION);
  // Move the motor until the limit switch is triggered
  while (digitalRead(MIN_LIMIT_PIN) == HIGH)
  {
    stepper.runSpeed();
  }

  // Stop the motor and move a small distance away from the limit switch
  // stepper.stop();
  // stepper.move(HOME_DIRECTION * 200);
  // stepper.runToPosition();
}

void moveMotor(float distance_mm)
{
  // Calculate the target position in steps
  int targetSteps = distance_mm / MM_PER_STEP;

  // Move the motor to the target position
  stepper.move(targetSteps);
  stepper.runToPosition();
}

void setMotorDirection(bool direction)
{
  digitalWrite(DIR_PIN, direction);
}

void setMotorSpeed(int speed)
{
  ledcWrite(0, speed);
}

void stopMotor()
{
  setMotorSpeed(0);
}

void feeding()
{
  moveMotor(13);
  delay(500);

  setMotorDirection(HIGH); // Set motor direction to move towards the maximum limit switch

  while (digitalRead(MAX_LIMIT_PIN) == LOW)
  {
    setMotorSpeed(MAX_SPEED); // Run the motor at maximum speed
  }

  stopMotor();
  setMotorDirection(LOW); // Set motor direction to move towards the minimum limit switch
  while (digitalRead(MIN_LIMIT_PIN) == LOW)
  {
    setMotorSpeed(MAX_SPEED); // Run the motor at maximum speed
  }
}

void lowerGripper()
{
  homeMotor();
}

void pickup()
{
  moveMotor(300);
}
