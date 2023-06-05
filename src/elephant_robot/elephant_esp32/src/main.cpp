#include <AccelStepper.h>
// #define USE_USBCON
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
// #include <ezButton.h>
const int PNEUM_PIN = 16;
// Pin configuration for TB6600
const int STEP_PIN = 13;     // Step pin
const int DIR_PIN_STEP = 12; // Direction pin
const int STEP_LIMIT = 4;
const float MM_PER_STEP = 0.3675;

// Home sequence configuration
const int HOME_SPEED = 100;    // Speed for homing movement (steps per second)
const int HOME_DIRECTION = -1; // Direction for homing movement (-1 = counterclockwise)

// Pin configuration for Cytron MD13
const int DIR_PIN_MD = 26; // Direction pin
const int PWM_PIN = 25;    // PWM pin
const int MAX_SPEED = 50;
const int firstRing = 560;
const int FEED_MIN_LIMIT = 32; // Minimum limit switch pin
const int FEED_MAX_LIMIT = 33; // Maximum limit switch pin
//  Define the stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN_STEP);

ros::NodeHandle nh;
void feeding();
void lowerGripper();
void homeLifter();
void moveLifter(float);

void feedingCallback(const std_msgs::Empty &msg)
{
  feeding();
}
void homeLifterCallback(const std_msgs::Empty &msg)
{
  homeLifter();
}
void moveLifterCallback(const std_msgs::Float32 &msg)
{
  moveLifter(msg.data);
}

ros::Subscriber<std_msgs::Empty> sub1("feeding", feedingCallback);
ros::Subscriber<std_msgs::Empty> sub2("home_lifter", homeLifterCallback);
ros::Subscriber<std_msgs::Float32> sub3("move_lifter", moveLifterCallback);

bool dir = true;
void setup()
{
  pinMode(FEED_MIN_LIMIT, INPUT_PULLUP);
  pinMode(FEED_MAX_LIMIT, INPUT_PULLUP);
  //  Set the maximum speed and acceleration for the stepper motor

  stepper.setPinsInverted(true, true, false);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Initialize motor pins
  pinMode(DIR_PIN_MD, OUTPUT);

  // Set up PWM on PWM_PIN with a frequency of 1000Hz
  ledcSetup(0, 1000, 8);
  ledcAttachPin(PWM_PIN, 0);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  delay(1000);
}

void loop()
{
  nh.spinOnce();
  delay(50);
}
void homeLifter()
{
  // Set the motor speed and direction for homing
  stepper.setSpeed(HOME_SPEED * HOME_DIRECTION);

  // Move the motor until the limit switch is triggered
  while (digitalRead(FEED_MIN_LIMIT) == HIGH)
  {
    stepper.runSpeed();
    // stepper.run();
  }

  // Stop the motor and move a small distance away from the limit switch
  stepper.stop();
  // Serial.println(stepper.currentPosition());
  // stepper.move(-HOME_DIRECTION * 200);
  stepper.runToPosition();
}

void moveLifter(float distance_mm)
{
  // Calculate the target position in steps
  int targetSteps = distance_mm / MM_PER_STEP;
  if (distance_mm < 0)
    stepper.setSpeed(-100);
  else
    stepper.setSpeed(100);
  // Move the motor to the target position
  stepper.move(targetSteps);

  while (stepper.distanceToGo() > 0)
  {
    stepper.runSpeed();
  }
}

void setMotorDirection(bool direction)
{
  digitalWrite(DIR_PIN_MD, direction);
}

void setMotorSpeed(int speed)
{
  ledcWrite(0, speed);
}

void stopMotor()
{
  setMotorSpeed(0);
}
void pushRing()
{

  int max = digitalRead(FEED_MAX_LIMIT);
  int min = digitalRead(FEED_MIN_LIMIT);
  setMotorDirection(LOW); // Set motor direction to move towards the maximum limit switch
  while (max == HIGH)
  {
    // FEED_MAX_LIMIT.loop();
    // Serial.println(max);
    setMotorSpeed(MAX_SPEED); // Run the motor at maximum speed
    max = digitalRead(FEED_MAX_LIMIT);
    delay(5);
  }

  stopMotor();
  delay(700);
  // Serial.println("max feed hit");
  setMotorDirection(HIGH); // Set motor direction to move towards the minimum limit switch
  min = digitalRead(FEED_MIN_LIMIT);
  while (min == HIGH)
  {

    // FEED_MIN_LIMIT.loop();
    setMotorSpeed(MAX_SPEED); // Run the motor at maximum speed
    min = digitalRead(FEED_MIN_LIMIT);
    delay(5);
  }
  stopMotor();
}
void feeding()
{
  moveLifter(11);
  delay(500);

  pushRing();
}

void launch()
{
  digitalWrite(PNEUM_PIN, HIGH);
}

void lowerGripper()
{
  homeLifter();
}

void pickup()
{
  moveLifter(300);
}