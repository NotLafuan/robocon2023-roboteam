#include <AccelStepper.h>
// #define USE_USBCON
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <vesc_can_bus_arduino.h>
#include <config.h>

// VESC
CAN can;
float rpm = 0;

//  Define the stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN_STEP);

ros::NodeHandle nh;
void feeding();
void lowerGripper();
void homeLifter();
void moveLifter(float);
void launch();


void feedingCallback(const std_msgs::Empty &msg)
{
  stepper.setAcceleration(500);
  feeding();
}
void homeLifterCallback(const std_msgs::Empty &msg)
{
  homeLifter();
}
void moveLifterCallback(const std_msgs::Float32 &msg)
{
  stepper.setAcceleration(10000);
  moveLifter(msg.data);
}
void launchCallback(const std_msgs::Empty &msg)
{
  launch();
}
void vescRpmCallback(const std_msgs::Float32 &msg)
{
  rpm = msg.data;
}

ros::Subscriber<std_msgs::Empty> sub1("feeding", feedingCallback);
ros::Subscriber<std_msgs::Empty> sub2("home_lifter", homeLifterCallback);
ros::Subscriber<std_msgs::Float32> sub3("move_lifter", moveLifterCallback);
ros::Subscriber<std_msgs::Empty> sub4("launch", launchCallback);
ros::Subscriber<std_msgs::Float32> sub5("vesc_rpm", vescRpmCallback);

void delayWithVesc(uint32_t);

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
  // Initialize pneumatics pins
  pinMode(PNEUM_PIN, OUTPUT);

  // Set up PWM on PWM_PIN with a frequency of 1000Hz
  ledcSetup(0, 1000, 8);
  ledcAttachPin(PWM_PIN, 0);

  // VESC
  can.initialize();

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  delayWithVesc(1000);
}

void loop()
{
  nh.spinOnce();
  can.spin();
  can.vesc_set_erpm(rpm);
  delayWithVesc(50);
}

void delayWithVesc(uint32_t ms)
{
  long start_time = millis();
  while ((millis() - start_time) < ms)
  {
    can.vesc_set_erpm(rpm);
  }
}

void homeLifter()
{
  // Set the motor speed and direction for homing
  stepper.setSpeed(HOME_SPEED * HOME_DIRECTION);

  // Move the motor until the limit switch is triggered
  while (digitalRead(HOME_LIMIT) == LOW)
  {
    stepper.runSpeed();
  }

  // Stop the motor and move a small distance away from the limit switch
  stepper.stop();
  // stepper.runToPosition();
}

void moveLifter(float distance_mm)
{
  // Calculate the target position in steps
  int targetSteps = distance_mm / MM_PER_STEP;
  stepper.setSpeed(100);
  // Move the motor to the target position
  stepper.move(targetSteps);

  while (stepper.distanceToGo() > 0)
  {
    can.vesc_set_erpm(rpm);
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
    setMotorSpeed(MAX_SPEED); // Run the motor at maximum speed
    max = digitalRead(FEED_MAX_LIMIT);
    delayWithVesc(5);
  }

  stopMotor();
  delayWithVesc(700);
  setMotorDirection(HIGH); // Set motor direction to move towards the minimum limit switch
  min = digitalRead(FEED_MIN_LIMIT);
  while (min == HIGH)
  {
    setMotorSpeed(MAX_SPEED); // Run the motor at maximum speed
    min = digitalRead(FEED_MIN_LIMIT);
    delayWithVesc(5);
  }
  stopMotor();
}

void feeding()
{
  moveLifter(13);
  delayWithVesc(500);
  pushRing();
}

void launch()
{
  digitalWrite(PNEUM_PIN, HIGH);
  delayWithVesc(1000);
  digitalWrite(PNEUM_PIN, LOW);
}

void lowerGripper()
{
  homeLifter();
}

void pickup()
{
  moveLifter(300);
}
