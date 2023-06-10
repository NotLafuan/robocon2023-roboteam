#include <Arduino.h>
// #include <AccelStepper.h>
#define USE_USBCON
#include <config.h>
#include <motor.h>
#include <encoder.h>
#include <AccelStepper.h>

// #include <WiFi.h>

const char *ssid = "22 Ground Floor 2.4Ghz";
const char *password = "1234abcd";
String serial_input = "";

//  Define the stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN_STEP);

void DropDown();
void DropUp();
void launch();

long prevT = 0;
int motorspeed1 = 0;
int motorspeed2 = 0;
double targetRPM = 0;

// ////////// MOTOR //////////
Motor motor1(LEFT1, RIGHT1);
Motor motor2(LEFT2, RIGHT2);
Motor motor3(LEFT3, RIGHT3);

///////// ENCODER /////////
Encoder encoder1(ENCODER1A, ENCODER1B, true);
Encoder encoder2(ENCODER2A, ENCODER2B, true);
void encoder1Update();
void encoder2Update();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FEED_MIN_LIMIT, INPUT_PULLUP);
  pinMode(FEED_MAX_LIMIT, INPUT_PULLUP);
  Serial.begin(9600);
  analogWriteResolution(16);

  // Set the maximum speed and acceleration for the stepper motor
  stepper.setPinsInverted(true, true, false);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  //   WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  // Serial.println("Connecting to WiFi...");
  // }
  // Serial.println("Connected to WiFi!");

  attachInterrupt(digitalPinToInterrupt(ENCODER1A), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1B), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2A), encoder2Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2B), encoder2Update, CHANGE);
}

void loop()
{
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if ((int)c == 13)
    {
      ;
    }

    else if ((int)c == 10)
    {
      Serial.println(serial_input);
      if (serial_input == "p")
      {
        launch();
      }
      else if (serial_input == "u")
      {
        DropUp();
      }
      else if (serial_input == "d")
      {
        DropDown();
      }
      else
      {
        targetRPM = serial_input.toDouble();
      }
      serial_input = "";
    }

    else
    {
      serial_input += c;
    }
  }

  // Serial.print("Encoder 1 = ");
  double ENC1_Value = encoder1.get_value();
  ENC1_Value = ((abs(ENC1_Value) * (1. / 1200.)) / (double)(millis() - prevT)) * 60000.;
  motorspeed1 += (targetRPM - ENC1_Value) * 0.1;
  if (motorspeed1 > 32767)
    motorspeed1 = 32767;
  // Serial.print(ENC1_Value);
  // Serial.print("\t");
  // Serial.print(motorspeed1);
  motor1.setSpeed(motorspeed1);

  // Serial.print("\t");

  // Serial.print("Encoder 2 = ");
  double ENC2_Value = encoder2.get_value();
  ENC2_Value = ((abs(ENC2_Value) * (1. / 1200.)) / (double)(millis() - prevT)) * 60000.;
  motorspeed2 += (targetRPM - ENC2_Value) * 0.1;
  if (motorspeed2 > 32767)
    motorspeed2 = 32767;
  // Serial.print(ENC2_Value);
  // Serial.print("\t");
  // Serial.print(motorspeed2);
  motor2.setSpeed(motorspeed2);

  // Serial.println("\t");
  prevT = millis();

  delay(20);
}

void encoder1Update()
{
  encoder1.encoderUpdate();
}

void encoder2Update()
{
  encoder2.encoderUpdate();
}
void DropDown()
{
  Serial.println("DropDown");

  int max = digitalRead(FEED_MAX_LIMIT);
  int min = digitalRead(FEED_MIN_LIMIT);
  while (max == LOW)
  {
    motor3.setSpeed(MAX_SPEED); // Run the motor at maximum speed
    max = digitalRead(FEED_MAX_LIMIT);
  }

  motor3.setSpeed(0);
}

void DropUp()
{
  Serial.println("DropUp");

  int max = digitalRead(FEED_MAX_LIMIT);
  int min = digitalRead(FEED_MIN_LIMIT);
  while (min == LOW)
  {
    motor3.setSpeed(-MAX_SPEED); // Run the motor at maximum speed
    max = digitalRead(FEED_MAX_LIMIT);
  }
}

void launch()
{
  Serial.println("Pneumatic");
  digitalWrite(PNEUM_PIN, HIGH);
  delay(1000);
  digitalWrite(PNEUM_PIN, LOW);
}