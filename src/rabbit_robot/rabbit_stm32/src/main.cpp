#include <Arduino.h>
// #include <AccelStepper.h>
#define USE_USBCON
#include <config.h>
#include <motor.h>
#include <encoder.h>
// #include <WiFi.h>

const char* ssid = "22 Ground Floor 2.4Ghz";
const char* password = "1234abcd";

long prevT = 0;

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

  //   WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }

  Serial.println("Connected to WiFi!");

  attachInterrupt(digitalPinToInterrupt(ENCODER1A), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1B), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2A), encoder2Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2B), encoder2Update, CHANGE);
}

void loop()
{
  Serial.print("Encoder 1 = ");
  double ENC1_Value = encoder1.get_value();
  ENC1_Value = ENC1_Value * (1 / 1200) * 6000 * (millis() - prevT);
  double targetRPM = 5000;
  int motorspeed1 = targetRPM - ENC1_Value;
  motor1.setSpeed(motorspeed1);

  Serial.print("\t");

  Serial.print("Encoder 2 = ");
  double ENC2_Value = encoder2.get_value();
  ENC2_Value = ENC2_Value * (1 / 1200) * 6000 * (millis() - prevT);
  int motorspeed2 = targetRPM - ENC2_Value;
  motor2.setSpeed(motorspeed2);

  Serial.println("\t");
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

  int max = digitalRead(FEED_MAX_LIMIT);
  int min = digitalRead(FEED_MIN_LIMIT);
  while (max == HIGH)
  {
    motor3.setSpeed(MAX_SPEED); // Run the motor at maximum speed
    max = digitalRead(FEED_MAX_LIMIT);
  }

  motor3.setSpeed(0);
}


void DropUp()
{
  int max = digitalRead(FEED_MAX_LIMIT);
  int min = digitalRead(FEED_MIN_LIMIT);
  while (min ==   HIGH)
  {
    motor3.setSpeed(-MAX_SPEED); // Run the motor at maximum speed
    max = digitalRead(FEED_MAX_LIMIT);
  }



}


void launch()
{
  digitalWrite(PNEUM_PIN, HIGH);
  delay(1000);
  digitalWrite(PNEUM_PIN, LOW);
}
