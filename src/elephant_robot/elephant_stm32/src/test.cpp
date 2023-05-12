// #include <Arduino.h>
// #include <config.h>
// #include <motor.h>
// #include <encoder.h>

// // 11.5
// // 11
// // 10.5
// // 10.75

// Motor motor1(RIGHT1, LEFT1);
// Motor motor2(LEFT2, RIGHT2);
// Motor motor3(RIGHT3, LEFT3);
// Motor motor4(LEFT4, RIGHT4);
// Encoder encoder(ENCODER1A, ENCODER1B);
// void encoder1update();
// void setup()
// {
//     SerialUSB.begin(9600);
//     SerialUSB.println("start");
//     attachInterrupt(digitalPinToInterrupt(ENCODER1A), encoder1update, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(ENCODER1B), encoder1update, CHANGE);
//     delay(500);
//     analogWriteResolution(16);
//     motor1.setSpeed(5000 * (10.5/11.50));
//     motor2.setSpeed(5000 * (10.5/11.00));
//     motor3.setSpeed(5000 * (10.5/10.50));
//     motor4.setSpeed(5000 * (10.5/10.75));
//     // motor1.setSpeed(5000);
//     delay(10 * 1000);
//     motor1.setSpeed(0);
//     motor2.setSpeed(0);
//     motor3.setSpeed(0);
//     motor4.setSpeed(0);
// }

// void encoder1update()
// {
//     encoder.encoderUpdate();
// }

// void loop()
// {
//     SerialUSB.println(encoder.get_value());
// }