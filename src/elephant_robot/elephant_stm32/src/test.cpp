// #include <Arduino.h>

// #define USE_USBCON
// #include <ros.h>
// #include <std_msgs/Int16.h>
// #include <config.h>
// #include <encoder.h>

// ros::NodeHandle nh;

// std_msgs::Int16 enc_msg1;
// std_msgs::Int16 enc_msg2;
// ros::Publisher pub1("pub1", &enc_msg1);
// ros::Publisher pub2("pub2", &enc_msg2);

// // volatile int Encoder::value;
// Encoder encoder1(ENCODER1A, ENCODER1B);
// Encoder encoder2(ENCODER2A, ENCODER2B);

// void encoderUpdate();

// void setup()
// {
//     // SerialUSB.begin(57600);
//     // nh.getHardware()->setBaud(57600);
//     nh.initNode();
//     nh.advertise(pub1);
//     nh.advertise(pub2);
//     attachInterrupt(digitalPinToInterrupt(ENCODER1A), encoderUpdate, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(ENCODER1B), encoderUpdate, CHANGE);
// }

// void loop()
// {
//     enc_msg1.data = encoder1.get_value();
//     pub1.publish(&enc_msg1);
//     pub2.publish(&enc_msg2);
//     nh.spinOnce();
//     delay(20);
// }

// void encoderUpdate()
// {
//     encoder1.encoderUpdate();
// }