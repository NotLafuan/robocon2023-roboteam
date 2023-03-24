// #include <Arduino.h>

// #define USE_USBCON
// #include <ros.h>
// #include <geometry_msgs/Vector3.h>
// #include <std_msgs/Float32.h>
// #include <config.h>
// // #include <mpu6050.h>
// #include <hmc5883l.h>
// ros::NodeHandle nh;

// // geometry_msgs::Vector3 mpu_msg;
// std_msgs::Float32 hmc_msg;
// ros::Publisher pub1("angle", &hmc_msg);

// // MPU6050 mpu6050(MPU_SCL, MPU_SDA);
// HMC5883L hmc5883l(HMC_SCL, HMC_SDA);

// void encoderUpdate();

// void setup()
// {
//     // mag.begin();
//     // SerialUSB.begin(57600);
//     // nh.getHardware()->setBaud(57600);
//     nh.initNode();
//     nh.advertise(pub1);
//     // mpu6050.begin();
//     hmc5883l.begin();
// }

// void loop()
// {
//     hmc5883l.update();
//     // hmc_msg.x = hmc5883l.get_x();
//     // hmc_msg.y = hmc5883l.get_y();
//     // hmc_msg.z = hmc5883l.get_z();
//     // hmc_msg.data = hmc5883l.get_heading();
//     hmc_msg.data = hmc5883l.get_heading();
//     pub1.publish(&hmc_msg);
//     nh.spinOnce();
//     delay(20);
// }