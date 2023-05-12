#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from math import sin, cos, pi, radians
from typing import NamedTuple
from dataclasses import dataclass


class Position(NamedTuple):
    x: float
    y: float


def map_value(x: float,  in_min: float,  in_max: float,  out_min: float,  out_max: float):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


@dataclass
class Kinematics:
    encoder_diameter = 6.0  # m

    angle = 0
    angleOffset = 0
    position = Position(0, 0)

    current_motor1 = 0
    current_motor2 = 0
    current_motor3 = 0
    current_motor4 = 0
    current_pn = 0

    x, y, theta_dot = 0, 0, 0

    def motorUpdate(self):
        r = 1
        R = 1
        # if -1 < self.angle < 1:
        #     self.theta_dot = 0
        # elif self.angle < 0:
        #     self.theta_dot = map_value(-self.angle, 0, 180, 1000, 32768)
        # else:
        #     self.theta_dot = map_value(self.angle, 0, 180, -1900, -32768)
        w1 = (1 / r) * (-sin(radians(-self.angle) + (1 * pi / 4)) * self.x +
                        cos(radians(-self.angle) + (1 * pi / 4)) * self.y +
                        R * self.theta_dot)
        w2 = (1 / r) * (-sin(radians(-self.angle) + (3 * pi / 4)) * self.x +
                        cos(radians(-self.angle) + (3 * pi / 4)) * self.y +
                        R * self.theta_dot)
        w3 = (1 / r) * (-sin(radians(-self.angle) + (5 * pi / 4)) * self.x +
                        cos(radians(-self.angle) + (5 * pi / 4)) * self.y +
                        R * self.theta_dot)
        w4 = (1 / r) * (-sin(radians(-self.angle) + (7 * pi / 4)) * self.x +
                        cos(radians(-self.angle) + (7 * pi / 4)) * self.y +
                        R * self.theta_dot)

        self.current_motor1 = lerp(self.current_motor1, w1, 1)
        self.current_motor2 = lerp(self.current_motor2, w2, 1)
        self.current_motor3 = lerp(self.current_motor3, w3, 1)
        self.current_motor4 = lerp(self.current_motor4, w4, 1)
        rospy.loginfo(
            f'{self.current_motor1:8.2f} {self.current_motor2:8.2f} {self.current_motor3:8.2f} {self.current_motor4:8.2f} {self.angle}')
        # rospy.loginfo(self.angle)

    def cmd_vel_callback(self, twist: Twist):
        self.x = twist.linear.x
        self.y = twist.linear.y
        self.theta_dot = twist.angular.z
        self.current_pn = twist.linear.z

    def angle_callback(self, msg: Float32):
        if not self.angleOffset:
            self.angleOffset = -msg.data
        self.angle = msg.data + self.angleOffset
        self.angle = self.angle if self.angle > 0 else self.angle+360
        self.angle = self.angle if self.angle < 180 else self.angle-360


if __name__ == '__main__':
    rospy.init_node('kinematics')
    rospy.loginfo('Node started')

    kinematics = Kinematics()

    sub = rospy.Subscriber(
        '/elephant/cmd_vel', Twist, callback=kinematics.cmd_vel_callback)
    sub_angle = rospy.Subscriber(
        '/angle', Float32, callback=kinematics.angle_callback)
    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)
    pub5 = rospy.Publisher('pn', Int16, queue_size=10)


    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        kinematics.motorUpdate()
        mot1 = Int16()
        mot2 = Int16()
        mot3 = Int16()
        mot4 = Int16()
        pn = Int16()
        mot1.data = int(kinematics.current_motor1 * (10.5/(11.60)))
        mot2.data = int(kinematics.current_motor2 * (10.5/(11.00)))
        mot3.data = int(kinematics.current_motor3 * (10.5/(10.50)))
        mot4.data = int(kinematics.current_motor4 * (10.5/(10.75)))
        pn.data = int(kinematics.current_pn) 
        pub1.publish(mot1)
        pub2.publish(mot2)
        pub3.publish(mot3)
        pub4.publish(mot4)
        pub5.publish(pn)
        # rospy.spin()
        rate.sleep()
