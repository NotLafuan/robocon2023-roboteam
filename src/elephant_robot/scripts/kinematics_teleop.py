#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import numpy as np
from math import sin, cos, pi, radians
from dataclasses import dataclass
from utils import *


@dataclass
class Kinematics:
    encoder_diameter = 6.0  # m

    angle = 0
    angleOffset = 0
    position = Position(0, 0)

    x, y, theta_dot = 0, 0, 0
    w1, w2, w3, w4 = 0, 0, 0, 0

    def kinematics(self):
        r = 1
        R = 1
        theta = radians(-self.angle)
        array = [[-sin(theta+(1*pi/4)), cos(theta+(1*pi/4)), 1/2*R],
                 [-sin(theta+(3*pi/4)), cos(theta+(3*pi/4)), 1/2*R],
                 [-sin(theta+(5*pi/4)), cos(theta+(5*pi/4)), 1/2*R],
                 [-sin(theta+(7*pi/4)), cos(theta+(7*pi/4)), 1/2*R]]
        array = np.array(array)
        self.w1, self.w2, self.w3, self.w4 = \
            (1/r)*np.matmul(array, [self.x, self.y, self.theta_dot])

    def position_callback(self, twist: Twist):
        self.position.x = twist.linear.x
        self.position.y = twist.linear.y
        self.angle = twist.angular.z
        # if not self.angleOffset:
        #     self.angleOffset = -twist.angular.z
        # self.angle = twist.angular.z + self.angleOffset
        # self.angle = self.angle if self.angle > 0 else self.angle+360
        # self.angle = self.angle if self.angle < 180 else self.angle-360

    def cmd_vel_callback(self, twist: Twist):
        self.x = twist.linear.x
        self.y = twist.linear.y
        self.theta_dot = twist.angular.z


if __name__ == '__main__':
    rospy.init_node('kinematics')
    rospy.loginfo('Node started')

    kinematics = Kinematics()

    sub = rospy.Subscriber(
        '/elephant/cmd_vel', Twist, callback=kinematics.cmd_vel_callback)
    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        kinematics.kinematics()
        pub1.publish(Int16(int(kinematics.w1)))
        pub2.publish(Int16(int(kinematics.w2)))
        pub3.publish(Int16(int(kinematics.w3)))
        pub4.publish(Int16(int(kinematics.w4)))
        rate.sleep()
