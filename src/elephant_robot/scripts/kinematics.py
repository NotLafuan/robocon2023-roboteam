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

    target_angle: float = 0

    PID_angle = PID(kp=20,
                    ki=0.01,
                    kd=5,
                    target=0)
    PID_x = PID(kp=5,
                ki=0,
                kd=1,
                target=200)
    PID_y = PID(kp=5,
                ki=0,
                kd=1,
                target=200)

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

    def PID(self):
        self.PID_angle.value = self.angle
        self.PID_x.value = self.position.x
        self.PID_y.value = self.position.y
        self.theta_dot = self.PID_angle.total
        self.x = self.PID_x.total
        self.y = self.PID_y.total

    def update(self):
        self.PID()
        self.kinematics()

    def position_callback(self, twist: Twist):
        self.position.x = twist.linear.x
        self.position.y = twist.linear.y
        self.angle = twist.angular.z
        # if not self.angleOffset:
        #     self.angleOffset = -twist.angular.z
        # self.angle = twist.angular.z + self.angleOffset
        # self.angle = self.angle if self.angle > 0 else self.angle+360
        # self.angle = self.angle if self.angle < 180 else self.angle-360


if __name__ == '__main__':
    rospy.init_node('kinematics')
    rospy.loginfo('Node started')

    kinematics = Kinematics()

    sub = rospy.Subscriber('position', Twist,
                           callback=kinematics.position_callback)
    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        kinematics.update()
        pub1.publish(Int16(int(kinematics.w1)))
        pub2.publish(Int16(int(kinematics.w2)))
        pub3.publish(Int16(int(kinematics.w3)))
        pub4.publish(Int16(int(kinematics.w4)))
        rate.sleep()
