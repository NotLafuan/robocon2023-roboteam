#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
import numpy as np
from math import sin, cos, pi, radians
from typing import NamedTuple
from dataclasses import dataclass
import time

@dataclass
class Position:
    x: float
    y: float


def map_value(x: float,  in_min: float,  in_max: float,  out_min: float,  out_max: float):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    value: float = 0
    target: float = 0
    _i: float = 0
    error_prev: float = 0
    time_prev: float = time.time()

    @property
    def time_diff(self):
        return time.time() - self.time_prev

    @property
    def error(self):
        return self.target - self.value

    @property
    def p(self):
        return self.kp * self.error

    @property
    def i(self):
        self._i = self._i + self.ki * self.error
        return self._i

    @property
    def d(self):
        return self.kd*((self.error - self.error_prev)/self.time_diff)

    @property
    def total(self):
        pid = self.p + self.i + self.d
        self.error_prev = self.error
        self.time_prev = time.time()
        return pid


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
                kd=0.5,
                target=200)
    PID_y = PID(kp=5,
                ki=0,
                kd=0.5,
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

    def cmd_vel_callback(self, twist: Twist):
        self.x = twist.linear.x
        self.y = twist.linear.y
        self.theta_dot = twist.angular.z

    def position_callback(self, twist: Twist):
        self.angle = twist.angular.z
        self.position.x = twist.linear.x
        self.position.y = twist.linear.y
        # if not self.angleOffset:
        #     self.angleOffset = -twist.angular.z
        # self.angle = twist.angular.z + self.angleOffset
        # self.angle = self.angle if self.angle > 0 else self.angle+360
        # self.angle = self.angle if self.angle < 180 else self.angle-360


if __name__ == '__main__':
    rospy.init_node('kinematics')
    rospy.loginfo('Node started')

    kinematics = Kinematics()

    sub = rospy.Subscriber('/elephant/cmd_vel',
                           Twist,
                           callback=kinematics.cmd_vel_callback)
    sub_position = rospy.Subscriber('position',
                                    Twist,
                                    callback=kinematics.position_callback)
    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        kinematics.update()
        mot1 = Int16()
        mot2 = Int16()
        mot3 = Int16()
        mot4 = Int16()
        mot1.data = int(kinematics.w1)
        mot2.data = int(kinematics.w2)
        mot3.data = int(kinematics.w3)
        mot4.data = int(kinematics.w4)
        pub1.publish(mot1)
        pub2.publish(mot2)
        pub3.publish(mot3)
        pub4.publish(mot4)
        # rospy.spin()
        rate.sleep()
