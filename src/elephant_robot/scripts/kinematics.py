#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
import numpy as np
from math import sin, cos, pi, radians
from dataclasses import dataclass
from utils import *


@dataclass
class Kinematics:
    manual_mode = True
    encoder_diameter = 6.0  # m
    speed_multiplier = 1/2

    angle = 0
    angleOffset = 0
    position = Position(0, 0)

    x, y, theta_dot = 0, 0, 0
    w1, w2, w3, w4 = 0, 0, 0, 0

    target_angle: float = 0
    target_x: float = 0
    target_y: float = 0

    PID_angle = PID(kp=20000,
                    ki=1,
                    kd=500,
                    target=0)
    PID_x = PID(kp=50,
                ki=0,
                kd=1,
                target=0)
    PID_y = PID(kp=50,
                ki=0,
                kd=1,
                target=0)

    def kinematics(self):
        r = 1
        R = 1
        array = [[-sin(self.angle+(1*pi/4)), cos(self.angle+(1*pi/4)), 1/2*R],
                 [-sin(self.angle+(3*pi/4)), cos(self.angle+(3*pi/4)), 1/2*R],
                 [-sin(self.angle+(5*pi/4)), cos(self.angle+(5*pi/4)), 1/2*R],
                 [-sin(self.angle+(7*pi/4)), cos(self.angle+(7*pi/4)), 1/2*R]]
        array = np.array(array)
        self.w1, self.w2, self.w3, self.w4 = \
            (1/r)*np.matmul(array, [self.x, self.y, self.theta_dot])

    def PID_theta(self):
        self.PID_angle.target = self.target_angle
        self.PID_angle.value = self.angle
        self.theta_dot = self.PID_angle.total

    def PID(self):
        self.PID_x.target = self.target_x
        self.PID_y.target = self.target_y
        self.PID_x.value = self.position.x
        self.PID_y.value = self.position.y
        self.x = self.PID_x.total
        self.y = -self.PID_y.total

    def update(self):
        self.PID_theta()
        if not self.manual_mode:
            self.PID()
        self.kinematics()

    def position_callback(self, pose: Pose):
        self.manual_mode = False
        self.target_x = pose.position.x
        self.target_y = pose.position.y
        self.target_angle = pose.orientation.z

    def cmd_vel_callback(self, twist: Twist):
        self.manual_mode = True
        self.x = twist.linear.x * self.speed_multiplier
        self.y = twist.linear.y * self.speed_multiplier
        self.target_angle = twist.angular.z

    def imu_callback(self, imu: Imu):
        try:
            quat = [imu.orientation.x,
                    imu.orientation.y,
                    imu.orientation.z,
                    imu.orientation.w,]
            rot = Rotation.from_quat(quat)
            rot_euler = rot.as_euler('xyz')
            self.angle = rot_euler[2]
        except ValueError:
            self.angle = 0

    def pose_callback(self, pose: Pose):
        self.position.x = pose.position.x
        self.position.y = pose.position.y


if __name__ == '__main__':
    rospy.init_node('kinematics')
    rospy.loginfo('Node started')

    kinematics = Kinematics()

    sub1 = rospy.Subscriber('position', Pose,
                            callback=kinematics.position_callback)
    sub2 = rospy.Subscriber('/elephant/cmd_vel', Twist,
                            callback=kinematics.cmd_vel_callback)
    sub3 = rospy.Subscriber('/imu/data', Imu,
                            callback=kinematics.imu_callback)
    sub3 = rospy.Subscriber('elephant_pose', Pose,
                            callback=kinematics.pose_callback)
    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)

    def shutdown_hook():
        pub1.publish(Int16(0))
        pub2.publish(Int16(0))
        pub3.publish(Int16(0))
        pub4.publish(Int16(0))
    rospy.on_shutdown(shutdown_hook)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        kinematics.update()
        pub1.publish(Int16(int(clamp(kinematics.w1, -32768, 32767))))
        pub2.publish(Int16(int(clamp(kinematics.w2, -32768, 32767))))
        pub3.publish(Int16(int(clamp(kinematics.w3, -32768, 32767))))
        pub4.publish(Int16(int(clamp(kinematics.w4, -32768, 32767))))
        rate.sleep()
        print('\r',
              f'{kinematics.w1:6.0f}',
              f'{kinematics.w2:6.0f}',
              f'{kinematics.w3:6.0f}',
              f'{kinematics.w4:6.0f}',
              f'{kinematics.position.x:6.0f}',
              f'{kinematics.position.y:6.0f}',
              f'{kinematics.angle:6.3f}',
              f'{kinematics.x:6.0f}', end='', flush=True)
