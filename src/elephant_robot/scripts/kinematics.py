#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from math import sin, cos, pi, radians
from typing import NamedTuple


class Position(NamedTuple):
    x: float
    y: float


angle = 0
angleOffset = 0
position = [0, 0]
encoder_diameter = 0.6  # m
previous_encoder1 = 0
previous_encoder2 = 0

current_motor1 = 0
current_motor2 = 0
current_motor3 = 0
current_motor4 = 0

w1 = 0
w2 = 0
w3 = 0
w4 = 0

x, y, theta_dot = 0, 0, 0


def map_value(x: float,  in_min: float,  in_max: float,  out_min: float,  out_max: float):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def motorUpdate():
    global angle
    global current_motor1
    global current_motor2
    global current_motor3
    global current_motor4
    global x, y, theta_dot
    r = 1
    R = 1
    # global w1
    # global w2
    # global w3
    # global w4
    # theta_dot = -angle * 182
    if -2 < angle < 2:
        theta_dot = 0
    elif angle < 0:
        theta_dot = map_value(-angle, 0, 180, 11000, 32768)
    else:
        theta_dot = map_value(angle, 0, 180, -11000, -32768)
    # theta_dot = 10000
    w1 = (1 / r) * (-sin(angle + (1 * pi / 4)) * x +
                    cos(angle + (1 * pi / 4)) * y +
                    R * theta_dot)
    w2 = (1 / r) * (-sin(angle + (3 * pi / 4)) * x +
                    cos(angle + (3 * pi / 4)) * y +
                    R * theta_dot)
    w3 = (1 / r) * (-sin(angle + (5 * pi / 4)) * x +
                    cos(angle + (5 * pi / 4)) * y +
                    R * theta_dot)
    w4 = (1 / r) * (-sin(angle + (7 * pi / 4)) * x +
                    cos(angle + (7 * pi / 4)) * y +
                    R * theta_dot)

    current_motor1 = lerp(current_motor1, w1, 1)
    current_motor2 = lerp(current_motor2, w2, 1)
    current_motor3 = lerp(current_motor3, w3, 1)
    current_motor4 = lerp(current_motor4, w4, 1)
    # rospy.loginfo(
    #     f'{angle} {current_motor1:.0f}, {current_motor2:.0f}, {current_motor3:.0f}, {current_motor4:.0f}')
    rospy.loginfo(
        f'{angle} {theta_dot}')
    # rospy.loginfo(f'{w1:.0f}, {w2:.0f}, {w3:.0f}, {w4:.0f}')


def cmd_vel_callback(twist: Twist):
    global x, y, theta_dot
    global w1
    global w2
    global w3
    global w4
    # rospy.loginfo(f'{twist.linear.x}, {twist.linear.y}, {twist.angular.z}')
    # r = 1
    # R = 1
    x = twist.linear.x
    y = twist.linear.y
    theta_dot = twist.angular.z
    # w1 = (1 / r) * (-sin(angle + (1 * pi / 4)) * twist.linear.x +
    #                 cos(angle + (1 * pi / 4)) * twist.linear.y +
    #                 R * twist.angular.z)
    # w2 = (1 / r) * (-sin(angle + (3 * pi / 4)) * twist.linear.x +
    #                 cos(angle + (3 * pi / 4)) * twist.linear.y +
    #                 R * twist.angular.z)
    # w3 = (1 / r) * (-sin(angle + (5 * pi / 4)) * twist.linear.x +
    #                 cos(angle + (5 * pi / 4)) * twist.linear.y +
    #                 R * twist.angular.z)
    # w4 = (1 / r) * (-sin(angle + (7 * pi / 4)) * twist.linear.x +
    #                 cos(angle + (7 * pi / 4)) * twist.linear.y +
    #                 R * twist.angular.z)


def encoder1_callback(value: Int16):
    global previous_encoder1
    global angle
    global position
    change = value.data - previous_encoder1
    previous_encoder1 = value.data
    change = change * 2 * pi * (encoder_diameter/2)  # to meter
    position[0] = change * cos(angle)


def encoder2_callback(value: Int16):
    global previous_encoder2
    global angle
    global position
    change = value.data - previous_encoder2
    previous_encoder2 = value.data
    change = change * 2 * pi * (encoder_diameter/2)  # to meter
    position[1] = change * cos(angle)


def angle_callback(msg: Float32):
    global angleOffset
    global angle
    if not angleOffset:
        angleOffset = -msg.data
    # angle = msg.data + angleOffset
    angle = msg.data
    # angle = angle if angle > 0 else angle+360
    old_angle = angle
    angle = angle if angle < 180 else angle-360
    # rospy.loginfo(f'{old_angle} {angle}')


if __name__ == '__main__':
    rospy.init_node('kinematics')
    rospy.loginfo('Node started')

    sub = rospy.Subscriber(
        '/elephant/cmd_vel', Twist, callback=cmd_vel_callback)
    sub_enc1 = rospy.Subscriber(
        '/encoder1', Int16, callback=encoder1_callback)
    sub_enc2 = rospy.Subscriber(
        '/encoder1', Int16, callback=encoder2_callback)
    sub_angle = rospy.Subscriber('/angle', Float32, callback=angle_callback)
    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        motorUpdate()
        mot1 = Int16()
        mot2 = Int16()
        mot3 = Int16()
        mot4 = Int16()
        mot1.data = int(current_motor1)
        mot2.data = int(current_motor2)
        mot3.data = int(current_motor3)
        mot4.data = int(current_motor4)
        pub1.publish(mot1)
        pub2.publish(mot2)
        pub3.publish(mot3)
        pub4.publish(mot4)
        # rospy.spin()
        rate.sleep()
