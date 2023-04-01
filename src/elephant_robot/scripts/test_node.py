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


if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo('Node started')

    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        mot1 = Int16()
        mot2 = Int16()
        mot3 = Int16()
        mot4 = Int16()
        mot1.data = int(5000 * (10.5/(11.60)))
        mot2.data = int(5000 * (10.5/(11.00)))
        mot3.data = int(5000 * (10.5/(10.50)))
        mot4.data = int(5000 * (10.5/(10.75)))
        pub1.publish(mot1)
        pub2.publish(mot2)
        pub3.publish(mot3)
        pub4.publish(mot4)
        rospy.sleep(10)

        mot1.data = 0
        mot2.data = 0
        mot3.data = 0
        mot4.data = 0
        pub1.publish(mot1)
        pub2.publish(mot2)
        pub3.publish(mot3)
        pub4.publish(mot4)
        rospy.sleep(10)
        break
        # # rospy.spin()
        rate.sleep()
