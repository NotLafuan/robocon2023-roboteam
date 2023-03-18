#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from math import sin, cos, pi


def cmd_vel_callback(twist: Twist):
    rospy.loginfo(f'{twist.linear.x}, {twist.linear.y}, {twist.angular.z}')
    r = 1
    R = 1
    theta = 0
    w1 = (1 / r) * (-sin(theta + (1 * pi / 4)) * twist.linear.x + cos(theta + (1 * pi / 4)) * twist.linear.y + R * twist.angular.z)
    w2 = (1 / r) * (-sin(theta + (3 * pi / 4)) * twist.linear.x + cos(theta + (3 * pi / 4)) * twist.linear.y + R * twist.angular.z)
    w3 = (1 / r) * (-sin(theta + (5 * pi / 4)) * twist.linear.x + cos(theta + (5 * pi / 4)) * twist.linear.y + R * twist.angular.z)
    w4 = (1 / r) * (-sin(theta + (7 * pi / 4)) * twist.linear.x + cos(theta + (7 * pi / 4)) * twist.linear.y + R * twist.angular.z)
    mot1 = Int16()
    mot2 = Int16()
    mot3 = Int16()
    mot4 = Int16()
    mot1.data = int(w1)
    mot2.data = int(w2)
    mot3.data = int(w3)
    mot4.data = int(w4)
    pub1.publish(mot1)
    pub2.publish(mot2)
    pub3.publish(mot3)
    pub4.publish(mot4)


if __name__ == '__main__':
    rospy.init_node('kinematics')
    rospy.loginfo('Node started')

    sub = rospy.Subscriber(
        '/elephant/cmd_vel', Twist, callback=cmd_vel_callback)
    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)
    rospy.spin()
