#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32


angleOffset = 0
angle = 0


def map_value(x: float,  in_min: float,  in_max: float,  out_min: float,  out_max: float):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def chatter_callback(msg: Float32):
    global angleOffset
    global angle
    if not angleOffset:
        angleOffset = -msg.data
    angle = msg.data + angleOffset
    actual_angle = angle if angle > 0 else angle+360
    rospy.loginfo(f'{angle}, {actual_angle}')


if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo('Node started')

    sub = rospy.Subscriber('/angle', Float32, callback=chatter_callback)
    rospy.spin()
    rospy.on_shutdown(lambda: print())
