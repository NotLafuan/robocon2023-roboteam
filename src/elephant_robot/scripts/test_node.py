#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def chatter_callback(msg: String):
    rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo('Node started')

    sub = rospy.Subscriber('chatter', String, callback=chatter_callback)
    rospy.spin()