#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16


if __name__ == '__main__':
    rospy.init_node('test_motor')
    rospy.loginfo('Node started')

    pub1 = rospy.Publisher('motor1', Int16, queue_size=10)
    pub2 = rospy.Publisher('motor2', Int16, queue_size=10)
    pub3 = rospy.Publisher('motor3', Int16, queue_size=10)
    pub4 = rospy.Publisher('motor4', Int16, queue_size=10)

    val = Int16()

    val.data = 0
    pub1.publish(val)
    pub2.publish(val)
    pub3.publish(val)
    pub4.publish(val)

    val.data = 60000//10

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pub1.publish(val)
        pub2.publish(val)
        pub3.publish(val)
        pub4.publish(val)
        rate.sleep()
    val.data = 0
    pub1.publish(val)
    pub2.publish(val)
    pub3.publish(val)
    pub4.publish(val)