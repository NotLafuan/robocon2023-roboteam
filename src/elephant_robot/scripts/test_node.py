#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from itertools import cycle

poses = [[0, 0],
         [500, 0],
         [500, 500],
         [0, 500]]
pool = cycle(poses)

if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo('Node started')

    pub = rospy.Publisher('position', Pose, queue_size=10)

    rate = rospy.Rate(0.25)
    while not rospy.is_shutdown():
        pos = next(pool)
        pose = Pose()
        pose.position.x, pose.position.y = pos
        pub.publish(pose)
        rate.sleep()
