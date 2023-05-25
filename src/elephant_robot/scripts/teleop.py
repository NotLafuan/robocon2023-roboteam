#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import sys
from select import select
import termios
import tty

moveBindings = {
    '': (0, 0, 0, 0),
    'w': (2500*6, 0, 0, 0),
    'a': (0, 2500*6, 0, 0),
    's': (-2500*6, 0, 0, 0),
    'd': (0, -2500*6, 0, 0),
    'q': (0, 0, -0.1, 0),
    'e': (0, 0, 0.1, 0),
    'p': (0,0,0,1),
    'o': (0,0,0,0),
}


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    rospy.init_node('teleop')
    rospy.loginfo('Node started')

    pub = rospy.Publisher('/elephant/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        key = getKey(termios.tcgetattr(sys.stdin), 0.1)
        if (key == '\x03'):
            break
        twist = Twist()
        if key in moveBindings:
            twist.linear.x, twist.linear.y, twist.angular.z, twist.linear.z = moveBindings[key]
        pub.publish(twist)
        rate.sleep()
