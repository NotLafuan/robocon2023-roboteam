#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pyPS4Controller.controller import Controller


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.x = 0
        self.y = 0

    def on_L3_up(self, value):
        self.y = value
        self.publish()

    def on_L3_down(self, value):
        self.y = value
        self.publish()

    def on_L3_right(self, value):
        self.x = value
        self.publish()

    def on_L3_left(self, value):
        self.x = value
        self.publish()

    def on_L3_x_at_rest(self):
        self.x = 0
        self.publish()

    def on_L3_y_at_rest(self):
        self.y = 0
        self.publish()

    def publish(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.linear.y = self.y
        pub.publish(twist)
        print('\r',
              f'{self.x:6.0f}',
              f'{self.y:6.0f}', end='', flush=True)


if __name__ == '__main__':
    rospy.init_node('ps4_teleop')
    rospy.loginfo('Node started')
    pub = rospy.Publisher('/elephant/cmd_vel', Twist, queue_size=10)

    controller = MyController(
        interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
