#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32
from pyPS4Controller.controller import Controller


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.x = 0
        self.y = 0
        self.z = 0

    def on_L3_up(self, value):
        self.y = -value
        self.publish()

    def on_L3_down(self, value):
        self.y = -value
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

    def on_R3_left(self, value):
        self.z = (value/32768)*0.01
        self.publish()

    def on_R3_right(self, value):
        self.z = (value/32768)*0.01
        self.publish()

    def on_R3_x_at_rest(self):
        self.z = 0
        self.publish()

    def on_R3_down(self, _):
        self.publish()

    def on_R3_up(self, _):
        self.publish()

    def on_R3_y_at_rest(self):
        self.publish()

    def on_L1_press(self):
        pub2.publish(Empty())

    def publish(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.linear.y = self.y
        twist.angular.z = self.z
        pub1.publish(twist)
        print('\r',
              f'{self.x:6.0f}',
              f'{self.y:6.0f}', end='', flush=True)


if __name__ == '__main__':
    rospy.init_node('ps4_teleop')
    rospy.loginfo('Node started')
    pub1 = rospy.Publisher('/elephant/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('feeding', Empty, queue_size=10)
    pub3 = rospy.Publisher('home_lifter', Empty, queue_size=10)
    pub4 = rospy.Publisher('move_lifter', Float32, queue_size=10)

    controller = MyController(
        interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
