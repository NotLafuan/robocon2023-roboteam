#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32
from pyPS4Controller.controller import Controller
import time
from numpy import pi


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.x = 0
        self.y = 0
        self.z = 0
        self.rpm = 0

    ########## MOVEMENT TRANSLATIONAL ##########

    def on_L3_up(self, value):
        self.y = -value
        self.publish_movement()

    def on_L3_down(self, value):
        self.y = -value
        self.publish_movement()

    def on_L3_right(self, value):
        self.x = value
        self.publish_movement()

    def on_L3_left(self, value):
        self.x = value
        self.publish_movement()

    def on_L3_x_at_rest(self):
        self.x = 0
        self.publish_movement()

    def on_L3_y_at_rest(self):
        self.y = 0
        self.publish_movement()

    ########### MOVEMENT ROTATIONAL ############

    # def on_R3_left(self, value):
    #     self.z = (value/32768)*0.01
    #     self.publish_movement()

    # def on_R3_right(self, value):
    #     self.z = (value/32768)*0.01
    #     self.publish_movement()

    # def on_R3_x_at_rest(self):
    #     self.z = 0
    #     self.publish_movement()

    # def on_R3_down(self, _):
    #     self.publish_movement()

    # def on_R3_up(self, _):
    #     self.publish_movement()

    # def on_R3_y_at_rest(self):
    #     self.publish_movement()

    ############### ANGLE CHANGE ###############

    def on_left_arrow_press(self):
        self.z -= pi/2
        if self.z >= pi/2:
            self.z = pi/2
        elif self.z <= -pi/2:
            self.z = -pi/2
        self.publish_movement()

    def on_right_arrow_press(self):
        self.z += pi/2
        if self.z >= pi/2:
            self.z = pi/2
        elif self.z <= -pi/2:
            self.z = -pi/2
        self.publish_movement()

    ################## SHOOT ###################

    # feeding
    def on_R1_press(self):
        pub2.publish(Empty())

    def on_L1_press(self):
        pub5.publish(Empty())

    ############# SPEED ADJUSMENTS #############

    def on_x_press(self):
        self.rpm = 8000
        pub6.publish(Float32(self.rpm))

    def on_square_press(self):
        self.rpm = 11600
        pub6.publish(Float32(self.rpm))

    def on_circle_press(self):
        self.rpm -= 100
        if self.rpm < 0:
            self.rpm = 0
        print('#'*40, 'RPM:', self.rpm)
        pub6.publish(Float32(self.rpm))

    def on_triangle_press(self):
        self.rpm += 100
        print('#'*40, 'RPM:', self.rpm)
        pub6.publish(Float32(self.rpm))

    ############## FEEDER CONTROL ##############

    def on_up_arrow_press(self):
        pub4.publish(Float32(180))

    def on_down_arrow_press(self):
        pub3.publish(Empty())

    def publish_movement(self):
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
    pub5 = rospy.Publisher('launch', Empty, queue_size=10)
    pub6 = rospy.Publisher('vesc_rpm', Float32, queue_size=10)

    controller = MyController(
        interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
