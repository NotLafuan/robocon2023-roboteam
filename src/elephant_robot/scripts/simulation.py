#!/usr/bin/env python3
import random
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16

import pygame
from sys import exit
from utils import *

WIDTH = 1280
HEIGHT = 720


def motor1_callback(data: Int16):
    global robot
    robot.w1 = data.data


def motor2_callback(data: Int16):
    global robot
    robot.w2 = data.data


def motor3_callback(data: Int16):
    global robot
    robot.w3 = data.data


def motor4_callback(data: Int16):
    global robot
    robot.w4 = data.data


if __name__ == '__main__':
    rospy.init_node('simulation')
    rospy.loginfo('Node started')

    ############### ROS ###############

    pub = rospy.Publisher('elephant_pose', Pose, queue_size=10)
    sub1 = rospy.Subscriber('motor1', Int16, motor1_callback)
    sub2 = rospy.Subscriber('motor2', Int16, motor2_callback)
    sub3 = rospy.Subscriber('motor3', Int16, motor3_callback)
    sub4 = rospy.Subscriber('motor4', Int16, motor4_callback)

    ############## PYGAME #############

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption('simulation')

    ############## ROBOT ##############

    robot = Robot(Time, pygame.Vector2(100, 100), pygame.Color(0, 0, 255))
    robot.transform = pygame.Vector2(WIDTH//2, HEIGHT//2)

    ############## LOOP ###############

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        Time.update()
        robot.update()

        screen.fill('white')
        robot.blit(screen)

        pygame.display.update()

        pos = Pose()
        pos.orientation.z = robot.angle + random.uniform(-1, 1)
        pos.position.x = robot.transform.x + random.uniform(-1, 1)
        pos.position.y = robot.transform.y + random.uniform(-1, 1)
        pub.publish(pos)

        rate.sleep()
