#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation


def imu_callback(imu: Imu):
    quat = [imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,]
    rot = Rotation.from_quat(quat)
    rot_euler = rot.as_euler('xyz')
    print(f'\r{rot_euler}', ' '*10, end='', flush=True)


if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo('Node started')

    sub = rospy.Subscriber('/imu/data', Imu, callback=imu_callback)
    rospy.spin()
