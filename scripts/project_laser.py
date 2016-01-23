#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
import numpy as np
import message_filters


def project_laser():
    rospy.init_node('project_laser', anonymous=True)
    laser_sub_1 = message_filters.Subscriber('laser1', LaserScan)
    laser_sub_2 = message_filters.Subscriber('laser2', LaserScan)
    tf_sub = message_filters.Subscriber('tf', tf)

    ts1 = message_filters.TimeSynchronizer([laser_sub_1, tf_sub], 10)
    ts1.registerCallback(laser_1_cb)

    ts2 = message_filters.TimeSynchronizer([laser_sub_2, tf_sub], 10)
    ts2.registerCallback(laser_2_cb)

    rospy.spin()

def laser_1_cb():

def laser_2_cb():

if __name__ == '__main__':
    try:
        project_laser()
    except rospy.ROSInterruptException:
        pass
