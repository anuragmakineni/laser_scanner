#!/usr/bin/env python


import message_filters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import math
import rospy
import tf
import numpy as np


class table_driver(object):
    def __init__(self):
        rospy.init_node('table_driver')
        self.angle = 0
        self.increment = 0.05
        self.reverse_state = 1

        self.broadcaster = tf.TransformBroadcaster()

        self.laser_side_sub = rospy.Subscriber("laser/scan1", LaserScan, self.laser_1_cb)
        self.laser_top_sub = rospy.Subscriber("laser/scan2", LaserScan, self.laser_2_cb)
        self.motor_sub = rospy.Subscriber("/motor_controller/state", JointState, self.motor_cb)

        self.motor_pub = rospy.Publisher("/motor_controller/command", Float64, queue_size=10)

        self.laser_1_called = False
        self.laser_2_called = False

    def laser_1_cb(self, data):
        """
        save an angle 0 to 2pi, then back to 0. random small number increment (.05)
        publish motor_pub.publish(angle)
        """

        # rospy.loginfo("laser1!")

        self.laser_1_called = True

        if self.laser_1_called and self.laser_2_called:
            self.increment = 0.017
            if self.angle < 0:
                self.reverse_state = 1
            elif self.angle > 2 * math.pi:
                self.reverse_state = -1

            self.angle += self.reverse_state * self.increment

            self.laser_1_called = False
            self.laser_2_called = False

            self.motor_pub.publish(self.angle)

    def laser_2_cb(self, data):
        """
        save an angle 0 to 2pi, then back to 0. random small number increment (.05)
        publish motor_pub.publish(angle)
        """

        # rospy.loginfo("laser2!")

        self.laser_2_called = True

        if self.laser_1_called and self.laser_2_called:
            self.increment = 0.017
            if self.angle < 0:
                self.reverse_state = 1
            elif self.angle > 2 * math.pi:
                self.reverse_state = -1

            self.angle += self.reverse_state * self.increment

            self.laser_1_called = False
            self.laser_2_called = False

            self.motor_pub.publish(self.angle)

    def motor_cb(self, data):
        """
        do the tf thing, get the angle from motor state (position field), 0 to 4095. convert that to radians
        then put it into the broadcaster.sendTransfoorm.... anurag wrote
            4095 = x 2pi
        """
        position = data.current_pos

        radius = 0.5  # CHANGE
        x = np.cos(position) * radius
        y = np.sin(position) * radius

        self.broadcaster.sendTransform((x, y, 0),
                                       tf.transformations.quaternion_from_euler(-math.pi / 2, 0, position + math.pi),
                                       rospy.Time.now(), "laser1", "world")

        self.broadcaster.sendTransform((0, 0, radius),
                                       tf.transformations.quaternion_from_euler(0, math.pi / 2, position),
                                       rospy.Time.now(), "laser2", "world")


if __name__ == '__main__':
    node = table_driver()
    try:
        rospy.spin()
        node.stop()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
