#!/usr/bin/env python


import message_filters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import math
import rospy
import tf
import numpy as np




def table_driver():
	angle = 0
	increment = 0.05

	reverse_state = False

	rospy.init_node('table_driver',anonymous=True)
	rate = rospy.Rate(100)
	init_time = rospy.get_rostime().to_sec()
	
	broadcaster = tf.TransformBroadcaster()

	laser_side_sub = message_filters.Subscriber("laser/scan1", LaserScan)
	laser_top_sub = message_filters.Subscriber("laser/scan2", LaserScan)
	motor_sub = rospy.Subscriber( "/motor_controller/state", JointState, motor_cb )
	
	motor_pub = rospy.Publisher("/motor_controller/command", Float64, queue_size=10 )

	
	ts = message_filters.TimeSynchronizer([laser_side_sub, laser_top_sub], 10)
	ts.registerCallback(laser_cb)

	print("hello")
	rospy.spin()


def laser_cb(data):
	"""
	save an angle 0 to 2pi, then back to 0. random small number increment (.05)
	publish motor_pub.publish(angle) 	
	"""
	if reverse_state == True:
		increment = -0.05
	if reverse_state == False:
		increment = 0.05
	if angle < 0 or angle >2*math.pi:
		reverse_state = -reverse_state
		
	angle+=increment
		
	motor_pub.publish(angle)
	pass

def motor_cb(data):
	"""
	do the tf thing, get the angle from motor state (position field), 0 to 4095. convert that to radians
	then put it into the broadcaster.sendTransfoorm.... anurag wrote
		4095 = x 2pi
	"""
	position = data.current_pos
	
	radius = 0.5 #CHANGE
        x = np.cos(position) * radius
        y = np.sin(position) * radius

	broadcaster.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(-math.pi/2, 0,  position + math.pi), rospy.Time.now(),"laser1", "world")
	
        broadcaster.sendTransform((0, 0, radius), tf.transformations.quaternion_from_euler(0, math.pi/2, position), rospy.Time.now(),   "laser2", "world")

	print("motorcb")
	pass
	


	

if __name__ == '__main__':
    try:
        table_driver()
    except rospy.ROSInterruptException:
        pass
