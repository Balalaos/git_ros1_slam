#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs

##---

def robominer_joint_positions_publisher():

	rospy.init_node('robominer_control_screws')

	#Define publishers for each joint position controller commands.
	pub_FR = rospy.Publisher('/robominer/FR_screw_velocity_controller/command', Float64, queue_size=1)
	pub_FL = rospy.Publisher('/robominer/FL_screw_velocity_controller/command', Float64, queue_size=1)

	pub_BR = rospy.Publisher('/robominer/BR_screw_velocity_controller/command', Float64, queue_size=1)
	pub_BL = rospy.Publisher('/robominer/BL_screw_velocity_controller/command', Float64, queue_size=1)
	
	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position
	while not rospy.is_shutdown():

		screw_velocity = 10
		pub_FL.publish(1*screw_velocity)
		pub_FR.publish(1*screw_velocity)
		

		pub_BR.publish(1*screw_velocity)
		pub_BL.publish(1*screw_velocity)

		rate.sleep() #sleep for rest of rospy.Rate(100)


if __name__ == '__main__':
	try: robominer_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
