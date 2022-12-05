#!/usr/bin/env python3
import rospy
from mav_msgs.msg import Actuators

class Hexapod():
	def __init__(self):
		motor_speed_pub = rospy.Publisher("")
		
if __name__ == '__main__':
	rospy.init_node("hexapod_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	instance = Hexapod()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")	
