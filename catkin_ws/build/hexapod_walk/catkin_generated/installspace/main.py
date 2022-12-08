#!/usr/bin/env python3
import rospy
from mav_msgs.msg import Actuators
from std_msgs.msg import Empty, Float64

class Hexapod():
	def __init__(self):
		self.servo_pos_pub = rospy.Publisher("/hexapod/leg1_knee_joint_pos_con/command", Float64, queue_size=10)
		self.msg = Float64()
		self.msg.data = 1.0
		self.servo_pos_pub.publish(self.msg)
		
if __name__ == '__main__':
	rospy.init_node("hexapod_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	instance = Hexapod()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
		

