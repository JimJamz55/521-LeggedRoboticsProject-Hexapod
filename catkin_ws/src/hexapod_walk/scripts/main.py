#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import time
import numpy as np

def hexapod_control():
    rospy.init_node('Hexapod_controller', anonymous=True)
    leg1_knee_joint_pos_con = rospy.Publisher('/hexapod/leg1_knee_joint_pos_con/command', Float64, queue_size=10)
    leg2_knee_joint_pos_con = rospy.Publisher('/hexapod/leg2_knee_joint_pos_con/command', Float64, queue_size=10)
    leg3_knee_joint_pos_con = rospy.Publisher('/hexapod/leg3_knee_joint_pos_con/command', Float64, queue_size=10)
    leg4_knee_joint_pos_con = rospy.Publisher('/hexapod/leg4_knee_joint_pos_con/command', Float64, queue_size=10)
    leg5_knee_joint_pos_con = rospy.Publisher('/hexapod/leg5_knee_joint_pos_con/command', Float64, queue_size=10)
    leg6_knee_joint_pos_con = rospy.Publisher('/hexapod/leg6_knee_joint_pos_con/command', Float64, queue_size=10)
    #rate = rospy.Rate(50) # 10hz
    #while not rospy.is_shutdown():
    time.sleep(1)
    for i in np.arange(-1, 0, 0.001):
    	leg1_knee_joint_pos_con.publish(i)
    	leg2_knee_joint_pos_con.publish(i)
    	leg3_knee_joint_pos_con.publish(i)
    	leg4_knee_joint_pos_con.publish(i)
    	leg5_knee_joint_pos_con.publish(i)
    	leg6_knee_joint_pos_con.publish(i)
    	
        #rate.sleep()

if __name__ == '__main__':
    try:
        hexapod_control()
    except rospy.ROSInterruptException:
        pass
