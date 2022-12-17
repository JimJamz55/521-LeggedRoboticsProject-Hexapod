#!/usr/bin/env python
    # license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import time
import numpy as np
from hex_IK import *
from trajectory import *
    
class hexapod_controller():
    def __init__(self):
        rospy.init_node('Hexapod_controller', anonymous=True)
        self.leg1_hip_joint_pos_con = rospy.Publisher('/hexapod/leg1_hip_joint_pos_con/command', Float64, queue_size=10)
        self.leg1_knee_joint_pos_con = rospy.Publisher('/hexapod/leg1_knee_joint_pos_con/command', Float64, queue_size=10)
        self.leg1_ankle_joint_pos_con = rospy.Publisher('/hexapod/Leg1_ankle_joint_pos_con/command', Float64, queue_size=10)
        
        self.leg2_hip_joint_pos_con = rospy.Publisher('/hexapod/leg2_hip_joint_pos_con/command', Float64, queue_size=10)
        self.leg2_knee_joint_pos_con = rospy.Publisher('/hexapod/leg2_knee_joint_pos_con/command', Float64, queue_size=10)
        self.leg2_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg2_ankle_joint_pos_con/command', Float64, queue_size=10)
        
        self.leg3_hip_joint_pos_con = rospy.Publisher('/hexapod/leg3_hip_joint_pos_con/command', Float64, queue_size=10)
        self.leg3_knee_joint_pos_con = rospy.Publisher('/hexapod/leg3_knee_joint_pos_con/command', Float64, queue_size=10)
        self.leg3_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg3_ankle_joint_pos_con/command', Float64, queue_size=10)
        
        self.leg4_hip_joint_pos_con = rospy.Publisher('/hexapod/leg4_hip_joint_pos_con/command', Float64, queue_size=10)
        self.leg4_knee_joint_pos_con = rospy.Publisher('/hexapod/leg4_knee_joint_pos_con/command', Float64, queue_size=10)
        self.leg4_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg4_ankle_joint_pos_con/command', Float64, queue_size=10)
        
        self.leg5_hip_joint_pos_con = rospy.Publisher('/hexapod/leg5_hip_joint_pos_con/command', Float64, queue_size=10)
        self.leg5_knee_joint_pos_con = rospy.Publisher('/hexapod/leg5_knee_joint_pos_con/command', Float64, queue_size=10)
        self.leg5_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg5_ankle_joint_pos_con/command', Float64, queue_size=10)
        
        self.leg6_hip_joint_pos_con = rospy.Publisher('/hexapod/leg6_hip_joint_pos_con/command', Float64, queue_size=10)
        self.leg6_knee_joint_pos_con = rospy.Publisher('/hexapod/leg6_knee_joint_pos_con/command', Float64, queue_size=10)
        self.leg6_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg6_ankle_joint_pos_con/command', Float64, queue_size=10)
        
        rate = rospy.Rate(100) # 10hz

        time.sleep(1)
        
        #initialize leg positions
        self.leg1_hip_joint_pos_con.publish(0) #0.66399
        self.leg2_hip_joint_pos_con.publish(0) #-0.66399
        self.leg3_hip_joint_pos_con.publish(0)
        self.leg4_hip_joint_pos_con.publish(0)
        self.leg5_hip_joint_pos_con.publish(0) #-0.66399
        self.leg6_hip_joint_pos_con.publish(0) #0.66399

    def control_loop(self):
        for i in np.arange(-0.5, 0.175, 0.001):
            self.leg1_knee_joint_pos_con.publish(i)
            self.leg2_knee_joint_pos_con.publish(i)
            self.leg3_knee_joint_pos_con.publish(i)
            self.leg4_knee_joint_pos_con.publish(i)
            self.leg5_knee_joint_pos_con.publish(i)
            self.leg6_knee_joint_pos_con.publish(i)
        for i in np.arange(-0.5, 0.175, 0.001):
            self.leg1_ankle_joint_pos_con.publish(i)
            self.leg2_ankle_joint_pos_con.publish(i)
            self.leg3_ankle_joint_pos_con.publish(i)
            self.leg4_ankle_joint_pos_con.publish(i)
            self.leg5_ankle_joint_pos_con.publish(i)
            self.leg6_ankle_joint_pos_con.publish(i)
        
        


if __name__ == '__main__':
    try:
        hexapod_con = hexapod_controller()
        hexapod_con.control_loop()
    except rospy.ROSInterruptException:
        pass