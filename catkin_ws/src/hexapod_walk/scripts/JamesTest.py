#!/usr/bin/env python

from hex_IK import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import math
import time
import numpy as np
from trajectory import *

P = [0, 0, 132.32, 0, 0, 0]
#P = [0, 0, 132.32, 0, 0, -5]
joints = hex_IK(P)
#print(joints)
print("alpha: " + str(joints[0][0]))
print("beta: " + str(joints[1][0]))
print("gamma: " + str(joints[2][0]))

def hexapod_control():
    rospy.init_node('Hexapod_controller', anonymous=True)
    leg1_knee_joint_pos_con = rospy.Publisher('/hexapod/leg1_knee_joint_pos_con/command', Float64, queue_size=10)
    leg2_knee_joint_pos_con = rospy.Publisher('/hexapod/leg2_knee_joint_pos_con/command', Float64, queue_size=10)
    leg3_knee_joint_pos_con = rospy.Publisher('/hexapod/leg3_knee_joint_pos_con/command', Float64, queue_size=10)
    leg4_knee_joint_pos_con = rospy.Publisher('/hexapod/leg4_knee_joint_pos_con/command', Float64, queue_size=10)
    leg5_knee_joint_pos_con = rospy.Publisher('/hexapod/leg5_knee_joint_pos_con/command', Float64, queue_size=10)
    leg6_knee_joint_pos_con = rospy.Publisher('/hexapod/leg6_knee_joint_pos_con/command', Float64, queue_size=10)
    leg1_hip_joint_pos_con = rospy.Publisher('/hexapod/leg1_hip_joint_pos_con/command', Float64, queue_size=10)
    leg2_hip_joint_pos_con = rospy.Publisher('/hexapod/leg2_hip_joint_pos_con/command', Float64, queue_size=10)
    leg3_hip_joint_pos_con = rospy.Publisher('/hexapod/leg3_hip_joint_pos_con/command', Float64, queue_size=10)
    leg4_hip_joint_pos_con = rospy.Publisher('/hexapod/leg4_hip_joint_pos_con/command', Float64, queue_size=10)
    leg5_hip_joint_pos_con = rospy.Publisher('/hexapod/leg5_hip_joint_pos_con/command', Float64, queue_size=10)
    leg6_hip_joint_pos_con = rospy.Publisher('/hexapod/leg6_hip_joint_pos_con/command', Float64, queue_size=10)
    leg1_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg1_ankle_joint_pos_con/command', Float64, queue_size=10)
    leg2_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg2_ankle_joint_pos_con/command', Float64, queue_size=10)
    leg3_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg3_ankle_joint_pos_con/command', Float64, queue_size=10)
    leg4_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg4_ankle_joint_pos_con/command', Float64, queue_size=10)
    leg5_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg5_ankle_joint_pos_con/command', Float64, queue_size=10)
    leg6_ankle_joint_pos_con = rospy.Publisher('/hexapod/leg6_ankle_joint_pos_con/command', Float64, queue_size=10)
    #odom_sub = rospy.Subscriber("/hexapod/leg1_hip_joint_pos_con/state", String, callback)
    
    #leg1_hip_joint_pos_con.publish(joints[0][0][0])
    leg1_hip_joint_pos_con.publish(-1)
    leg1_knee_joint_pos_con.publish(joints[1][0][0])
    leg1_ankle_joint_pos_con.publish(joints[2][0][0])
    
    leg2_hip_joint_pos_con.publish(joints[0][0][1])
    leg2_knee_joint_pos_con.publish(joints[1][0][1])
    leg2_ankle_joint_pos_con.publish(joints[2][0][1])
    
    leg3_hip_joint_pos_con.publish(joints[0][0][2])
    leg3_knee_joint_pos_con.publish(joints[1][0][2])
    leg3_ankle_joint_pos_con.publish(joints[2][0][2])
    
    leg4_hip_joint_pos_con.publish(joints[0][0][3])
    leg4_knee_joint_pos_con.publish(joints[1][0][3])
    leg4_ankle_joint_pos_con.publish(joints[2][0][3])
    
    leg5_hip_joint_pos_con.publish(joints[0][0][4])
    leg5_knee_joint_pos_con.publish(joints[1][0][4])
    leg5_ankle_joint_pos_con.publish(joints[2][0][4])
    
    leg6_hip_joint_pos_con.publish(joints[0][0][5])
    leg6_knee_joint_pos_con.publish(joints[1][0][5])
    leg6_ankle_joint_pos_con.publish(joints[2][0][5])
hexapod_control()

