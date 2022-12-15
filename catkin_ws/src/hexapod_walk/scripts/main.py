#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import time
import numpy as np
from hex_IK import *

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
		#self.leg1_hip_joint_pos_con.publish(0)
		#self.leg2_hip_joint_pos_con.publish(0)
		#self.leg3_hip_joint_pos_con.publish(0)
		#self.leg4_hip_joint_pos_con.publish(0)
		#self.leg5_hip_joint_pos_con.publish(0)
		#self.leg6_hip_joint_pos_con.publish(0)
		
	def wraptopi(self, x):
		return (x + np.pi) % (2 * np.pi) - np.pi
    		
	def control_loop(self):
	    P = [0, 0, 80, 0, 0, 0]
	    #forward = True
	    
	    u_traj = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
	    		
	    #print(u_traj[0][1])	    		
	    #print(type(u_traj))
	    startTime = rospy.get_time()
	    #print(startTime + 10)
	    strideHeight = 40
	    strideLength = 10
	    speed = 4

	    while(rospy.get_time()< startTime + pi):
	    	#print(rospy.get_time())
	    	#print(startTime + 5)
	    	simTime = rospy.get_time() - startTime
	    	print(simTime)
	    	
	    	#u_traj[2][0] = 50
	    	
	    	#print(sin(simTime)*2.5)
	    	z_comp_traj = -sin(simTime*speed)*strideHeight
	    	if z_comp_traj<0:
	    		u_traj[2][0] = z_comp_traj
	    	#	print("u_z Component: " +str(sin(simTime)*strideHeight))
	    		u_traj[1][0] = u_traj[1][0] - 0.1
	    		print("u_y Component: " + str(u_traj[1][0]))
	    #	print(rospy.get_rostime() - seconds)
	    	#print(i)
	    	#if sin(simTime - pi/2) < 0:
	    	#	u_traj[2][0] = -sin(simTime*2)*strideLength
	    	#	print("u_y Component: " +str(sin(simTime*2)*strideLength))
	    	#u_traj[1][0] = -sin(simTime-pi/2)*40
	    	#print(u_traj[2][0])
	    	#u_traj[2][0] = u_traj[2][0] - sin()
	    #	if (forward == True):
	    #		P[0] = P[0] + 0.2
	    #	else:
	    #		P[0] = P[0] - 0.2
	    #	if (P[0] >30):
	    #		forward = False
	    #	if (P[0] < -30):
	    #		forward = True
	    #	print("Y pos: " + str(P[1]))
	    #P = [0, 0, 132.32, 0, 0, 0]
	    #P = [0, 50, 80, 0, 0, 0]
	    
	    	joints = hex_IK(P, u_traj)
	    
	    #print("alpha: " + str(joints[0][0]))
	    
	    	for i, angle in enumerate(joints[0][0]):
	    		joints[0][0][i] = self.wraptopi(joints[0][0][i])
	    
	    	joints[1][0] = -joints[1][0]
		
	    	#print("alpha: " + str(joints[0][0][0]))
	    #print("beta: " + str(joints[1][0]))
	    #print("gamma: " + str(joints[2][0]))
		
		
	    	self.leg1_hip_joint_pos_con.publish(joints[0][0][0])
	    	self.leg1_knee_joint_pos_con.publish(joints[1][0][0])
	    	self.leg1_ankle_joint_pos_con.publish(joints[2][0][0])
	    
	    	self.leg2_hip_joint_pos_con.publish(joints[0][0][1])
	    	self.leg2_knee_joint_pos_con.publish(joints[1][0][1])
	    	self.leg2_ankle_joint_pos_con.publish(joints[2][0][1])
	    
	    	self.leg3_hip_joint_pos_con.publish(joints[0][0][2])
	    	self.leg3_knee_joint_pos_con.publish(joints[1][0][2])
	    	self.leg3_ankle_joint_pos_con.publish(joints[2][0][2])
	    
	    	self.leg4_hip_joint_pos_con.publish(joints[0][0][3])
	    	self.leg4_knee_joint_pos_con.publish(joints[1][0][3])
	    	self.leg4_ankle_joint_pos_con.publish(joints[2][0][3])
	    
	    	self.leg5_hip_joint_pos_con.publish(joints[0][0][4])
	    	self.leg5_knee_joint_pos_con.publish(joints[1][0][4])
	    	self.leg5_ankle_joint_pos_con.publish(joints[2][0][4])
	    
	    	self.leg6_hip_joint_pos_con.publish(joints[0][0][5])
	    	self.leg6_knee_joint_pos_con.publish(joints[1][0][5])
	    	self.leg6_ankle_joint_pos_con.publish(joints[2][0][5])
		#for i in np.arange(-0.5, 0, 0.001):
		#	self.leg1_knee_joint_pos_con.publish(i)
		#	self.leg2_knee_joint_pos_con.publish(i)
		#	self.leg3_knee_joint_pos_con.publish(i)
		#	self.leg4_knee_joint_pos_con.publish(i)
		#	self.leg5_knee_joint_pos_con.publish(i)
		#	self.leg6_knee_joint_pos_con.publish(i)
    		#
		###	self.leg3_ankle_joint_pos_con.publish(i)
		#	self.leg4_ankle_joint_pos_con.publish(i)
		#	self.leg5_ankle_joint_pos_con.publish(i)
		#	self.leg6_ankle_joint_pos_con.publish(i)
			
		#P = np.array([5, 5, 132.32, 0, 0, 0])
		#hex_IK(P)
    		



if __name__ == '__main__':
    try:
        hexapod_con = hexapod_controller()
        hexapod_con.control_loop()
    except rospy.ROSInterruptException:
        pass
