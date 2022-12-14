#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import math
import time
import numpy as np
from trajectory import *

def callback():
    #rospy.loginfo('here is what i receive: %s',data)
    print('data from joint state')
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
    odom_sub = rospy.Subscriber("/hexapod/leg1_hip_joint_pos_con/state", String, callback)

    time.sleep(0.4)
    leg1_knee_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg2_knee_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg4_knee_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg6_knee_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg5_knee_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg3_knee_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg1_ankle_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg2_ankle_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg4_ankle_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg6_ankle_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg5_ankle_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg3_ankle_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg1_hip_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg2_hip_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg4_hip_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg6_hip_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg5_hip_joint_pos_con.publish(0)
    time.sleep(0.4)
    leg3_hip_joint_pos_con.publish(0)

def homeposition(angle4):
    rospy.init_node('Hexapod_controller', anonymous=True)
    leg1_hip_joint_pos_con = rospy.Publisher('/hexapod/leg1_hip_joint_pos_con/command', Float64, queue_size=10)
    leg2_hip_joint_pos_con = rospy.Publisher('/hexapod/leg2_hip_joint_pos_con/command', Float64, queue_size=10)
    leg3_hip_joint_pos_con = rospy.Publisher('/hexapod/leg3_hip_joint_pos_con/command', Float64, queue_size=10)
    leg4_hip_joint_pos_con = rospy.Publisher('/hexapod/leg4_hip_joint_pos_con/command', Float64, queue_size=10)
    leg5_hip_joint_pos_con = rospy.Publisher('/hexapod/leg5_hip_joint_pos_con/command', Float64, queue_size=10)
    leg6_hip_joint_pos_con = rospy.Publisher('/hexapod/leg6_hip_joint_pos_con/command', Float64, queue_size=10)
    leg1_hip_joint_pos_con.publish(angle4[0][1])
    time.sleep(0.1)
    leg2_hip_joint_pos_con.publish(angle4[1][1])
    time.sleep(0.1)
    leg3_hip_joint_pos_con.publish(angle4[2][1])
    time.sleep(0.1)
    leg4_hip_joint_pos_con.publish(angle4[3][1])
    time.sleep(0.1)
    leg5_hip_joint_pos_con.publish(angle4[4][1])
    time.sleep(0.1)
    leg6_hip_joint_pos_con.publish(angle4[5][1])
    time.sleep(0.1)

def hexapod_controlinit(leg1,location1,leg2,location2):
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


    L1 = 0.052
    L2 = 0.06611
    L3 = 0.15869
    currentQ1 = location1
    currentQ2 = location2
    S = np.array([[0,1,1],[0,0,0],[1,0,0],[0,0,0],[0,0,0],[0,-L1,-L1-L2]])
    M = np.array([[0,1,0,0],[0,0,-1,L1+L2],[-1,0,0,-L3],[0,0,0,1]])
    #shows actual position matrix
    Pos1 = fkine(S,M,currentQ1,'space')
    Pos2 = fkine(S,M,currentQ2,'space')
    Tp =4 #period for the whole movement
    beta = 0.75
    L  = 0.04 #40 mm
    x0 = 0
    xf = -L
    tf = (1-beta)*Tp
    A = quinticpoly(0,tf,x0,xf)
    B = quinticpoly(0,tf,x0,-xf)
    time0 = time.time()
    time.sleep(0.1)
    #current position
    currentPose1 = np.array([Pos1[0,3],Pos1[1,3],Pos1[2,3]])
    currentPose2 = np.array([Pos2[0,3],Pos2[1,3],Pos2[2,3]])
    t = 0
    while (t<tf):
        timeactual = time.time()
        t = timeactual-time0
        if leg1==4:
            A = B #change the direction of the movement
        (xd1,zd1,xdp1,zdp1) = semicircle(A,t,0.5*L)
        (xd2,zd2,xdp2,zdp2) = semicircle(B,t,0.5*L)
        value = 0.9068
        if leg1==3 or leg1==4:
            value = 0
        targetPose1 = np.array([Pos1[0,3]+xd1,Pos1[1,3]+(xd1*math.cos(value)),Pos1[2,3]+zd1])
        targetPose2 = np.array([Pos2[0,3]+xd2,Pos2[1,3]+(xd2*math.cos(value)),Pos2[2,3]+zd2])
        error1=np.linalg.norm(targetPose1-currentPose1)
        error2=np.linalg.norm(targetPose2-currentPose2)
        start = time.time()
        currentQ1, currentPose1 = nextstep(targetPose1,currentPose1,currentQ1,S,M,error1)
        currentQ2, currentPose2 = nextstep(targetPose2,currentPose2,currentQ2,S,M,error2)
        #print(targetPose1,targetPose2,time.time()-time0)
        #print(np.rad2deg(currentQ),time.time()-time0)
        if leg1==1:
            leg1_hip_joint_pos_con.publish(currentQ1[0][0])
            leg1_knee_joint_pos_con.publish(currentQ1[0][1])
            leg1_ankle_joint_pos_con.publish(currentQ1[0][2])
            time.sleep(0.05)
        if leg1==2:
            leg2_hip_joint_pos_con.publish(currentQ1[0][0])
            leg2_knee_joint_pos_con.publish(currentQ1[0][1])
            leg2_ankle_joint_pos_con.publish(currentQ1[0][2])
            time.sleep(0.05)
        if leg1==3:
            leg3_hip_joint_pos_con.publish(currentQ1[0][0])
            leg3_knee_joint_pos_con.publish(currentQ1[0][1])
            leg3_ankle_joint_pos_con.publish(currentQ1[0][2])
            time.sleep(0.05)
        if leg1==4:
            leg4_hip_joint_pos_con.publish(currentQ1[0][0])
            leg4_knee_joint_pos_con.publish(currentQ1[0][1])
            leg4_ankle_joint_pos_con.publish(currentQ1[0][2])
            time.sleep(0.05)
        if leg2==5:
            leg5_hip_joint_pos_con.publish(currentQ2[0][0])
            leg5_knee_joint_pos_con.publish(currentQ2[0][1])
            leg5_ankle_joint_pos_con.publish(currentQ2[0][2])
            time.sleep(0.05)
        if leg2==6:
            leg6_hip_joint_pos_con.publish(currentQ2[0][0])
            leg6_knee_joint_pos_con.publish(currentQ2[0][1])
            leg6_ankle_joint_pos_con.publish(currentQ2[0][2])
            time.sleep(0.05)
    return currentQ1, currentQ2

if __name__ == '__main__':
    try:
        n=0 #number of iterations
        angle=np.array([[-0.03216931678612234, -0.04730419551678278, -0.09562572598838903, -0.059421669996820636, -0.03869111811914472, -0.04666039933393007, -0.12022704629133862, -0.006461711485485466, -0.04764317296659315, 0.0668439639303573, 0.012996042158297527, -0.12243667931465385, -0.05081259447977082, -0.061298791110647954, -0.08781632229306258, -0.3323084588304175, -0.1627504699968494, -0.25359482823108337]])
        angle4 = np.array([[angle[0][2],angle[0][0],angle[0][1]],
                [angle[0][5],angle[0][3],angle[0][4]],
                [angle[0][8],angle[0][6],angle[0][7]],
                [angle[0][11],angle[0][9],angle[0][10]],
                [angle[0][14],angle[0][12],angle[0][13]],
                [angle[0][17],angle[0][15],angle[0][16]]])
        #print('initial:',np.rad2deg(angle4))
        while(n<10):
            leg = np.array([4,4,2,5,3,3,1,6])#np.array([1,6,3,4,5,2])
            for i in range(0,len(leg),2):
                Q1, Q2 = hexapod_controlinit(leg[i],np.array([angle4[leg[i]-1]]),leg[i+1],np.array([angle4[leg[i+1]-1]]))
                print('LEG:',leg[i],leg[i+1])
                #print('LEG:',leg[i],leg[i+1],np.rad2deg(angle4[leg[i]-1][0]-Q1[0][0]),np.rad2deg(angle4[leg[i+1]-1][0]-Q1[0][0]))
                #print(np.rad2deg(Q1[0][0]),np.rad2deg(Q2[0][0]))
                #angle4[leg[i]-1][1]=Q1[0][1]
                #angle4[leg[i]-1][2]=Q1[0][2]
                #if leg[i] != leg[i+1]:
                #    angle4[leg[i+1]-1][1]=Q2[0][1]
                #    angle4[leg[i+1]-1][2]=Q2[0][2]
            n=n+1
            print(n)
            #angle4[0][0]=angle4[0][0]+0.087
            #angle4[1][0]=angle4[1][0]+0.087
            #angle4[2][0]=angle4[2][0]+0.087
            #angle4[3][0]=angle4[3][0]+0.087
            #angle4[4][0]=angle4[4][0]+0.087
            #angle4[5][0]=angle4[5][0]+0.087
            #homeposition(angle4)
        #print('final',angle4)

    except rospy.ROSInterruptException:
        pass

