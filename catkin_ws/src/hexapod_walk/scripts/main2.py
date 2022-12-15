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

def hexapod_controlinit(leg,location):
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
    correction = np.array([[0.9068,0,0],
                    [-0.9068,0,0],
                    [0,0,0],
                    [0,0,0],
                    [2.2348,0,0],
                    [0,0,0]])
    currentQ= location
    S = np.array([[0,1,1],[0,0,0],[1,0,0],[0,0,0],[0,0,0],[0,-L1,-L1-L2]])
    M = np.array([[0,1,0,0],[0,0,-1,L1+L2],[-1,0,0,-L3],[0,0,0,1]])
    #shows actual position matrix
    Pos = fkine(S,M,currentQ,'space')
    Tp =4
    beta = 0.75
    L  = 0.04
    x0 = 0
    xf = -L
    tf = (1-beta)*Tp
    A = quinticpoly(0,tf,x0,xf)
    B = quinticpoly(0,tf,x0,-xf)
    time0 = time.time()
    time.sleep(0.1)
    #current position
    currentPose = np.array([Pos[0,3],Pos[1,3],Pos[2,3]])
    t = 0
    while (t<tf):
        timeactual = time.time()
        t = timeactual-time0
        if leg==2 or leg==4 or leg==6:
            A = B #change the direction of the movement
        (xd,zd,xdp,zdp) = semicircle(A,t,0.5*L)
        value = 0.9068
        if leg==3 or leg==4:
            value = 0
        targetPose = np.array([Pos[0,3]+xd,Pos[1,3]+(xd*math.cos(value)),Pos[2,3]+zd])
        error=np.linalg.norm(targetPose-currentPose)
        start = time.time()
        currentQ, currentPose = nextstep(targetPose,currentPose,currentQ,S,M,error)
        print(targetPose,time.time()-time0)
        #print(np.rad2deg(currentQ),time.time()-time0)
        if leg==1:
            leg1_hip_joint_pos_con.publish(currentQ[0][0])
            leg1_knee_joint_pos_con.publish(currentQ[0][1])
            leg1_ankle_joint_pos_con.publish(currentQ[0][2])
            time.sleep(0.1)
        if leg==2:
            leg2_hip_joint_pos_con.publish(currentQ[0][0])
            leg2_knee_joint_pos_con.publish(currentQ[0][1])
            leg2_ankle_joint_pos_con.publish(currentQ[0][2])
            time.sleep(0.1)
        if leg==3:
            leg3_hip_joint_pos_con.publish(currentQ[0][0])
            leg3_knee_joint_pos_con.publish(currentQ[0][1])
            leg3_ankle_joint_pos_con.publish(currentQ[0][2])
            time.sleep(0.1)
        if leg==4:
            leg4_hip_joint_pos_con.publish(currentQ[0][0])
            leg4_knee_joint_pos_con.publish(currentQ[0][1])
            leg4_ankle_joint_pos_con.publish(currentQ[0][2])
            time.sleep(0.1)
        if leg==5:
            leg5_hip_joint_pos_con.publish(currentQ[0][0])
            leg5_knee_joint_pos_con.publish(currentQ[0][1])
            leg5_ankle_joint_pos_con.publish(currentQ[0][2])
            time.sleep(0.1)
        if leg==6:
            leg6_hip_joint_pos_con.publish(currentQ[0][0])
            leg6_knee_joint_pos_con.publish(currentQ[0][1])
            leg6_ankle_joint_pos_con.publish(currentQ[0][2])
            time.sleep(0.1)
    print(currentQ)

if __name__ == '__main__':
    try:
        n=0
        while(n<10):
            theta=1
            if theta==1:
                angle=np.array([[0.0018884109732280052, 0.00015063984619967385, -0.025365803535576603, 0.02484363998338246, 0.2491338623607504, -0.07267071785213997, -0.0343592577750389, 0.0020000228688239474, -0.05948176707751007, -0.06483710318484448, 0.013091245829608411, -0.10401692564627751, -0.07414571097637657, -0.014822103839051692, -0.11002534799403119, -0.2937605140740196, 0.0013544077730376358, -0.21108933007627506]])
                leg = np.array([4,2,5,3,1,6])#np.array([1,6,3,4,5,2])
                angle4 = np.array([[angle[0][2],angle[0][0],angle[0][1]],
                    [angle[0][5],angle[0][3],angle[0][4]],
                    [angle[0][8],angle[0][6],angle[0][7]],
                    [angle[0][11],angle[0][9],angle[0][10]],
                    [angle[0][14],angle[0][12],angle[0][13]],
                    [angle[0][17],angle[0][15],angle[0][16]]])
                correction = np.array([[0.9068,0,0],
                    [-0.9068,0,0],
                    [0,0,0],
                    [0,0,0],
                    [2.2348,0,0],
                    [0,0,0]])
                for i in range(0,len(leg),2):
                    #print('LEG:',leg[i],leg[i+1])
                    hexapod_controlinit(leg[i],np.array([angle4[leg[i]-1]]))
            n=n+1
            #print(np.rad2deg(angle4))
            homeposition(angle4)

    except rospy.ROSInterruptException:
        pass

