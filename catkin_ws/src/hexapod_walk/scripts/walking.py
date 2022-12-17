#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import math
import time
import numpy as np
from trajectory import *

def hexapod_control(A):
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
    print('Setting initial position')
    time.sleep(0.4)
    leg1_knee_joint_pos_con.publish(A[0][1])
    time.sleep(0.4)
    leg2_knee_joint_pos_con.publish(A[1][1])
    time.sleep(0.4)
    leg4_knee_joint_pos_con.publish(A[3][1])
    time.sleep(0.4)
    leg6_knee_joint_pos_con.publish(A[5][1])
    time.sleep(0.4)
    leg5_knee_joint_pos_con.publish(A[4][1])
    time.sleep(0.4)
    leg3_knee_joint_pos_con.publish(A[2][1])
    time.sleep(0.4)
    leg1_ankle_joint_pos_con.publish(A[0][2])
    time.sleep(0.4)
    leg2_ankle_joint_pos_con.publish(A[1][2])
    time.sleep(0.4)
    leg4_ankle_joint_pos_con.publish(A[3][2])
    time.sleep(0.4)
    leg6_ankle_joint_pos_con.publish(A[5][2])
    time.sleep(0.4)
    leg5_ankle_joint_pos_con.publish(A[4][2])
    time.sleep(0.4)
    leg3_ankle_joint_pos_con.publish(A[2][2])
    time.sleep(0.4)
    leg1_hip_joint_pos_con.publish(A[0][0])
    time.sleep(0.4)
    leg2_hip_joint_pos_con.publish(A[1][0])
    time.sleep(0.4)
    leg4_hip_joint_pos_con.publish(A[3][0])
    time.sleep(0.4)
    leg6_hip_joint_pos_con.publish(A[5][0])
    time.sleep(0.4)
    leg5_hip_joint_pos_con.publish(A[4][0])
    time.sleep(0.4)
    leg3_hip_joint_pos_con.publish(A[2][0])

def homeposition(leg,dato):
    rospy.init_node('Hexapod_controller', anonymous=True)
    leg1_hip_joint_pos_con = rospy.Publisher('/hexapod/leg1_hip_joint_pos_con/command', Float64, queue_size=10)
    leg2_hip_joint_pos_con = rospy.Publisher('/hexapod/leg2_hip_joint_pos_con/command', Float64, queue_size=10)
    leg3_hip_joint_pos_con = rospy.Publisher('/hexapod/leg3_hip_joint_pos_con/command', Float64, queue_size=10)
    leg4_hip_joint_pos_con = rospy.Publisher('/hexapod/leg4_hip_joint_pos_con/command', Float64, queue_size=10)
    leg5_hip_joint_pos_con = rospy.Publisher('/hexapod/leg5_hip_joint_pos_con/command', Float64, queue_size=10)
    leg6_hip_joint_pos_con = rospy.Publisher('/hexapod/leg6_hip_joint_pos_con/command', Float64, queue_size=10)
    if leg == 1:
        leg1_hip_joint_pos_con.publish(dato)
        leg6_hip_joint_pos_con.publish(-dato)
    if leg == 2:
        leg2_hip_joint_pos_con.publish(-dato)
        leg5_hip_joint_pos_con.publish(dato)
    if leg == 3:
        leg3_hip_joint_pos_con.publish(dato)
    if leg == 4:
        leg4_hip_joint_pos_con.publish(-dato)
    time.sleep(0.05)

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
    L  = 0.02 #40 mm0
    x0 = 0
    xf = -L
    tf = (1-beta)*Tp
    A = quinticpoly(0,tf,x0,xf)
    B = quinticpoly(0,tf,x0,-xf)
    time0 = time.time()
    currentPose1 = np.array([Pos1[0,3],Pos1[1,3],Pos1[2,3]])
    currentPose2 = np.array([Pos2[0,3],Pos2[1,3],Pos2[2,3]])
    matrix = np.array([0.9068,-0.9068,1.5708,-1.5708,2.2340,-2.2340])
    theta1 = matrix[leg1-1]
    Rz1 = np.array([[math.cos(theta1),-math.sin(theta1)],[math.sin(theta1), math.cos(theta1)]])
    theta2 = matrix[leg2-1]
    Rz2 = np.array([[math.cos(theta2),-math.sin(theta2)],[math.sin(theta2), math.cos(theta2)]])
    Posbody1 = np.dot(Rz1,np.array([Pos1[0,3],Pos1[1,3]]))
    Posbody2 = np.dot(Rz2,np.array([Pos2[0,3],Pos2[1,3]]))
    t = 0
    while (t<tf):
        timeactual = time.time()
        t = timeactual-time0
        (xd1,zd1,xdp1,zdp1) = semicircle(A,t,0.5*L) #patas 2,4,6
        (xd2,zd2,xdp2,zdp2) = semicircle(A,t,0.5*L) #patas 1,3,5
        x1 = Posbody1[0]
        y1 = Posbody1[1]+xd1
        data1 = np.dot(np.linalg.inv(Rz1),np.array([x1,y1]))
        x2 = Posbody2[0]
        y2 = Posbody2[1]+xd2
        data2 = np.dot(np.linalg.inv(Rz2),np.array([x2,y2]))

        targetPose1 = np.array([data1[0],data1[1],Pos1[2,3]+zd1])
        targetPose2 = np.array([data2[0],data2[1],Pos2[2,3]+zd2])
        error1=np.linalg.norm(targetPose1-currentPose1)
        error2=np.linalg.norm(targetPose2-currentPose2)
        start = time.time()
        currentQ1, currentPose1 = nextstep(targetPose1,currentPose1,currentQ1,S,M,error1)
        currentQ2, currentPose2 = nextstep(targetPose2,currentPose2,currentQ2,S,M,error2)
        if leg2==1:
            leg1_hip_joint_pos_con.publish(currentQ2[0][0])
            leg1_knee_joint_pos_con.publish(currentQ2[0][1])
            leg1_ankle_joint_pos_con.publish(currentQ2[0][2])
            time.sleep(0.01)
        if leg1==2: # matriz de rotacion Rz1
            leg2_hip_joint_pos_con.publish(currentQ1[0][0])
            leg2_knee_joint_pos_con.publish(currentQ1[0][1])
            leg2_ankle_joint_pos_con.publish(currentQ1[0][2])
            time.sleep(0.01)
        if leg2==3:
            leg3_hip_joint_pos_con.publish(currentQ2[0][0])
            leg3_knee_joint_pos_con.publish(currentQ2[0][1])
            leg3_ankle_joint_pos_con.publish(currentQ2[0][2])
            time.sleep(0.01)
        if leg1==4: # matriz de rotacion Rz1
            leg4_hip_joint_pos_con.publish(currentQ1[0][0])
            leg4_knee_joint_pos_con.publish(currentQ1[0][1])
            leg4_ankle_joint_pos_con.publish(currentQ1[0][2])
            time.sleep(0.01)
        if leg2==5:
            leg5_hip_joint_pos_con.publish(currentQ2[0][0])
            leg5_knee_joint_pos_con.publish(currentQ2[0][1])
            leg5_ankle_joint_pos_con.publish(currentQ2[0][2])
            time.sleep(0.01)
        if leg1==6: # matriz de rotacion Rz1
            leg6_hip_joint_pos_con.publish(currentQ1[0][0])
            leg6_knee_joint_pos_con.publish(currentQ1[0][1])
            leg6_ankle_joint_pos_con.publish(currentQ1[0][2])
            time.sleep(0.01)
    return currentQ1, currentQ2

if __name__ == '__main__':
    try:
        n=0 #number of iterations
        angle=np.array([[0.175, 0, 0.175, 0.175, 0, 0.175, 0.175, 0, 0.175, 0.175, 0, 0.175, 0.175, 0, 0.175, 0.175, 0, 0.175]])
        angle4 = np.array([[angle[0][1],angle[0][2],angle[0][0]],
                [angle[0][4],angle[0][5],angle[0][3]],
                [angle[0][7],angle[0][8],angle[0][6]],
                [angle[0][10],angle[0][11],angle[0][9]],
                [angle[0][13],angle[0][14],angle[0][12]],
                [angle[0][16],angle[0][17],angle[0][15]]])
        angle5 = np.array([[2.47539e-02,1.09258474,1.22356866],
                [-2.47539390e-02,1.0925874,1.22356866],
                [-6.283185,1.06185831,1.17817347],
                [-6.30793925,1.06185831,1.17817347],
                [-6.3079,1.0925874,1.22356866],
                [-6.2584,1.0925874,1.22356866]])
        print('initial:',angle4)
        hexapod_control(angle4)
        print('forward movement')
        start16=0 #leg1 leg 6
        start25 = 0 #leg 2 leg 5
        start3 = 0 #leg 3
        start4 = 0 # leg4
        paso = 0.110
        while(n<4):
            leg = np.array([4,4,2,5,3,3,6,1])#np.array([6,1,4,4,2,5,3,3])
            for i in range(0,len(leg),2):
                Q1, Q2 = hexapod_controlinit(leg[i],np.array([angle4[leg[i]-1]]),leg[i+1],np.array([angle4[leg[i+1]-1]]))
                print('LEG:',leg[i],leg[i+1])
                if n>0:
                    if i == 0:
                        homeposition(1,0.66*paso)
                        homeposition(2,0)
                        homeposition(3,0.33*paso)
                        angle4[0][0]=0.66*paso
                        angle4[5][0]=-0.66*paso
                        angle4[1][0]=0
                        angle4[2][0]=0.33*paso

                    if i == 1:
                        homeposition(1,0.33*paso)
                        homeposition(3,0)
                        homeposition(4,0.66*paso)
                        angle4[0][0]=0.33*paso
                        angle4[5][0]=-0.33*paso
                        angle4[2][0]=0
                        angle4[3][0]=-0.66*paso
                    if i == 2:
                        homeposition(1,0)
                        homeposition(2,0.66*paso)
                        homeposition(4,0.33*paso)
                        angle4[0][0]=0
                        angle4[5][0]=0
                        angle4[1][0]=-0.66*paso
                        angle4[3][0]=-0.33*paso
                    if i == 3:
                        homeposition(4,0)
                        homeposition(2,0.33*paso)
                        homeposition(3,0.66*paso)
                        angle4[3][0]=0
                        angle4[1][0]=-0.33*paso
                        angle4[4][0]=0.33*paso
                        angle4[2][0]=0.66*paso

                angle4[leg[i]-1][0]=Q1[0][0]
                angle4[leg[i]-1][1]=Q1[0][1]
                angle4[leg[i]-1][2]=Q1[0][2]
                if leg[i] != leg[i+1]:
                    angle4[leg[i+1]-1][0]=Q2[0][0]
                    angle4[leg[i+1]-1][1]=Q2[0][1]
                    angle4[leg[i+1]-1][2]=Q2[0][2]
            print(angle4)
            n=n+1
            print(n)
        print('final',angle4)
        n=0

    except rospy.ROSInterruptException:
        pass

