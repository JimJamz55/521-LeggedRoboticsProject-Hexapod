#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  6 10:56:07 2022

@author: vgrefa
"""
import numpy as np
import math
import time
import matplotlib.pyplot as plt

def adjointa(V,T):
    R = T[:3,:3]
    p = T[:3,3]
    w = np.array([[0, -p[2], p[1]], 
                  [p[2], 0, -p[0]],
                  [-p[1], p[0], 0]])
    x = np.concatenate((R,np.dot(w,R)),axis=0)
    y = np.concatenate((0*np.identity(3),R),axis=0)
    z = np.concatenate((x,y),axis=1)
    Vtrans = np.dot(z,V)
    return Vtrans

def fkine(S,M,q,frame):
    size = S.shape
    #[x,y]=size(S);
    e0=np.identity(4)
    for i in range(0,size[1]):
        ei=twist2ht(S[:,i],q[0][i])
        e0=np.dot(e0,ei)
    if frame=='body':
        T=np.dot(M,e0)
    if frame == 'space':
       T = np.dot(e0,M)
    return T

def twistspace2body(V_s,T):
    R = T[:3,:3]
    p = T[:3,3:4]
    T = np.concatenate((np.linalg.inv(R), -np.dot(np.linalg.inv(R),p)),axis=1)
    T = np.concatenate((T,np.array([[0,0,0,1]])),axis=0)
    R = T[:3,:3]
    p = T[:3,3:4]
    w = np.array([[0, -p[2][0], p[1][0]], 
                  [p[2][0], 0, -p[0][0]],
                  [-p[1][0], p[0][0], 0]])
    x = np.concatenate((R,0*np.identity(3)),axis=1)
    y = np.concatenate((np.dot(w,R),R),axis=1)
    z = np.concatenate((x,y),axis=0)
    V_b = np.dot(z,V_s)
    return V_b

def twist2ht(S,theta):
    omega=np.array([[S[0], S[1], S[2]]])
    v=np.array([[S[3]], [S[4]], [S[5]]])
    w = np.array([[0, -omega[0][2], omega[0][1]],
                  [omega[0][2], 0, -omega[0][0]],
                  [-omega[0][1], omega[0][0], 0]])
    R = np.identity(3)+np.dot(math.sin(theta),w)+(1-math.cos(theta))*np.dot(w,w)
    Rv = np.dot((np.identity(3)*theta+(1-math.cos(theta))*w+(theta-math.sin(theta))*np.dot(w,w)),v)
    T = np.concatenate((R,Rv),axis=1)
    T = np.concatenate((T,np.array([[0,0,0,1]])),axis=0)
    return T

def jacob0(S,q):
    size = S.shape
    jac=np.zeros((6,size[1]))
    jan=np.zeros((6,size[1]))
    jac[:,0]=S[:,0]
    if (size[1] > 1):
        for i in range(size[1]-1,0,-1): 
            ei=twist2ht(S[:,i-1],q[0][i-1])
            jac[:,i]=adjointa(S[:,i],ei)
            for j in range(i-1,0,-1): 
                ej=twist2ht(S[:,j-1],q[0][j-1])
                jac[:,i]=adjointa(jac[:,i],ej)
    J=jac;
    return J

def jacobe(S,M,q):
    J_s = jacob0(S,q)
    T = fkine(S,M,q,'space')
    J_b = twistspace2body(J_s,T)
    return J_b

def jacoba(S,M,q):  
    J_b = jacobe(S,M,q)
    T = fkine(S,M,q,'space')
    Rot =  T[:3,:3]
    J_a = np.dot(Rot,J_b[3:6,:])
    return J_a

def quinticpoly(t0,tf,q0,qf):
    Ac = np.array([[q0], [0], [0], [qf], [0], [0]])
    Quintic = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                        [1, tf, tf**2, tf**3, tf**4, tf**5],
                        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
    A = np.dot(np.linalg.inv(Quintic),Ac)
    return A

def semicircle(A,tt,ht):
    a1=A[0][0]
    a2=A[1][0]
    a3=A[2][0]
    a4=A[3][0]
    a5=A[4][0]
    a6=A[5][0]
    # x position desired
    xd = a1 + a2*tt + a3*tt**2 + a4*tt**3 + a5*tt**4 + a6*tt**5; 
    # Trajectory for velocity in x, xdp 
    xdp = a2 + 2* a3*tt + 3* a4*tt**2 + 4*a5*tt**3 + 5*a6*tt**4;
    # Because is a semi-circle: H^2 = x^2+y^2
    yd = math.sqrt((ht**2 - (xd - ht)**2));
    # Diferential of velocity in y
    if yd != 0:
        ydp = (2*ht-2*xd)/(2*yd)
    else: ydp = 0
    #return xd,yd.real,xdp,ydp.real
    return xd, yd.real, xdp, ydp

def nextstep(targetPose,currentPose,currentQ,S,M,error):
    while (error>1e-3):
        J_a = jacoba(S, M, currentQ)
        J = np.dot(J_a.transpose(),np.linalg.pinv(np.dot(J_a,J_a.transpose())+0.05*np.identity(3)))
        deltaQ = np.dot(J,(targetPose-currentPose))
        currentQ = currentQ + deltaQ.transpose()
        T = fkine(S,M,currentQ,'space')
        currentPose = np.array([T[0,3], T[1,3], T[2,3]])
        error = np.linalg.norm(targetPose-currentPose)
    return currentQ, currentPose

