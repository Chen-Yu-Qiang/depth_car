#!/usr/bin/env python
import os
import rospy
import numpy as np

class KF_updater:
    def __init__(self,n,kf):
        self.R=np.eye(n)
        self.H=np.zeros((n,kf.n))
        self.kf=kf
    
    def constantSpeedWDrift_Speed(self,R_v):
        self.H=np.array([[0,1,0]])
        self.R[0][0]=R_v
    def constantSpeedWDrift_SpeedWDrift(self,R_v):
        self.H=np.array([[0,1,1]])
        self.R[0][0]=R_v
    def constantSpeedWDrift_Position(self,R_p):
        self.H=np.array([[1,0,0]])
        self.R[0][0]=R_p
    def constantSpeed_Position(self,R_p):
        self.H=np.array([[1,0]])
        self.R[0][0]=R_p
    def constantPosition_Position(self,R_p):
        self.H=np.array([[1]])
        self.R[0][0]=R_p

    def update(self,Z):
        Y=Z-np.dot(self.H,self.kf.X)
        S=np.dot(np.dot(self.H,self.kf.P),np.transpose(self.H))+self.R
        K=np.dot(np.dot(self.kf.P,np.transpose(self.H)),np.linalg.inv(S))
        self.kf.X=self.kf.X+np.dot(K,Y)
        self.kf.P=np.dot(np.eye(self.kf.n)-np.dot(K,self.H),self.kf.P)
        # print(self.kf.X,np.dot(K,Y))


        
class KalmanFilter:
    def __init__(self,n):
        self.F=np.eye(n)
        self.B=np.array([[]])
        self.Q=np.eye(n)
        self.P=np.eye(n)
        self.X=np.zeros((n,1))
        self.n=n
        self.F_future=np.eye(n)
    def get_future(self):
        my_x=self.X
        my_x=np.dot(self.F_future,my_x)
        
        return my_x

    def prediction(self,u):
        self.X = np.dot(self.F,self.X)+np.dot(self.B,u)
        self.P = np.dot(np.dot(self.F,self.P),np.transpose(self.F)) + self.Q
    
    def constantSpeedWDrift(self,dt,x0_p,x0_v,x0_v_drift,Q_p,Q_v,Q_v_drift):
        self.F=np.array([[1,dt,0],[0,1,0],[0,0,1]])
        self.Q[0][0]=Q_p
        self.Q[1][1]=Q_v
        self.Q[2][2]=Q_v_drift
        self.X[0][0]=x0_p
        self.X[1][0]=x0_v
        self.X[2][0]=x0_v_drift
        self.F_future=self.F
        for i in range(90):
            self.F_future=np.dot(self.F,self.F_future)
    def constantSpeed(self,dt,x0_p,x0_v,Q_p,Q_v):
        self.F=np.array([[1,dt],[0,1]])
        self.Q[0][0]=Q_p
        self.Q[1][1]=Q_v
        self.X[0][0]=x0_p
        self.X[1][0]=x0_v
        self.F_future=self.F
        for i in range(90):
            self.F_future=np.dot(self.F,self.F_future)
    def constantPosition(self,dt,x0_p,Q_p):
        self.F=np.array([[1]])
        self.Q[0][0]=Q_p
        self.X[0][0]=x0_p
        self.F_future=self.F
        for i in range(90):
            self.F_future=np.dot(self.F,self.F_future)
class EKF_updater:
    def __init__(self,n,kf):
        self.R=np.eye(n)
        self.H=None
        self.H_d=None
        self.kf=kf


    def update(self,Z):
        Y=Z-self.H(self.kf.X)
        S=np.dot(np.dot(self.H_d(self.kf.X),self.kf.P),np.transpose(self.H_d(self.kf.X)))+self.R
        K=np.dot(np.dot(self.kf.P,np.transpose(self.H_d(self.kf.X))),np.linalg.inv(S))
        self.kf.X=self.kf.X+np.dot(K,Y)
        self.kf.P=np.dot(np.eye(self.kf.n)-np.dot(K,self.H_d(self.kf.X)),self.kf.P)


class ExtendedKalmanFilter:
    def __init__(self,n):
        self.F=None
        self.F_d=None
        self.Q=np.eye(n)
        self.P=np.eye(n)
        self.X=np.zeros((n,1))
        self.n=n

    def prediction(self,u):
        self.X = self.F(self.X,u)
        self.P = np.dot(np.dot(self.F_d(self.X,u),self.P),np.transpose(self.F_d(self.X,u))) + self.Q
    


if __name__ == '__main__':
    kf=KalmanFilter(2)
    kf.F[0][1]=1
    kf.X[1]=1
    measure1=KF_updater(2,kf)
    measure1.H[0][0]=1
    measure1.H[1][1]=1

    
    for t in range(0,100):
        kf.prediction([])
        if t%10==0:
            measure1.update(np.array([[1],[1]]))
        print(kf.X)

