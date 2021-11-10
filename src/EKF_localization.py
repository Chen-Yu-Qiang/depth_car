#!/usr/bin/env python

import numpy as np
import tf
DELTA_T=0.1
def get_Gt(v,omg,theta):
    Gt=np.eye(3)
    if omg==0:
        Gt[0][2]=-v*np.sin(theta)*DELTA_T
        Gt[1][2]=v*np.cos(theta)*DELTA_T
    else:
        Gt[0][2]=-v/omg*np.cos(theta)+v/omg*np.cos(theta+omg*DELTA_T)
        Gt[1][2]=-v/omg*np.sin(theta)+v/omg*np.sin(theta+omg*DELTA_T)
    return Gt

def get_Vt(v,omg,theta):
    Vt=np.zeros((3,2))
    if omg==0:
        Vt[0][0]=np.cos(theta)*DELTA_T
        Vt[1][0]=np.sin(theta)*DELTA_T
        Vt[0][1]=-v*np.sin(theta)*DELTA_T*DELTA_T*0.5
        Vt[1][1]=v*np.cos(theta)*DELTA_T*DELTA_T*0.5
    else:
        Vt[0][0]=(-np.sin(theta)+np.sin(theta+omg*DELTA_T))/omg
        Vt[1][0]=(np.cos(theta)-np.cos(theta+omg*DELTA_T))/omg
        Vt[0][1]=v*(np.sin(theta)-np.sin(theta+omg*DELTA_T))/(omg*omg)+v*np.cos(theta+omg*DELTA_T)*DELTA_T/omg
        Vt[1][1]=-v*(np.cos(theta)-np.cos(theta+omg*DELTA_T))/(omg*omg)+v*np.sin(theta+omg*DELTA_T)*DELTA_T/omg
    Vt[2][1]=DELTA_T
    return Vt

def get_Mt(v,omg):
    Mt=np.zeros((2,2))
    alpha1=1
    alpha2=0
    alpha3=0
    alpha4=1
    Mt[0][0]=alpha1*v*v+alpha2*omg*omg
    Mt[1][1]=alpha3*v*v+alpha4*omg*omg
    return Mt

def get_ut(ut_1,v,omg):
    theta=ut_1[2][0]
    A=np.zeros((3,1))
    if omg==0:
        A[0][0]=v*np.cos(theta)*DELTA_T
        A[1][0]=v*np.sin(theta)*DELTA_T
        A[2][0]=0
    else:
        A[0][0]=-v*np.sin(theta)/omg+v*np.sin(theta+omg*DELTA_T)/omg
        A[1][0]=v*np.cos(theta)/omg-v*np.cos(theta+omg*DELTA_T)/omg
        A[2][0]=omg*DELTA_T
    return ut_1+A

def get_sigma(sigmat_1,gt,vt,mt):
    return np.dot(np.dot(gt,sigmat_1),gt.T)+np.dot(np.dot(vt,mt),vt.T)



def a_landmark(sigma,Qt,mx,my,ms,u):
    q=(mx-u[0][0])**2+(my-u[1][0])**2
    z_hat=np.zeros((3,1))
    z_hat[0][0]=np.sqrt(q)
    z_hat[1][0]=np.arctan2((my-u[1][0]),(mx-u[0][0]))-u[2][0]
    z_hat[2][0]=ms
    H=np.zeros((3,3))
    H[0][0]=-(mx-u[0][0])/np.sqrt(q)
    H[0][1]=-(my-u[1][0])/np.sqrt(q)
    H[1][0]=(my-u[1][0])/q
    H[1][1]=-(mx-u[0][0])/q
    H[1][2]=-1

    S=np.dot(np.dot(H,sigma),H.T)+Qt

    return z_hat,H,S
def list_2_landmark_Z(a):
    z=np.zeros((3,1))
    z[0][0]=a[3]
    z[1][0]=a[4]
    z[2][0]=a[2]
    return z

def Odom_2_position_Z(a):
    z=np.zeros((3,1))
    z[0][0]=a.pose.pose.position.x
    z[1][0]=a.pose.pose.position.y
    z[2][0]=tf.transformations.euler_from_quaternion([0,0,a.pose.pose.orientation.z,a.pose.pose.orientation.w])[2]
    return z

def set_u_init(x,y,theta):
    u=np.zeros((3,1))
    u[0][0]=x
    u[1][0]=y
    u[2][0]=theta
    return u

class EKF_localization:
    def __init__(self,u_init):
        self.sigma=np.eye(3)
        self.Qt=np.zeros((3,3))
        self.Qt[0][0]=10**(-2)
        self.Qt[1][1]=10**(-2)
        self.Qt[2][2]=100
        self.Qt2=np.eye(3)

        self.u=u_init
        tree_data_1900=[[2.5,12.5,0.5],[3.5,5.0,0.5],[4.0,-1.0,0.5],[9.0,10.0,0.5],[9.5,6.0,0.5],[10.0,3.0,0.5],[13.5,8.0,0.5]]
        self.tree_data=tree_data_1900

    def prediction(self,v,omg):
        theta=self.u[2][0]
        self.u=get_ut(self.u,v,omg)
        self.sigma=get_sigma(self.sigma,get_Gt(v,omg,theta),get_Vt(v,omg,theta),get_Mt(v,omg))

    

    def update_landmark(self,Z):
        z_hat=[]
        H=[]
        S=[]
        j=[]
        max_j_index=-1
        max_j=0.1
        for i in range(len(self.tree_data)):
            mx=self.tree_data[i][0]
            my=self.tree_data[i][1]
            ms=self.tree_data[i][2]
            z_hat_k,H_k,S_k=a_landmark(self.sigma,self.Qt,mx,my,ms,self.u)
            z_hat.append(z_hat_k)
            H.append(H_k)
            S.append(S_k)
            z_error=Z-z_hat_k
            z_error[2][0]=0
            j_k=np.linalg.det(2*np.pi*S_k)**(-0.5)*np.exp((-0.5)*np.dot(np.dot(z_error.T,np.linalg.inv(S_k)),z_error))
            if j_k>max_j:
                max_j=j_k
                max_j_index=i
        if max_j_index==-1:
            return
        j=max_j_index
        print(j+1,max_j[0][0],Z,z_hat[j])
        K=np.dot(np.dot(self.sigma,H[j].T),np.linalg.inv(S[j]))
        self.u=self.u+np.dot(K,(Z-z_hat[j]))
        self.sigma=np.dot((np.eye(3)-np.dot(K,H[j])),self.sigma)
        return


    def update_positon(self,Z):
        z_hat=self.u
        S=self.sigma+self.Qt2
        K=np.dot(self.sigma,np.linalg.inv(S))
        self.u=self.u+np.dot(K,(Z-z_hat))
        self.sigma=np.dot((np.eye(3)-K),self.sigma)
        return
