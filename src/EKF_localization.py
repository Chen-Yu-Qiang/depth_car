#!/usr/bin/wowpython

import numpy as np
import tf
from pyproj import Proj
import sys
import time

DELTA_T=0.2
STATE_NUM=5
WHITE_LIST=list(range(100))
def get_Gt(v,omg,theta):
    Gt=np.eye(STATE_NUM)
    if abs(omg)<10**(-2):
        Gt[0][2]=-v*np.sin(theta)*DELTA_T
        Gt[1][2]=v*np.cos(theta)*DELTA_T
    else:
        Gt[0][2]=-v/omg*np.cos(theta)+v/omg*np.cos(theta+omg*DELTA_T)
        Gt[1][2]=-v/omg*np.sin(theta)+v/omg*np.sin(theta+omg*DELTA_T)
    return Gt

def get_Vt(v,omg,theta):
    Vt=np.zeros((STATE_NUM,2))
    if abs(omg)<10**(-2):
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
    # Mt=np.ones((2,2))*0.5
    alpha1=0.5
    alpha2=0.1
    alpha3=0.01
    alpha4=0.5
    Mt[0][0]=Mt[0][0]+alpha1*v*v+alpha2*omg*omg
    Mt[1][1]=Mt[1][1]+alpha3*v*v+alpha4*omg*omg
    return Mt

def get_ut(ut_1,v,omg):
    theta=ut_1[2][0]
    A=np.zeros((STATE_NUM,1))
    if abs(omg)<10**(-2):
        A[0][0]=v*np.cos(theta)*DELTA_T
        A[1][0]=v*np.sin(theta)*DELTA_T
        A[2][0]=0
    else:
        A[0][0]=-v*np.sin(theta)/omg+v*np.sin(theta+omg*DELTA_T)/omg
        A[1][0]=v*np.cos(theta)/omg-v*np.cos(theta+omg*DELTA_T)/omg
        A[2][0]=omg*DELTA_T

    return ut_1+A

def get_sigma(sigmat_1,gt,vt,mt):
    a=np.zeros((STATE_NUM,STATE_NUM))
    a[3][3]=10**(-5)
    a[4][4]=10**(-5)
    # print( np.dot(np.dot(gt,sigmat_1),gt.T)+np.dot(np.dot(vt,mt),vt.T))
    return np.dot(np.dot(gt,sigmat_1),gt.T)+np.dot(np.dot(vt,mt),vt.T)+a



def a_landmark(sigma,Qt,mx,my,ms,u):
    q=(mx-u[0][0])**2+(my-u[1][0])**2
    z_hat=np.zeros((3,1))
    z_hat[0][0]=np.sqrt(q)
    z_hat[1][0]=((np.arctan2((my-u[1][0]),(mx-u[0][0]))-u[2][0]+np.pi)%(np.pi*2.0))-np.pi
    z_hat[2][0]=ms
    H=np.zeros((3,STATE_NUM))
    H[0][0]=-(mx-u[0][0])/np.sqrt(q)
    H[0][1]=-(my-u[1][0])/np.sqrt(q)
    H[1][0]=(my-u[1][0])/q
    H[1][1]=-(mx-u[0][0])/q
    H[1][2]=-1

    S=np.dot(np.dot(H,sigma),H.T)+Qt

    return z_hat,H,S


def a_landmark_xz(sigma,Qt,mx,my,ms,u):
    z_hat=np.zeros((3,1))
    z_hat[0][0]=(mx-u[0][0])*np.sin(u[2][0])*(-1.0)+(my-u[1][0])*np.cos(u[2][0])
    z_hat[1][0]=(mx-u[0][0])*np.cos(u[2][0])+(my-u[1][0])*np.sin(u[2][0])
    z_hat[2][0]=ms
    H=np.zeros((3,STATE_NUM))
    H[0][0]=np.sin(u[2][0])
    H[0][1]=-np.cos(u[2][0])
    H[0][2]=(mx-u[0][0])*np.cos(u[2][0])*(-1.0)-(my-u[1][0])*np.sin(u[2][0])
    H[1][0]=-np.cos(u[2][0])
    H[1][1]=-np.sin(u[2][0])
    H[1][2]=(-1.0)*(mx-u[0][0])*np.sin(u[2][0])+(my-u[1][0])*np.cos(u[2][0])
    S=np.dot(np.dot(H,sigma),H.T)+Qt

    return z_hat,H,S
def list_2_landmark_Z(a):
    z=np.zeros((3,1))
    z[0][0]=a[3]
    z[1][0]=a[4]
    z[2][0]=a[2]
    return z

def list_2_landmark_Z_together(a,i):
    ARRAY_LAY1=20
    z=np.zeros((3,1))
    z[0][0]=a[ARRAY_LAY1*i+3]
    z[1][0]=a[ARRAY_LAY1*i+4]
    z[2][0]=a[ARRAY_LAY1*i+2]
    return z
def list_2_landmark_xz_Z(a):
    z=np.zeros((3,1))
    z[0][0]=a[0]
    z[1][0]=a[1]
    z[2][0]=a[2]
    return z
def list_2_landmark_xz_Z_together(a,i):
    ARRAY_LAY1=20
    z=np.zeros((3,1))
    z[0][0]=a[ARRAY_LAY1*i+0]
    z[1][0]=a[ARRAY_LAY1*i+1]
    z[2][0]=a[ARRAY_LAY1*i+2]
    return z

def Odom_2_position_Z(a,x0,y0):
    z=np.zeros((3,1))
    z[0][0]=a.pose.pose.position.x+x0
    z[1][0]=a.pose.pose.position.y+y0
    z[2][0]=tf.transformations.euler_from_quaternion([0,0,a.pose.pose.orientation.z,a.pose.pose.orientation.w])[2]
    # print("pose",a,x0,y0,z)
    return z

def Odom_2_angle_Z(a):
    return tf.transformations.euler_from_quaternion([0,0,a.pose.pose.orientation.z,a.pose.pose.orientation.w])[2]

def Vector_2_angle_Z(a):
    return a.vector.z*0.0174532

def gps_2_utm_Z(a):
    z=np.zeros((2,1))
    WD=a.latitude  # WD(N)
    JD=a.longitude # JD(E)
    p = Proj(proj='utm',zone=51,ellps='WGS84', preserve_units=False)
    E,N = p(JD, WD)
    x=N
    y=-E # W
    z[0][0]=x
    z[1][0]=y

    return z

def gps_utm_2_Z(a):
    z=np.zeros((2,1))
    z[0][0]=a.linear.x
    z[1][0]=a.linear.y

    return z

def get_dis(ax,ay,bx,by):
    return np.sqrt((ax-bx)**2+(ay-by)**2)


def set_u_init(x,y,theta,x_offset,y_offset):
    u=np.zeros((STATE_NUM,1))
    u[0][0]=x
    u[1][0]=y
    u[2][0]=theta
    u[3][0]=x_offset
    u[4][0]=y_offset


    return u


class EKF_localization:
    def __init__(self,u_init):
        self.sigma=np.eye(STATE_NUM)*1.0
        self.sigma[3][3]=10**(-1)
        self.sigma[4][4]=10**(-1)
        self.Qt=np.zeros((3,3))
        self.Qt[0][0]=10**(2)
        self.Qt[1][1]=10**(2)
        self.Qt[2][2]=10**(2)
        self.Qt_pos=np.eye(3)
        self.Qt_pos[0][0]=10.0**(-1)
        self.Qt_pos[1][1]=10.0**(-1)
        self.Qt_pos[2][2]=10.0**(-2)
        self.Qt_ang=10**(-5)
        self.Qt_utm=np.eye(2)*10**(-1)
        self.max_j_th=4.0

        self.u=u_init

        self.tree_data=[]

    def prediction(self,v,omg):
        theta=self.u[2][0]

        # print("pre",self.sigma,self.u,v,omg,get_Gt(v,omg,theta),get_Vt(v,omg,theta),get_Mt(v,omg))
        self.u=get_ut(self.u,v,omg)
        self.sigma=get_sigma(self.sigma,get_Gt(v,omg,theta),get_Vt(v,omg,theta),get_Mt(v,omg))
        # print(self.sigma)

    def get_like(self,z_error):
        j_k=np.exp((-0.5)*z_error[0][0]*z_error[0][0])+2.0*np.exp((-2.0)*z_error[1][0]*z_error[1][0])+1.7*np.exp((-1.5)*z_error[2][0]*z_error[2][0])

        return j_k     

    def get_like_xz(self,z_error):
        w=np.zeros((3,3))
        w[0][0]=0.5
        w[1][1]=0.5
        w[2][2]=0.1
        j_k=np.exp((-0.1)*np.dot(z_error.T,np.dot(w,z_error)))[0][0]

        return j_k    
    def update_landmark_sim(self,Z,lock_tree={}):
        z_hat=[]
        H=[]
        S=[]
        j=[]
        max_j_index=0
        max_j=0
        max_j_th=self.max_j_th
        for i in range(len(self.tree_data)):
            mx=self.tree_data[i][0]
            my=self.tree_data[i][1]
            ms=self.tree_data[i][2]
            z_hat_k,H_k,S_k=a_landmark(self.sigma,self.Qt,mx,my,ms,self.u)
            z_hat.append(z_hat_k)
            H.append(H_k)
            S.append(S_k)
            z_error=Z-z_hat_k

            j_k=[[self.get_like(z_error)]]
            # print(i+1,j_k,z_hat_k[0][0],z_hat_k[1][0],z_hat_k[2][0])
            if j_k[0][0]>max_j and not ((i+1) in lock_tree) :
                max_j=j_k[0][0]
                max_j_index=i+1
        
        j=max_j_index-1
        if max_j_index==0 or max_j<max_j_th:
            # print(max_j_index*(-1),max_j,Z,z_hat,-1)
            return max_j_index*(-1),max_j,Z,z_hat[j],-1
        
        # print(j+1,max_j,Z,z_hat[j])
        K=np.dot(np.dot(self.sigma,H[j].T),np.linalg.inv(S[j]))
        return max_j_index,max_j,Z,z_hat[j],np.dot(K,(Z-z_hat[j]))

    def update_landmark_know_cor(self,Z,i):
        if i<1:
            return i,0,Z,np.zeros((3,1))*(-1.0),np.ones((3,1))*(-1.0)
        print("update_landmark_know_cor",i)
        mx=self.tree_data[int(i-1)][0]
        my=self.tree_data[int(i-1)][1]
        ms=self.tree_data[int(i-1)][2]

        z_hat,H,S=a_landmark(self.sigma,self.Qt,mx,my,ms,self.u)


        K=np.dot(np.dot(self.sigma,H.T),np.linalg.inv(S))

        z_error=Z-z_hat

        j_k=self.get_like(z_error)

        delta=np.dot(K,(Z-z_hat))
        if j_k<self.max_j_th:
            print("likelihood too small")
            return i*(-1.0),j_k,Z,z_hat,np.ones((3,1))*(-1.0)
        elif abs(delta[0][0])>0.4 or abs(delta[1][0])>0.4 or abs(delta[2][0])>0.1:
            print("Delta too big")
            return i*(-1.0),j_k,Z,z_hat,np.ones((3,1))*(-1.0)
        else:
        #     print("self.u ",self.u)
        #     print("delta ",delta)
        #     print("K ",K)
        #     print("inv(S) ",np.linalg.inv(S))

            self.u=self.u+delta
            self.sigma=np.dot((np.eye(STATE_NUM)-np.dot(K,H)),self.sigma)
            return i,j_k,Z,z_hat,delta

    def update_landmark(self,Z):
        max_j_index,max_j,_,z_hat,_=self.update_landmark_sim(Z)
        if max_j_index>0:
            _,_,_,_,delta=self.update_landmark_know_cor(Z,max_j_index)
            return max_j_index,max_j,Z,z_hat,delta
        else:
            return max_j_index,max_j,Z,z_hat,-1



    def update_landmark_xz_know_cor(self,Z,i):
        if i<1:
            return i,0,Z,np.zeros((3,1))*(-1.0),np.ones((3,1))*(-1.0)

        mx=self.tree_data[int(i-1)][0]
        my=self.tree_data[int(i-1)][1]
        ms=self.tree_data[int(i-1)][2]
        z_hat,H,S=a_landmark_xz(self.sigma,self.Qt,mx,my,ms,self.u)

        z_error=Z-z_hat
        K=np.dot(np.dot(self.sigma,H.T),np.linalg.inv(S))
        delta=np.dot(K,(Z-z_hat))

        j_k=self.get_like_xz(z_error)
        delta[2][0]=0
        if j_k<self.max_j_th:
            print("likelihood too small",i,j_k)
            return i*(-1.0),j_k,Z,z_hat,np.ones((3,1))*(-1.0)
        elif abs(delta[0][0])>1.0 or abs(delta[1][0])>1.0:
            print("XY Delta too big",i,j_k,delta[0][0],delta[1][0])
            return i*(-1.0),j_k,Z,z_hat,np.ones((3,1))*(-1.0)
        elif abs(delta[2][0])>0.1:
            print("THETA Delta too big",delta[2][0])
            return i*(-1.0),j_k,Z,z_hat,np.ones((3,1))*(-1.0)
        else:        
            self.u=self.u+delta
            self.sigma=np.dot((np.eye(STATE_NUM)-np.dot(K,H)),self.sigma)
            return i,j_k,Z,z_hat,delta

    def update_landmark_xz_sim(self,Z):
        z_hat=[]
        H=[]
        S=[]
        j=[]
        max_j_index=0
        max_j=0
        max_j_th=self.max_j_th
        for i in range(len(self.tree_data)):
            if not i in WHITE_LIST:

                z_hat.append(0)
                H.append(0)
                S.append(0)
                continue
            mx=self.tree_data[i][0]
            my=self.tree_data[i][1]
            ms=self.tree_data[i][2]
            z_hat_k,H_k,S_k=a_landmark_xz(self.sigma,self.Qt,mx,my,ms,self.u)
            z_hat.append(z_hat_k)
            H.append(H_k)
            S.append(S_k)
            z_error=Z-z_hat_k
            # z_error[1][0]=z_error[1][0]*3
            # j_k=(np.linalg.det(2*np.pi*S_k)**(-0.5))*np.exp((-0.5)*np.dot(np.dot(z_error.T,np.linalg.inv(S_k)),z_error))
            if i in WHITE_LIST:
                j_k=[[self.get_like_xz(z_error)]]
            else:
                j_k=-100
            # print(i+1,j_k,z_hat_k[0][0],z_hat_k[1][0])
            if j_k[0][0]>max_j:
                max_j=j_k[0][0]
                max_j_index=i+1
        
        j=max_j_index-1
        if max_j_index==0 or max_j<max_j_th:
            # print(max_j_index*(-1),max_j,Z,z_hat,-1)
            return max_j_index*(-1),max_j,Z,z_hat[j],-1
        
        # print(j+1,max_j,Z,z_hat[j])
        K=np.dot(np.dot(self.sigma,H[j].T),np.linalg.inv(S[j]))
        return max_j_index,max_j,Z,z_hat[j],np.dot(K,(Z-z_hat[j]))


    def update_landmark_xz(self,Z):
        tt=time.time()
        max_j_index,max_j,_,z_hat,_=self.update_landmark_xz_sim(Z)
        # print(time.time()-tt)
        if max_j_index>0:
            max_j_index,_,_,_,delta=self.update_landmark_xz_know_cor(Z,max_j_index)
            return max_j_index,max_j,Z,z_hat,delta
        else:
            return max_j_index,max_j,Z,z_hat,-1


    def update_positon(self,Z):
        z_hat=self.u
        n2pi=int((z_hat[2][0]+np.pi)/(2*np.pi))
        e_p=abs(Z[2][0]+(n2pi+1)*2*np.pi-z_hat[2][0])
        e_n=abs(Z[2][0]+(n2pi-1)*2*np.pi-z_hat[2][0])
        e=abs(Z[2][0]+(n2pi)*2*np.pi-z_hat[2][0])
        if e_p<e and e_p<e_n:
            Z[2][0]=Z[2][0]+(n2pi+1)*2*np.pi
        elif e_n<e and e_n<e_p:
            Z[2][0]=Z[2][0]+(n2pi-1)*2*np.pi
        elif e<e_n and e<e_p:
            Z[2][0]=Z[2][0]+(n2pi)*2*np.pi
        # print(z_hat[2][0],Z[2][0],n2pi,e,e_p,e_n)

        H=np.zeros((3,STATE_NUM))
        H[0][0]=1
        H[1][1]=1
        H[2][2]=1
        H[0][3]=1
        H[1][4]=1
        z_hat=np.dot(H,self.u)
        S=np.dot(H,np.dot(self.sigma,H.T))+self.Qt_pos
        K=np.dot(np.dot(self.sigma,H.T),np.linalg.inv(S))

        delta=np.dot(K,(Z-z_hat))

        delta[0][0]=min(max(delta[0][0],-0.4),0.4)
        delta[1][0]=min(max(delta[1][0],-0.4),0.4)
        delta[2][0]=min(max(delta[2][0],-0.1),0.1)
        self.u=self.u+delta
        self.sigma=np.dot((np.eye(STATE_NUM)-np.dot(K,H)),self.sigma)
        return



    def update_angle(self,Z):
        z_hat=self.u[2][0]
        n2pi=int((z_hat+np.pi)/(2*np.pi))
        e_p=abs(Z+(n2pi+1)*2*np.pi-z_hat)
        e_n=abs(Z+(n2pi-1)*2*np.pi-z_hat)
        e=abs(Z+(n2pi)*2*np.pi-z_hat)
        if e_p<e and e_p<e_n:
            Z=Z+(n2pi+1)*2*np.pi
        elif e_n<e and e_n<e_p:
            Z=Z+(n2pi-1)*2*np.pi
        elif e<e_n and e<e_p:
            Z=Z+(n2pi)*2*np.pi
        # print(z_hat,Z,n2pi,e,e_p,e_n)
        H=np.zeros((1,STATE_NUM))
        H[0][2]=1
        S=np.dot(H,np.dot(self.sigma,H.T))+self.Qt_ang
        # print("ang",self.sigma)
        K=np.dot(np.dot(self.sigma,H.T),np.linalg.inv(S))

        delta=np.dot(K,(Z-z_hat))

        delta[0][0]=min(max(delta[0][0],-0.4),0.4)
        delta[1][0]=min(max(delta[1][0],-0.4),0.4)
        delta[2][0]=min(max(delta[2][0],-0.1),0.1)
        self.u=self.u+delta
        self.sigma=np.dot((np.eye(STATE_NUM)-np.dot(K,H)),self.sigma)
        return

    def update_gps_utm(self,Z):
        z_hat=self.u[0:2]+self.u[3:5]

        H=np.zeros((2,STATE_NUM))
        H[0][0]=1
        H[0][3]=1

        H[1][1]=1
        H[1][4]=1
        z_hat=np.dot(H,self.u)
        S=np.dot(H,np.dot(self.sigma,H.T))+self.Qt_utm
        # print("gps",self.sigma)
        K=np.dot(np.dot(self.sigma,H.T),np.linalg.inv(S))

        delta=np.dot(K,(Z-z_hat))

        delta[0][0]=min(max(delta[0][0],-0.4),0.4)
        delta[1][0]=min(max(delta[1][0],-0.4),0.4)
        delta[2][0]=min(max(delta[2][0],-0.1),0.1)
        self.u=self.u+delta
        self.sigma=np.dot((np.eye(STATE_NUM)-np.dot(K,H)),self.sigma)
        return