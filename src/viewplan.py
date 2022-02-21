#!/usr/bin/env wowpython
import numpy as np
import time
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import TREEDATA
ALPHA_H=0.6119
# ALPHA_H=0.5
ALPHA_V=0.75
Z_T=0.2
Z_B=7
THETA_A=np.pi/6
RHO=0.5


def cameraFrame2WorldFrame(cpk,ci):
    theta=ci[3]
    pk=[0.0,0.0,0.0,0.0]
    pk[0]=cpk[0]*np.cos(theta)-cpk[2]*np.sin(theta)+ci[0]
    pk[1]=cpk[0]*np.sin(theta)+cpk[2]*np.cos(theta)+ci[1]
    pk[2]=(-1.0)*cpk[1]+ci[2]
    pk[3]=cpk[3]+theta
    return pk

def worldFrame2CameraFrame(pk,ci):
    theta=ci[3]
    cpk=[0.0,0.0,0.0,0.0]
    t=[0.0,0.0,0.0]
    t[0]=pk[0]-ci[0]
    t[1]=pk[1]-ci[1]
    t[2]=pk[2]-ci[2]
    cpk[0]=t[0]*np.cos(theta)+t[1]*np.sin(theta)
    cpk[1]=(-1.0)*t[2]
    cpk[2]=(-1.0)*t[0]*np.sin(theta)+t[1]*np.cos(theta)
    cpk[3]=np.arctan2(np.sin(pk[3]-theta),np.cos(pk[3]-theta))
    return cpk

def ciSpace2tiSpace(cpk):
    tpk=[0.0,0.0,0.0,0.0]
    tpk[0]=cpk[0]/(max(abs(cpk[2]),0.001)*np.tan(ALPHA_H))
    tpk[1]=cpk[1]/(max(abs(cpk[2]),0.001)*np.tan(ALPHA_V))
 
    tpk[2]=(2.0*cpk[2]-Z_T-Z_B)/(Z_B-Z_T)
    # tpk[3]=np.tan(abs(cpk[3])*0.5)/np.tan(THETA_A*0.5)
    tpk[3]=0
    return tpk

def d_v(pk,ci):
    cpk=worldFrame2CameraFrame(pk,ci)
    if cpk[2]<=0:
        cpk[2]=0
    tpk=ciSpace2tiSpace(cpk)
    # print("max",tpk[0],tpk[1],tpk[2],tpk[3])
    if cpk[2]>=0:
        # print("cpk[2]>0",cpk,tpk,ci)
        return max(abs(tpk[0]),abs(tpk[1]),abs(tpk[2]),abs(tpk[3]))
    if cpk[2]<=0:
        # print("cpk[2]<=0",cpk,tpk,ci)
        return max(abs(tpk[0]),abs(tpk[1]),abs(tpk[2]),abs(tpk[3]))

def C_s(pk,ci):
    th=1

    if d_v(pk,ci)>th:
        return 0
    else:
        # return (-1.0)*RHO*d_v(pk,ci)*d_v(pk,ci)
        return np.exp((-1.0)*RHO*d_v(pk,ci)*d_v(pk,ci))
        # return np.exp((-1.0)*RHO*d_v(pk,ci))
def C_s_all(_taskpoint,ci):
    a=0
    for i in _taskpoint:
        a=a+C_s(i,ci)
    return a
def whatPartition(pk,ci):
    cpk=worldFrame2CameraFrame(pk,ci)
    tpk=ciSpace2tiSpace(cpk)
    if d_v(pk,ci)==tpk[0]:
        return 1
    elif d_v(pk,ci)==(-1.0)*tpk[0]:
        return 2
    elif d_v(pk,ci)==tpk[1]:
        return 3
    elif d_v(pk,ci)==(-1.0)*tpk[1]:
        return 4
    elif d_v(pk,ci)==tpk[2]:
        return 5
    elif d_v(pk,ci)==(-1.0)*tpk[2]:
        return 6
    elif d_v(pk,ci)==tpk[3]:
        return 7

def Partial_C_s_Partial_ci(pk,ci,partition):
    _Partial_C_s_Partial_ci=[0.0,0.0,0.0,0.0]
    partial_C_s_partial_dv=(-1.0)*RHO*np.exp((-1.0)*RHO*d_v(pk,ci))
    # partial_C_s_partial_dv=(-2.0)*d_v(pk,ci)*RHO*np.exp((-1.0)*RHO*d_v(pk,ci)*d_v(pk,ci))
    # partial_C_s_partial_dv=(-2.0)*RHO*d_v(pk,ci)
    cpk=worldFrame2CameraFrame(pk,ci)
    if partition==7:
        if cpk[3]<0:
            Partial_dv_Partial_ci_3=(1.0/(2.0*np.tan(THETA_A/2)))*(1.0/(np.cos(cpk[3]/2)*np.cos(cpk[3]/2)))
        else:
            Partial_dv_Partial_ci_3=(1.0/(2.0*np.tan(THETA_A/2)))*(1.0/(np.cos(cpk[3]/2)*np.cos(cpk[3]/2)))*(-1)

        _Partial_C_s_Partial_ci=[0,0,0,partial_C_s_partial_dv*Partial_dv_Partial_ci_3]
    else:
        Partial_cpk_Partial_theta=[0.0,0.0,0.0]
        # Partial_cpk_Partial_theta[0]=np.sin(pk[3])*(-1.0)*(pk[0]-ci[0])-np.cos(pk[3])*(pk[2]-ci[2])
        # Partial_cpk_Partial_theta[1]=np.cos(pk[3])*(pk[0]-ci[0])-np.sin(pk[3])*(pk[2]-ci[2])
        # Partial_cpk_Partial_theta[0]=np.sin(pk[3])*(-1.0)*(pk[0]-ci[0])+np.cos(pk[3])*(pk[1]-ci[1])
        # Partial_cpk_Partial_theta[1]=np.cos(pk[3])*(pk[0]-ci[0])*(-1.0)-np.sin(pk[3])*(pk[1]-ci[1])
        Partial_cpk_Partial_theta[0]=np.sin(ci[3])*(-1.0)*(pk[0]-ci[0])+np.cos(ci[3])*(pk[1]-ci[1])
        Partial_cpk_Partial_theta[1]=np.cos(ci[3])*(pk[0]-ci[0])*(-1.0)-np.sin(ci[3])*(pk[1]-ci[1])
        # Partial_cpk_Partial_ci = [-R^T , Partial_cpk_Partial_theta]_{3*4}

        Partial_dv_Partial_cpk=[0.0,0.0,0.0]
        if partition==1:
            Partial_dv_Partial_cpk[0]=1.0/(cpk[2]*np.tan(ALPHA_H))
            Partial_dv_Partial_cpk[1]=0.0
            Partial_dv_Partial_cpk[2]=(-1.0*cpk[0])/(cpk[2]*cpk[2]*np.tan(ALPHA_H))
        elif partition==2:
            Partial_dv_Partial_cpk[0]=(-1.0)/(cpk[2]*np.tan(ALPHA_H))
            Partial_dv_Partial_cpk[1]=0.0
            Partial_dv_Partial_cpk[2]=(1.0*cpk[0])/(cpk[2]*cpk[2]*np.tan(ALPHA_H))
        elif partition==3:
            Partial_dv_Partial_cpk[0]=0.0
            Partial_dv_Partial_cpk[1]=(1.0)/(cpk[2]*np.tan(ALPHA_V))
            Partial_dv_Partial_cpk[2]=(-1.0*cpk[1])/(cpk[2]*cpk[2]*np.tan(ALPHA_V))
        elif partition==4:
            Partial_dv_Partial_cpk[0]=0.0
            Partial_dv_Partial_cpk[1]=(-1.0)/(cpk[2]*np.tan(ALPHA_V))
            Partial_dv_Partial_cpk[2]=(1.0*cpk[1])/(cpk[2]*cpk[2]*np.tan(ALPHA_V))
        elif partition==5:
            Partial_dv_Partial_cpk[0]=0.0
            Partial_dv_Partial_cpk[1]=0.0
            Partial_dv_Partial_cpk[2]=(2.0)/(Z_B-Z_T)
        elif partition==6:
            Partial_dv_Partial_cpk[0]=0.0
            Partial_dv_Partial_cpk[1]=0.0
            Partial_dv_Partial_cpk[2]=(-2.0)/(Z_B-Z_T)
        
        Partial_dv_Partial_ci=[0.0,0.0,0.0,0.0]
        # Partial_dv_Partial_ci[0]=Partial_dv_Partial_cpk[0]*(-np.cos(pk[3]))+Partial_dv_Partial_cpk[2]*(np.sin(pk[3]))
        # Partial_dv_Partial_ci[1]=Partial_dv_Partial_cpk[0]*(-np.sin(pk[3]))-Partial_dv_Partial_cpk[2]*(np.cos(pk[3]))
        Partial_dv_Partial_ci[0]=Partial_dv_Partial_cpk[0]*(-np.cos(ci[3]))+Partial_dv_Partial_cpk[2]*(np.sin(ci[3]))
        Partial_dv_Partial_ci[1]=Partial_dv_Partial_cpk[0]*(-np.sin(ci[3]))-Partial_dv_Partial_cpk[2]*(np.cos(ci[3]))
        Partial_dv_Partial_ci[2]=Partial_dv_Partial_cpk[1]
        Partial_dv_Partial_ci[3]=Partial_dv_Partial_cpk[0]*Partial_cpk_Partial_theta[0]+Partial_dv_Partial_cpk[1]*Partial_cpk_Partial_theta[1]

        _Partial_C_s_Partial_ci[0]=partial_C_s_partial_dv*Partial_dv_Partial_ci[0]
        _Partial_C_s_Partial_ci[1]=partial_C_s_partial_dv*Partial_dv_Partial_ci[1]
        _Partial_C_s_Partial_ci[2]=partial_C_s_partial_dv*Partial_dv_Partial_ci[2]
        _Partial_C_s_Partial_ci[3]=partial_C_s_partial_dv*Partial_dv_Partial_ci[3]

    return _Partial_C_s_Partial_ci


def gradient(pk,ci,it_num,it_length):
    partition=whatPartition(pk,ci)
    all_delta=[0.0,0.0,0.0,0.0]
    for i in range(it_num):
        delta=Partial_C_s_Partial_ci(pk,ci,partition)
        ci[0]=ci[0]+it_length*delta[0]
        ci[1]=ci[1]+it_length*delta[1]
        ci[2]=ci[2]+it_length*delta[2]
        ci[3]=ci[3]+it_length*delta[3]
        all_delta[0]=delta[0]
        all_delta[1]=delta[1]
        all_delta[2]=delta[2]
        all_delta[3]=delta[3]
    return ci,all_delta

def plot_contourf(taskPoint,z=0,th=0,x_min=0,x_max=10,y_min=0,y_max=10):
    x_list=np.linspace(x_min,x_max,101)
    y_list=np.linspace(y_min,y_max,101)

    def f(x,y):
        C_s_sum=0
        _ci=[x,y,z,th]
        for i in range(len(taskPoint)):
            C_s_sum=C_s_sum+C_s(taskPoint[i],_ci)
            # if C_s(taskPoint[i],_ci)==0:
            #     return 0
        return C_s_sum


    Z=[[0 for i in range(len(x_list))]for j in range(len(y_list))]
    for i in range(len(x_list)):
         for j in range(len(y_list)):
             Z[j][i]=f(x_list[i],y_list[j])


    plt.contourf(x_list, y_list, Z,100,cmap='jet',vmin=0, vmax=3)
    # plt.contour(x_list, y_list, Z, [np.exp((-1.0)*RHO)])


    for i in range(len(taskPoint)):
        plt.plot(taskPoint[i][0],taskPoint[i][1], 'o',markersize=5,color="k")
    plt.colorbar()  
    plt.axis([x_min,x_max,y_min,y_max])

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("th="+str(round(th*57.3,3)+90)+" deg")
    plt.gca().set_aspect('equal')
  
    # plt.show()


def plot_contourf_allmax(taskPoint,z=0,x_min=0,x_max=10,y_min=0,y_max=10):
    x_list=np.linspace(x_min,x_max,101)
    y_list=np.linspace(y_min,y_max,101)

    def f(x,y,th):
        C_s_sum=0
        _ci=[x,y,z,th]
        for i in range(len(taskPoint)):
            C_s_sum=C_s_sum+C_s(taskPoint[i],_ci)
            # if C_s(taskPoint[i],_ci)==0:
            #     return 0
        return C_s_sum


    Z=[[0 for i in range(len(x_list))]for j in range(len(y_list))]
    for i in range(len(x_list)):
         for j in range(len(y_list)):
            th_v=[]
            for k in range(0,36):
                th_v.append(f(x_list[i],y_list[j],k*0.1745))
            Z[j][i]=max(th_v)

    plt.contourf(x_list, y_list, Z,100,cmap='jet',vmin=0, vmax=5)
    plt.colorbar()    
    # plt.contour(x_list, y_list, Z, [np.exp((-1.0)*RHO)])

    for i in range(len(taskPoint)):
        plt.plot(taskPoint[i][0],taskPoint[i][1], 'o',markersize=5,color="k")
    plt.axis([x_min,x_max,y_min,y_max])

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.show()
def mut_point(ci,taskPoint):
    all_delta_ci=[0.0,0.0,0.0,0.0]
    for i in range(len(taskPoint)):
        partition=whatPartition(taskPoint[i],ci)
        
        # print(str(i),partition)
        delta=Partial_C_s_Partial_ci(taskPoint[i],ci,partition)

        all_delta_ci[0]=delta[0]+all_delta_ci[0]
        all_delta_ci[1]=delta[1]+all_delta_ci[1]
        all_delta_ci[2]=delta[2]+all_delta_ci[2]
        all_delta_ci[3]=delta[3]+all_delta_ci[3]
        # print(str(i),delta,partition)
    return all_delta_ci

def treedata2taskPoint(td):
    tk=[]
    for i in td:
        tk.append([i[0],i[1],0,0])
    return tk

def line_C_s(tk,p1,p2,ang):
    x_delta=p1[0]-p2[0]
    y_delta=p1[1]-p2[1]
    dis=np.sqrt(x_delta**2+y_delta**2)
    n=max(2,int(dis*5))
    C=0
    for i in range(n):
        x=p1[0]-x_delta/(n)*(i+1)
        y=p1[1]-y_delta/(n)*(i+1)
        C=C+C_s_all(tk,[x,y,0,ang])
    return C/n

class viewPanner:
    def __init__(self):
        self.taskPoint=[]
        self.ci=[1.5,0,0,np.pi*0.5]
        self.ci_last=[1.5,0,0,np.pi*0.5]
        self.it_length=1
        self.g_time=20
        self.occ=[]
        self.myName=""
        self.vt=[0,0,0,0]
        self.bata=0.9
        self.esp=1
    def set_taskPoint(self,_taskPoint):
        self.taskPoint=_taskPoint
    def set_occ(self,_occ):
        self.occ=_occ
    
    def isTooBig(self):
        d_all=0
        for i in range(len(self.taskPoint)):
            d_all=d_all+d_v(self.taskPoint[i],self.ci)
        if d_all>1.2:
            return 1
        return 0

    def F_rep_one(self,pos,occ_pos):
        d=0.0
        d=d+(pos[0]-occ_pos[0])**2
        d=d+(pos[1]-occ_pos[1])**2
        d=d+((pos[2]-occ_pos[2]))**2
        d=np.sqrt(d)
        # print(pos,occ_pos)
        R=1.0
        if d>R:
            # print(d)
            return [0.0,0.0,0.0,0.0]
        gain=0.0*(1.0/d-1/R)/d/d
        ret=[0.0,0.0,0.0,0.0]
        ret[0]=(pos[0]-occ_pos[0])*gain
        ret[1]=(pos[1]-occ_pos[1])*gain
        ret[2]=(pos[2]-occ_pos[2])*gain
        # print(ret[2],pos[2],occ_pos[2])
        return ret

    def F_rep_all(self):
        F=[0.0,0.0,0.0,0.0]
        for i in self.occ:
            ff=self.F_rep_one(self.ci,i)
            F[0]=F[0]+ff[0]
            F[1]=F[1]+ff[1]
            F[2]=F[2]+ff[2]
        # print(self.myName,F)
        return F

    def one_it_momentum(self):
        delta_ci=mut_point(self.ci,self.taskPoint)
        for i in range(4):
            self.vt[i]=self.vt[i]*self.bata+self.it_length*delta_ci[i]
            if self.vt[i]>1:
                self.vt[i]=1
            elif self.vt[i]<-1:
                self.vt[i]=-1
            self.ci[i]=self.ci[i]+self.vt[i]
        
        return self.ci      
    def opt_fun(self,ci):
        return -C_s_all(self.taskPoint,ci)

    def one_it_steepest(self,esp):
        delta_ci=mut_point(self.ci,self.taskPoint)
        a=[0,0,0,0]
        b=[0,0,0,0]
        rate=[5,5,5,0.05]
        for i in range(4):
            a[i]=self.ci[i]#-self.it_length*rate[i]*delta_ci[i]
            b[i]=self.ci[i]+self.it_length*rate[i]*delta_ci[i]
        self.ci=opt_1d.gold_div_search_vet(a,b,self.opt_fun,esp)
        return self.ci  


    def one_it(self):
        delta_ci=mut_point(self.ci,self.taskPoint)
        delta_F=self.F_rep_all()
        # print(delta_ci)
        self.ci[0]=self.ci[0]+self.it_length*(delta_ci[0]+delta_F[0])
        self.ci[1]=self.ci[1]+self.it_length*(delta_ci[1]+delta_F[1])
        self.ci[2]=self.ci[2]+self.it_length*(delta_ci[2]+delta_F[2])
        self.ci[3]=self.ci[3]+self.it_length*delta_ci[3]
        return self.ci
    def gant(self,times=100,dec=0.9):
        self.esp=1
        self.it_length=1
        if self.g_time>0:
            for i in range(times*5):
                self.one_it_steepest(self.esp)
                self.it_length=self.it_length*dec
            self.g_time=self.g_time-1
            # print(self.g_time)
        else:
            for i in range(times):
                self.one_it_steepest(self.esp)
                self.esp=self.esp*0.9
                self.it_length=self.it_length*dec
        
        
        # print(self.myName,self.ci_last[0]-self.ci[0],self.ci_last[1]-self.ci[1],self.ci_last[2]-self.ci[2],self.ci_last[3]-self.ci[3])
        self.ci_last[0]=self.ci[0]
        self.ci_last[1]=self.ci[1]
        self.ci_last[2]=self.ci[2]
        self.ci_last[3]=self.ci[3]
        return self.ci
    # def gant(self,times=100,dec=0.9):
    #     self.it_length=1
    #     if self.g_time>0:
    #         for i in range(times*5):
    #             self.one_it()
    #             self.it_length=self.it_length*dec
    #         self.g_time=self.g_time-1
    #         # print(self.g_time)
    #     else:
    #         for i in range(times):
    #             self.one_it()
    #             self.it_length=self.it_length*dec
        
        
    #     # print(self.myName,self.ci_last[0]-self.ci[0],self.ci_last[1]-self.ci[1],self.ci_last[2]-self.ci[2],self.ci_last[3]-self.ci[3])
    #     self.ci_last[0]=self.ci[0]
    #     self.ci_last[1]=self.ci[1]
    #     self.ci_last[2]=self.ci[2]
    #     self.ci_last[3]=self.ci[3]
    #     return self.ci

    def get_tpk(self):
        tpk_list=[0 for i in range(len(self.taskPoint)*4)]
        for i in range(len(self.taskPoint)):
            cpk=worldFrame2CameraFrame(self.taskPoint[i],self.ci)
            tpk_list[i*4:i*4+4]=ciSpace2tiSpace(cpk)
            
        return tpk_list

if __name__=="__main__":

    tk=treedata2taskPoint(TREEDATA.TREE_DATA)
    for i in range(0,36):
        ii=i*np.pi/18.0
        plot_contourf(tk,z=0,th=ii,x_min=TREEDATA.X_MIN,x_max=TREEDATA.X_MAX,y_min=TREEDATA.Y_MIN,y_max=TREEDATA.Y_MAX)
        # plt.show()
        plt.savefig("20220127_12-11-38-"+str(ii)+".png",dpi=600)
        plt.clf()

    plot_contourf_allmax(tk,z=0,x_min=TREEDATA.X_MIN,x_max=TREEDATA.X_MAX,y_min=TREEDATA.Y_MIN,y_max=TREEDATA.Y_MAX)
    # plt.savefig("20220127_12-11-38-"+".png",dpi=600)
    plt.clf()