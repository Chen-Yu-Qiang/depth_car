#!/usr/bin/env python
import os
import rospy
import tf
import numpy as np
from sensor_msgs.msg import MagneticField,Imu


import kf_lib

def F(x,u):
    dt=0.02
    f=np.eye(4)
    f[2][3]=dt
    return np.dot(f,x)
def F_d(x,u):
    dt=0.02
    f=np.eye(4)
    f[2][3]=dt
    return f

def H1(x):
    # y=np.array([[x[0][0]+x[2][0]*np.cos(x[4][0])],[x[1][0]+x[3][0]*np.sin(x[4][0])]])

    y=np.array([[x[0][0]+(6.25*10**(-5))*np.cos(x[2][0])*(-1.0)],[x[1][0]+(6.25*10**(-5))*np.sin(x[2][0])]])
    return y

def H1_d(x):
    # y1=np.array([1,0,np.cos(x[4][0]),0,np.sin(x[4][0])*(-1.0)*x[2][0],0])
    # y2=np.array([0,1,0,np.sin(x[4][0]),np.cos(x[4][0])*x[3][0],0])
    y1=np.array([1,0,np.sin(x[2][0])*(6.25*10**(-5)),0])
    y2=np.array([0,1,np.cos(x[2][0])*(6.25*10**(-5)),0])
    return np.array([y1,y2])

def H2(x):
    return x[3][0]

def H2_d(x):
    return np.array([[0,0,0,1]])

def H3(x):
    return x[2][0]

def H3_d(x):
    return np.array([[0,0,1,0]])

def H4(x):
    return np.array([[x[0][0]],[x[1][0]]])

def H4_d(x):
    y1=np.array([1,0,0,0])
    y2=np.array([0,1,0,0])
    return np.array([y1,y2])


def cb_mag(data):
    global imu_mag
    z_x=data.magnetic_field.x
    z_y=data.magnetic_field.y
    imu_mag.update(np.array([[z_x],[z_y]]))

def cb_imu(data):
    global imu_omg
    z=data.angular_velocity.z*10.0
    imu_omg.update(np.array([[z]]))
n2pi=0
def cb_gps(data):
    global gps_ang,n2pi
    _,_,z=tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    gps_ang.update(np.array([[z+n2pi*2.0*np.pi]]))
    print(z+n2pi*2.0*np.pi)

def cb_ls(data):
    global ls_org
    z_x=data.magnetic_field.x
    z_y=data.magnetic_field.y
    ls_org.update(np.array([[z_x],[z_y]]))
    print(np.array([[z_x],[z_y]]))

mag_ekf=kf_lib.ExtendedKalmanFilter(4)
mag_ekf.F=F
mag_ekf.F_d=F_d

# Initial value
# x[0] cx
# x[1] cy
# x[2] theta
# x[3] omega

mag_ekf.X[0]=-5.75*10**(-5)
mag_ekf.X[1]=-8.75*10**(-5)
mag_ekf.X[2]=0.0
mag_ekf.X[3]=0.0
mag_ekf.Q=np.eye(4)
mag_ekf.Q[0][0]=10**(-7)
mag_ekf.Q[1][1]=10**(-7)
mag_ekf.Q[2][2]=10**(-6)
mag_ekf.Q[3][3]=10**(-2)

mag_ekf.P[0][0]=10**(-2)
mag_ekf.P[1][1]=10**(-2)
mag_ekf.P[2][2]=10**(-3)




imu_mag=kf_lib.EKF_updater(2,mag_ekf)
imu_mag.H=H1
imu_mag.H_d=H1_d



imu_omg=kf_lib.EKF_updater(1,mag_ekf)
imu_omg.H=H2
imu_omg.H_d=H2_d
imu_omg.R[0][0]=10**(-2)

gps_ang=kf_lib.EKF_updater(1,mag_ekf)
gps_ang.H=H3
gps_ang.H_d=H3_d
imu_omg.R[0][0]=10**(-2)

ls_org=kf_lib.EKF_updater(2,mag_ekf)
ls_org.H=H4
ls_org.H_d=H4_d
ls_org.R[0][0]=10**(-2)
ls_org.R[1][1]=10**(-2)

rospy.init_node('kf', anonymous=True)

mag_sub = rospy.Subscriber('/imu/mag', MagneticField, cb_mag)
imu_sub = rospy.Subscriber('/imu/data', Imu, cb_imu)
gps_sub = rospy.Subscriber('/gps_ang', Imu, cb_gps)
ls_sub = rospy.Subscriber('/LS/mag_org', MagneticField, cb_ls)
mag_org_pub = rospy.Publisher('/EKF/mag_org', MagneticField, queue_size=1)
mag_ang_pub = rospy.Publisher('/EKF/mag_ang', Imu, queue_size=1)
rate = rospy.Rate(50.0)



while not rospy.is_shutdown():
    #t=time.time()

    mag_ekf.prediction([])



    mag_org_msg=MagneticField()
    mag_org_msg.magnetic_field.x=mag_ekf.X[0]
    mag_org_msg.magnetic_field.y=mag_ekf.X[1]
    mag_org_pub.publish(mag_org_msg)



    mag_ang_msg=Imu()
    mag_ang_msg.angular_velocity.z=mag_ekf.X[3]
    ang_0_2pi=mag_ekf.X[2] % (2.0*np.pi)
    n2pi=int((mag_ekf.X[2]+np.pi)/(2.0*np.pi))
    ang_q=tf.transformations.quaternion_from_euler(0,0,ang_0_2pi)
    mag_ang_msg.orientation.z=ang_q[2]
    mag_ang_msg.orientation.w=ang_q[3]
    mag_ang_pub.publish(mag_ang_msg)

    
    rate.sleep()