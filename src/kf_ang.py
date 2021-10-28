#!/usr/bin/env python
import os
import rospy
import numpy as np
from sensor_msgs.msg import MagneticField,Imu


import kf_lib

def F(x,u):
    dt=0.02
    f=np.eye(6)
    f[4][5]=dt
    return np.dot(f,x)
def F_d(x,u):
    dt=0.02
    f=np.eye(6)
    f[4][5]=dt
    return f

def H1(x):
    # y=np.array([[x[0][0]+x[2][0]*np.cos(x[4][0])],[x[1][0]+x[3][0]*np.sin(x[4][0])]])

    y=np.array([[x[0][0]+(6.25*10**(-5))*np.cos(x[4][0])],[x[1][0]+(6.25*10**(-5))*np.sin(x[4][0])]])
    return y

def H1_d(x):
    # y1=np.array([1,0,np.cos(x[4][0]),0,np.sin(x[4][0])*(-1.0)*x[2][0],0])
    # y2=np.array([0,1,0,np.sin(x[4][0]),np.cos(x[4][0])*x[3][0],0])
    y1=np.array([1,0,np.cos(x[4][0]),0,np.sin(x[4][0])*(-1.0)*(6.25*10**(-5)),0])
    y2=np.array([0,1,0,np.sin(x[4][0]),np.cos(x[4][0])*(6.25*10**(-5)),0])
    return np.array([y1,y2])

def H2(x):
    return x[5][0]

def H2_d(x):
    return np.array([[0,0,0,0,0,1]])


def cb_mag(data):
    global imu_mag
    z_x=data.magnetic_field.x
    z_y=data.magnetic_field.y
    imu_mag.update(np.array([[z_x],[z_y]]))

def cb_imu(data):
    global imu_omg
    z=data.angular_velocity.z*3.0
    imu_omg.update(np.array([[z]]))

mag_ekf=kf_lib.ExtendedKalmanFilter(6)
mag_ekf.F=F
mag_ekf.F_d=F_d

# Initial value
# x[0] cx
# x[1] cy
# x[2] rx
# x[3] ry
# x[4] theta
# x[5] omega
mag_ekf.X[0]=-5.75*10**(-5)
mag_ekf.X[1]=-8.75*10**(-5)
mag_ekf.X[2]=6.25*10**(-5)
mag_ekf.X[3]=6.25*10**(-5)
mag_ekf.X[4]=0
mag_ekf.X[5]=0.0
mag_ekf.Q=np.eye(6)
mag_ekf.Q[0][0]=10**(-7)
mag_ekf.Q[1][1]=10**(-7)
mag_ekf.Q[2][2]=10**(-10)
mag_ekf.Q[3][3]=10**(-10)
mag_ekf.Q[4][4]=10**(-8)
mag_ekf.Q[5][5]=10**(-5)




imu_mag=kf_lib.EKF_updater(2,mag_ekf)
imu_mag.H=H1
imu_mag.H_d=H1_d



imu_omg=kf_lib.EKF_updater(1,mag_ekf)
imu_omg.H=H2
imu_omg.H_d=H2_d
imu_omg.R[0][0]=10**(-2)
rospy.init_node('kf', anonymous=True)

mag_sub = rospy.Subscriber('/imu/mag', MagneticField, cb_mag)
imu_sub = rospy.Subscriber('/imu/data', Imu, cb_imu)
mag_org_pub = rospy.Publisher('/imu/mag_org', MagneticField, queue_size=1)
mag_rad_pub = rospy.Publisher('/imu/mag_rad', MagneticField, queue_size=1)
mag_ang_pub = rospy.Publisher('/imu/mag_ang', Imu, queue_size=1)
rate = rospy.Rate(50.0)



while not rospy.is_shutdown():
    #t=time.time()

    mag_ekf.prediction([])



    mag_org_msg=MagneticField()
    mag_org_msg.magnetic_field.x=mag_ekf.X[0]
    mag_org_msg.magnetic_field.y=mag_ekf.X[1]
    mag_org_pub.publish(mag_org_msg)

    mag_rad_msg=MagneticField()
    mag_rad_msg.magnetic_field.x=mag_ekf.X[2]
    mag_rad_msg.magnetic_field.y=mag_ekf.X[3]
    mag_rad_pub.publish(mag_rad_msg)

    mag_ang_msg=Imu()
    mag_ang_msg.angular_velocity.z=mag_ekf.X[5]
    mag_ang_msg.orientation.z=np.sin(mag_ekf.X[4]*0.5)
    mag_ang_msg.orientation.w=np.cos(mag_ekf.X[4]*0.5)
    mag_ang_pub.publish(mag_ang_msg)

    
    rate.sleep()