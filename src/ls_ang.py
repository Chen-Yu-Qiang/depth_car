#!/usr/bin/env python
import os
import rospy
import tf
import numpy as np
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64

mag_x=0
mag_y=0
centre_x=-5.75*10**(-5)
centre_y=-8.75*10**(-5)
def cb_mag(data):
    global mag_x,mag_y,centre_x,centre_y
    mag_x=data.magnetic_field.x
    mag_y=data.magnetic_field.y

    ang=np.arctan2(-(mag_y-centre_y),(mag_x-centre_x))
    mag_ang_msg=Imu()
    ang_0_2pi= ang % (2.0*np.pi)
    ang_q=tf.transformations.quaternion_from_euler(0,0,ang_0_2pi)
    mag_ang_msg.orientation.z=ang_q[2]
    mag_ang_msg.orientation.w=ang_q[3]
    mag_ang_pub.publish(mag_ang_msg)

mag_x_list=np.zeros((36,))
mag_y_list=np.zeros((36,))
for i in range(36):
    mag_x_list[i]=centre_x-np.cos(i*0.1745)*(6.25*10**(-5))
    mag_y_list[i]=centre_y+np.sin(i*0.1745)*(6.25*10**(-5))


def cb_imu(data):
    global mag_x_list,mag_y_list,mag_x,mag_y
    ang=tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    p=int((ang[2]% (2.0*np.pi))/0.1745)
    mag_x_list[p-1]=mag_x
    mag_y_list[p-1]=mag_y



rospy.init_node('get_mag_org', anonymous=True)

mag_sub = rospy.Subscriber('/imu/mag', MagneticField, cb_mag)
imu_sub = rospy.Subscriber('/imu/data', Imu, cb_imu)
mag_org_pub = rospy.Publisher('/imu2/mag_org', MagneticField, queue_size=1)
mag_residuals_pub = rospy.Publisher('/imu2/residuals', Float64, queue_size=1)
mag_ang_pub = rospy.Publisher('/imu2/mag_ang', Imu, queue_size=1)
rate = rospy.Rate(50.0)


k_old=None
while not rospy.is_shutdown():
    #t=time.time()
    A_3=-1.0/(mag_x_list*mag_x_list+mag_y_list*mag_y_list)
    A_1=mag_x_list*A_3
    A_2=mag_y_list*A_3
    A=np.array([A_1,A_2,A_3]).T
    # try:
    #     k = np.linalg.inv(np.dot(A.T,A))
    # except:
    #     continue
    # k = np.dot(k,A.T)
    # k = np.dot(k,np.ones((k.shape[1],1)))
    y=np.ones((36,))
    k,residuals,_,_=np.linalg.lstsq(A,y,rcond=None)


    if residuals[0]<0.5 or (k_old is None):
        k_old=k
    else:
        print("Too big error")
        k=k_old

    centre_x = k[0]/(-2)
    centre_y = k[1]/(-2)
    radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2])

    mag_residuals_msg=Float64()
    mag_residuals_msg.data=residuals[0]
    mag_residuals_pub.publish(mag_residuals_msg)

    mag_org_msg=MagneticField()
    mag_org_msg.magnetic_field.x=centre_x
    mag_org_msg.magnetic_field.y=centre_y
    mag_org_pub.publish(mag_org_msg)


    print(centre_x,centre_y,radius_r,residuals[0])
    rate.sleep()