#!/usr/bin/wowpython
import csv
import time

from datetime import datetime
from std_msgs.msg import Time
import numpy as np
import rosbag
import tf
import sys
import time 
import os
import cv2
from cv2 import aruco

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist

def mat_4_4_to_msg(mat):
    q_zyx=tf.transformations.euler_from_matrix(mat[0:3,0:3], 'sxyz')

    msg=Twist()
    msg.linear.x=mat[0,3]
    msg.linear.y=mat[1,3]
    msg.linear.z=mat[2,3]
    msg.angular.x=q_zyx[0]
    msg.angular.y=q_zyx[1]
    msg.angular.z=q_zyx[2]
    return msg

def time_to_msg(f):
    msg=Time()
    msg.data.secs=int(f)
    msg.data.nsecs=(f-int(f))*(10**9)
    return msg

def fun(file_path):
    i=0
    
    # file_path=raw_input("Please enter the video path and filename : ")
    while not os.path.isfile(file_path):
        print("File does not exist, please check again")
    start_time=1642497766.821
    cap = cv2.VideoCapture(file_path)
    fps=cap.get(cv2.CAP_PROP_FPS)
    cap.set(cv2.CAP_PROP_POS_MSEC,0.0)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(file_path[:-4]+'output.avi', fourcc, 60.0, (960,540))
    print("FPS = ",fps)
    rvec_ground, tvec_ground = None,None
    rvec_car, tvec_car = None,None
    t=start_time
    t_list=[]
    T_ground_list=[]
    T_ground_car_list=[]

    while 1:
        success, image = cap.read()

        image = cv2.rotate(image, cv2.ROTATE_180)
        if success:
            t=t+1.0/fps
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict)
            aruco.drawDetectedMarkers(image, corners,ids)

            

            ##### for 1080p
            # camera_matrix = np.array([[1.67682097e+03, 0.00000000e+00, 9.74170554e+02],[0.00000000e+00, 1.68228753e+03, 5.54943156e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
            
            
            ##### for 4k  
            camera_matrix = np.array([[3.25454682e+03, 0.00000000e+00, 1.90003022e+03],[0.00000000e+00, 3.26454411e+03, 1.09048736e+03],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
            
            
            dist_coeffs = np.array([[ 2.44394963e-01, -1.62419397e+00,  1.88079631e-03,-6.46917823e-03,4.07465043e+00]])
            board_corners = [np.array([[0,0.18,0],[0.18,0.18,0],[0.18,0,0],[0,0,0]],dtype=np.float32) ,
            np.array([[0,0.99,0],[0.18,0.99,0],[0.18,0.81,0],[0,0.81,0]],dtype=np.float32),
            np.array([[0.75,0.18,0],[0.93,0.18,0],[0.93,0,0],[0.75,0,0]],dtype=np.float32)]
            aruco_marker_length_meters=0.5
            board_ids = np.array([[0],[1],[2]], dtype=np.int32)
            board = aruco.Board_create(board_corners,
                                    aruco.getPredefinedDictionary(
                                    aruco.DICT_5X5_100),
                                    board_ids)

            retval, rvec_ground, tvec_ground = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec_ground, tvec_ground)
            if rvec_ground is None or tvec_ground is None:
                pass
            else:
                image = aruco.drawAxis( image, camera_matrix, dist_coeffs, rvec_ground, tvec_ground, aruco_marker_length_meters )
                image = aruco.drawDetectedMarkers( image, corners, ids )
                rr, _ = cv2.Rodrigues(np.array([rvec_ground[0][0], rvec_ground[1][0], rvec_ground[2][0]]))
                tt = np.array([tvec_ground[0][0], tvec_ground[1][0], tvec_ground[2][0]])
                rr_ground = rr.transpose()
                tt_ground = -rr_ground.dot(tt)
                T_ground=np.zeros((4,4))
                T_ground[0:3,0:3]=rr_ground
                T_ground[0:3,3]=tt_ground
                T_ground[3,3]=1

            board_offset=np.array([-0.1,-0.05,0],dtype=np.float32)
            board_corners = [np.array([[0,0.18,0],[0.18,0.18,0],[0.18,0,0],[0,0,0]],dtype=np.float32)+board_offset]



            # board_offset=np.array([-0.08,0.37,0],dtype=np.float32)
            # board_corners = [np.array([[0,0,0.18],[0.18,0,0.18],[0.18,0,0],[0,0,0]],dtype=np.float32)+board_offset]
            aruco_marker_length_meters=0.5
            board_ids = np.array([[51]], dtype=np.int32)
            board = aruco.Board_create(board_corners,
                                    aruco.getPredefinedDictionary(
                                    aruco.DICT_5X5_100),
                                    board_ids)

            retval, rvec_car, tvec_car = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec_car, tvec_car)
            if (not retval==1) or rvec_car is None or tvec_car is None:
                pass
            else:
                image = aruco.drawAxis( image, camera_matrix, dist_coeffs, rvec_car, tvec_car, aruco_marker_length_meters )
                image = aruco.drawDetectedMarkers( image, corners, ids )
                rr_car, _ = cv2.Rodrigues(np.array([rvec_car[0][0], rvec_car[1][0], rvec_car[2][0]]))
                tt_car = np.array([tvec_car[0][0], tvec_car[1][0], tvec_car[2][0]])
                T_car=np.zeros((4,4))
                T_car[0:3,0:3]=rr_car
                T_car[0:3,3]=tt_car
                T_car[3,3]=1

                T_ground_car=np.dot(T_ground,T_car)

                t_list.append(time_to_msg(t))
                T_ground_list.append(mat_4_4_to_msg(T_ground))
                T_ground_car_list.append(mat_4_4_to_msg(T_ground_car))
            image2 = cv2.resize(image, (960,540), interpolation=cv2.INTER_AREA)

            out.write(image2)
            cv2.imshow("b",image2)
            cv2.waitKey(1)
        else:
            break
        # time.sleep(1/fps)
    
    
    out.release()

    bag = rosbag.Bag(file_path[:-4]+'.bag', 'w')
    for i in range(len(T_ground_car_list)):
        bag.write('T_ground', T_ground_list[i], t=t_list[i].data)
        bag.write('T_ground_car', T_ground_car_list[i], t=t_list[i].data)
    bag.close()



if __name__=='__main__':
    
    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/PXL_20220118_092246821.mp4"
    fun(file_path)
    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/PXL_20220118_092527147.mp4"
    fun(file_path)
    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/PXL_20220118_092811891.mp4"
    fun(file_path)
    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/PXL_20220118_092916642.mp4"
    fun(file_path)
    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/PXL_20220118_093008492.mp4"
    fun(file_path)
    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/PXL_20220118_093111377.mp4"
    fun(file_path)