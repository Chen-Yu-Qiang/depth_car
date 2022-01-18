#!/usr/bin/wowpython
import time

from datetime import datetime
import numpy as np
import rospy
import sys
import time
import os
import cv2
import glob

if __name__=='__main__':

    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/1k"


    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    w = 10
    h = 6
    objp = np.zeros((w*h,3), np.float32)
    objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
    objp = objp*20.0  # 18.1 mm


    objpoints = []
    imgpoints = []


    i=0
    for fname in range(50):

        img = cv2.imread(file_path+'/image'+str(fname)+'.jpg')

        h1, w1 = img.shape[0], img.shape[1]
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        u, v = img.shape[:2]

        ret, corners = cv2.findChessboardCorners(gray, (w,h),None)

        if ret == True:
            print("i:", i)
            i = i+1

            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            objpoints.append(objp)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(img, (w,h), corners, ret)
            image2 = cv2.resize(img, (960,540), interpolation=cv2.INTER_AREA)
            cv2.imshow("findCorners",image2)

            cv2.waitKey(200)
    cv2.destroyAllWindows()
    print('正在计算')

    ret, mtx, dist, rvecs, tvecs = \
        cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


    print("ret:",ret  )
    print("mtx:\n",mtx)
    print("dis:\n",dist   )
    print("rvecs:\n",rvecs)
    print("tvecs:\n",tvecs  )
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0, (u, v))
    print('newcameramtx',newcameramtx)

    cv2.destroyAllWindows()


