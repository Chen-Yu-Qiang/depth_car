#!/usr/bin/wowpython
import time

from datetime import datetime
import numpy as np
import rospy
import sys
import time
import os
import cv2


if __name__=='__main__':
    i=0
    
    # file_path=raw_input("Please enter the video path and filename : ")
    file_path="/media/yuqiang/2CFC30D1FC309754/research data/1110121/aruco/PXL_20220118_112017729.mp4"
    while not os.path.isfile(file_path):
        print("File does not exist, please check again")
    start_time=1642497766.821
    cap = cv2.VideoCapture(file_path)
    fps=cap.get(cv2.CAP_PROP_FPS)

    print("FPS = ",fps)
    while 1:
        success, image = cap.read()


        if success:
            # image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            image2 = cv2.resize(image, (960,540), interpolation=cv2.INTER_AREA)
            cv2.imshow("a",image2)
            e=cv2.waitKey(1)
            if e==32:

                cv2.imwrite('image'+str(i)+'.jpg', image)
                i=i+1
                print("save a picture")
        else:
            break
        time.sleep(1/fps)