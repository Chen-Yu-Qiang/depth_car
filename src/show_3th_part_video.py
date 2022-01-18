#!/usr/bin/wowpython
import time

from datetime import datetime
import numpy as np

import rospy
import sys
import time
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import MagneticField,NavSatFix
import os
import cv2

ck=0.0
ck_system=time.time()
is_paused=1
bag_rate=None
def ck_cb(data):
    global ck,is_paused,ck_system,bag_rate

    if ck==data.clock.secs+data.clock.nsecs*(10**-9):
        is_paused=1
    else:
        ck_old=ck
        ck=data.clock.secs+data.clock.nsecs*(10**-9)
        bag_rate=(ck-ck_old)/(time.time()-ck_system)

        ck_system=time.time()

        is_paused=0
if __name__=='__main__':
    
    
    # file_path=raw_input("Please enter the video path and filename : ")
    file_path="/media/yuqiang/2CFC30D1FC309754/BAG/220113_Dahu/video/2022-01-13-16-45-54.mp4"
    while not os.path.isfile(file_path):
        print("File does not exist, please check again")
        file_path=raw_input("Please enter the video path and filename : ")
    # start_time=float(raw_input("Please enter the video start unix time : "))
    start_time=1642062427.062

    cap = cv2.VideoCapture(file_path)
    fps=cap.get(cv2.CAP_PROP_FPS)

    print("FPS = ",fps)

    rospy.init_node("show_video", anonymous=True)
    rospy.Subscriber("clock",Clock,ck_cb,queue_size=1)
    no_use=rospy.wait_for_message("clock",Clock)
    time.sleep(5)
    rate=rospy.Rate(fps*bag_rate)

    print("bag_rate = ",bag_rate)
    print("wait for clock")
    no_use=rospy.wait_for_message("clock",Clock)
    while (ck-start_time)<0:
        print("Wait for start time ",start_time," Now is ",ck)
        time.sleep(1)
    cap.set(cv2.CAP_PROP_POS_MSEC,(ck-start_time)*1000.0)

    while not rospy.is_shutdown():
        if not is_paused:
            success, image = cap.read()


            if success:
                image = cv2.resize(image, (960,540), interpolation=cv2.INTER_AREA)
                image = cv2.rotate(image, cv2.ROTATE_180)
                t_ros=datetime.fromtimestamp(ck)
                cv2.putText(image, t_ros.strftime("%Y %m %d %H:%M:%S.%f")[:-3], (0, 500), cv2.FONT_HERSHEY_SIMPLEX,0.8, (255, 255, 255), 1, cv2.LINE_AA)

                cv2.imshow("a",image)
                cv2.waitKey(1)

        rate.sleep()