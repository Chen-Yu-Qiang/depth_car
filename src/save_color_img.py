#!/usr/bin/env python

import numpy as np
import cv2



if __name__=="__main__":
    for AA in range(1116):
        file_path = "/home/yuqiang/catkin_car/src/depth_car/src/syn_rosbag3/"
        npColor = np.load(file_path+"color/"+str(AA)+".npy")
        npColor=cv2.cvtColor(npColor,cv2.COLOR_BGR2RGB)
        npColor=cv2.putText(npColor,"Frame= "+str(AA),(100,100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        cv2.imshow("color",npColor)
        cv2.imwrite(file_path+"color/img/"+str(AA)+".png", npColor)
