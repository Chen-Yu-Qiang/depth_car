#!/usr/bin/env python

from logging import info

from scipy.sparse import csr_matrix,coo_matrix,dia_matrix
import sys

import numpy as np
import matplotlib.pyplot as plt

import time
import cv2
import csv




MAP_X_SIZE=5000  # pixel
MAP_Y_SIZE=5000  # pixel
LOCAL_MAP_X_SIZE=500 # pixel
LOCAL_MAP_Y_SIZE=500 # pixel
MAP_RESOLUTION=20  # mm pre pixel
MAP_X_CENTER=2500 # pixel
MAP_Y_CENTER=2500 # pixel

HEIGHT_H=1000 # mm
HEIGHT_L=500 # mm
DEPTH_H=8000 # mm
DEPTH_L=500 # mm

def depth_to_3DPoint(npDepth):

    npDepth=cv2.GaussianBlur(npDepth,(11,11),0)
    npDepth = npDepth.astype('float64')

    # npDepth[npDepth>10000]=np.nan
    cx_d = 320.6562194824219 #424
    cy_d = 241.57083129882812 #241
    fx_d = 384.31365966796875 #424
    fy_d = 384.31365966796875 #424
    npPointX = np.asarray(range(640))-cx_d
    npPointX = np.diag(npPointX)
    npPointX = dia_matrix(npPointX)
    npPointX = npDepth*npPointX
    # npPointX = npDepth.dot(npPointX)/ fx_d * (-1)
    npPointX = npPointX/ fx_d * (-1)
    npPointX = npPointX.astype('float64')

    npPointY = np.asarray(range(480))-cy_d
    npPointY = np.diag(npPointY)
    theta = 0/180*np.pi
    npPointY = dia_matrix(npPointY)
    npPointY = npPointY * npDepth
    npPointY = npPointY/ fy_d * (-1) 
    # npPointY = npPointY.dot(npDepth)/ fy_d * (-1) 
    npPointY = npPointY*np.cos(theta) + npDepth * np.sin(theta) + 410
    npPointY = npPointY.astype('float64')
    npHeight = np.copy(npPointY)
    # npHeight[npHeight<=300]=300
    # npHeight[npHeight>=2000]=2000

    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npPointX,100)
    # plt.colorbar()
    # plt.show(block=False)
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npHeight,100)
    # plt.colorbar()
    # plt.show(block=False)
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npDepth,100)
    # plt.colorbar()
    # plt.show()
    return npPointX,npHeight,npDepth

def get_tree_mask(npPointX,npHeight,npDepth):


    down_component=np.zeros((480,640))
    kernel=np.array([[-0.2,-0.2,-0.2,-0.2,-0.2],[0,0,0,0,0],[0.2,0.2,0.2,0.2,0.2]])
    npPointX_d=cv2.filter2D(npPointX,-1,kernel=kernel)
    npHeight_d=cv2.filter2D(npHeight,-1,kernel=kernel)
    npDepth_d=cv2.filter2D(npDepth,-1,kernel=kernel)
    v1_norm=np.sqrt(npPointX_d*npPointX_d+npHeight_d*npHeight_d+npDepth_d*npDepth_d)
    down_component=npHeight_d/v1_norm
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npPointX,100)
    # plt.colorbar()
    # plt.savefig("x.svg")
    # plt.show(block=False)
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npHeight,100)
    # plt.colorbar()
    # plt.savefig("y.svg")
    # plt.show(block=False)
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npDepth,100)
    # plt.colorbar()
    # plt.savefig("z.svg")
    # plt.show(block=False)


    down_component[npDepth>DEPTH_H]=0
    down_component[npDepth<DEPTH_L]=0
    down_component[npHeight>HEIGHT_H]=0
    down_component[npHeight<HEIGHT_L]=0

    tree_mask=down_component<-0.6
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npPointX_d/v1_norm,100)
    # plt.colorbar()
    # plt.savefig("dx.svg")
    # plt.show(block=False)
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npHeight_d/v1_norm,100)
    # plt.colorbar()
    # plt.savefig("dy.svg")
    # plt.show(block=False)
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),npDepth_d/v1_norm,100)
    # plt.colorbar()
    # plt.savefig("dz.svg")
    # plt.show(block=False)
    # plt.figure()
    # plt.contourf(range(640),479-np.asarray(range(480)),tree_mask,100)
    # plt.colorbar()
    # plt.savefig("tm.svg")
    # plt.show()
    tree_mask=tree_mask.astype('uint16')

    kernel = np.ones((5,5), np.uint16)
    tree_mask = cv2.erode(tree_mask, kernel, iterations = 1)
    kernel = np.ones((20,5), np.uint16)
    tree_mask = cv2.dilate(tree_mask,kernel,iterations = 3)

    return tree_mask

def img_mask(depthimg,mask):
    y=np.copy(depthimg)
    y[mask==0]=8000
    return y


def depth_filter(npPointX,npHeight,npDepth,mask=np.ones((480,640)),height_range_low=500,height_range_high=900):
    
    

    t1=time.time()
    x_depth=np.reshape(npPointX,(307200,))
    y_depth=np.reshape(npHeight,(307200,))
    z_depth=np.reshape(npDepth,(307200,))
    mask2=mask.copy()
    mask2=np.reshape(mask2,(307200,))

    mask2[z_depth>=LOCAL_MAP_X_SIZE*MAP_RESOLUTION]=0
    mask2[x_depth>=(LOCAL_MAP_X_SIZE*MAP_RESOLUTION)*0.5]=0
    mask2[x_depth<=(LOCAL_MAP_X_SIZE*MAP_RESOLUTION)*(-0.5)]=0
    mask2[y_depth>height_range_high]=0
    mask2[y_depth<height_range_low]=0
    z_depth=z_depth[~(mask2==0)]
    x_depth=x_depth[~(mask2==0)]    
    return z_depth, x_depth

def depth_to_topView(z_depth, x_depth):
    mygrid=np.zeros((LOCAL_MAP_X_SIZE,LOCAL_MAP_Y_SIZE))
    for i in range(len(z_depth)):
        mygrid[int(z_depth[i]/MAP_RESOLUTION),int(x_depth[i]/MAP_RESOLUTION)+250]+=1
    
    mygrid=mygrid>3

    return mygrid.astype('uint8')

def into_world_map(map,z_depth, x_depth,x,y,theta):
    """
        x(mm)
        y(mm)
        theta(rad)
    """
    M = np.asarray([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    xy=np.array([x_depth,z_depth])
    # xy=np.array([[1000,1000,1000,1000,1000,1000,2000,2000,2000,2000,2000,2000,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

    xy=np.dot(M,xy)
    xy=xy+np.array([[x],[y]])
    xy=xy/MAP_RESOLUTION+np.array([[MAP_X_CENTER],[MAP_Y_CENTER]])
    x_list=xy[0,:]
    y_list=xy[1,:]
    mask=np.ones((len(xy[1,:]),))
    mask[x_list>MAP_X_SIZE]=0
    mask[x_list<0]=0
    mask[y_list>MAP_Y_SIZE]=0
    mask[y_list<0]=0
    x_list=x_list[~(mask==0)]
    y_list=y_list[~(mask==0)]
    x_list=x_list.astype("uint16")
    y_list=y_list.astype("uint16")
    map_temp=np.zeros((MAP_X_SIZE,MAP_X_SIZE))
    for i in range(len(x_list)):
        map_temp[x_list[i],y_list[i]]+=1
    map_temp=map_temp>3
    map=map+map_temp
    return map.astype('uint8')

def circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,x,y,theta):

    M = np.asarray([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    xz=np.array([centre_x_list,centre_z_list])

    # xz=np.array([[1000,2000,3000],[0,0,0]])
    # radius_r_list=[1000,1000,1000]
    xz=np.dot(M,xz)
    xz=xz+np.array([[x],[y]])
    xz=xz/MAP_RESOLUTION+np.array([[MAP_X_CENTER],[MAP_Y_CENTER]])
    x_list=xz[0,:]
    z_list=xz[1,:]
    r_list=np.array(radius_r_list)/MAP_RESOLUTION
    mask=np.ones((len(xz[1,:]),))
    mask[x_list>MAP_X_SIZE]=0
    mask[x_list<0]=0
    mask[z_list>MAP_Y_SIZE]=0
    mask[z_list<0]=0
    x_list=x_list[~(mask==0)]
    z_list=z_list[~(mask==0)]
    r_list=r_list[~(mask==0)]
    x_list=x_list.astype("uint16")
    z_list=z_list.astype("uint16")
    r_list=r_list.astype("uint16")
    for i in range(len(x_list)):
        # if r_list[i]<10:
        #     r=10
        # else:
        #     r=r_list[i]
         map=cv2.circle(map,(z_list[i], x_list[i]), r_list[i], 50, 2)
        #  map=cv2.circle(map,(z_list[i], x_list[i]), 1, 50, 2)
    return map

def draw_arrowed(x,y,theta,img,color=(0,255,0)):
    """
        x(mm)
        y(mm)
        theta(rad)
    """
    arrowed_start=np.array([y,x])
    arrowed_end=arrowed_start+np.array([np.sin(theta)*1000,np.cos(theta)*1000])
    arrowed_start=(arrowed_start/MAP_RESOLUTION+2500).astype("uint16")
    arrowed_end=(arrowed_end/MAP_RESOLUTION+2500).astype("uint16")
    arrowed_start=tuple(arrowed_start)
    arrowed_end=tuple(arrowed_end)
    img=cv2.arrowedLine(img, arrowed_start, arrowed_end, color,5,8,0,0.3)
    return img

'''
circle c
c["x"] x  in camera frame
c["y"] y  in camera frame
c["r"] Radius
c["res"] Residual

'''

def topView_to_circle(tv):
    t1=time.time()
    hieght_or = np.zeros((LOCAL_MAP_X_SIZE,LOCAL_MAP_Y_SIZE), dtype=np.uint8)
    hieght_or[tv>0]=255
    hieght_or = hieght_or.astype('uint8')
    kernel = np.ones((1,1), np.uint8)
    # hieght_or = cv2.erode(hieght_or, kernel, iterations = 1)
    kernel = np.ones((3,3), np.uint8)
    # hieght_or = cv2.dilate(hieght_or,kernel,iterations = 1)
    cv2.imshow("hieght_or",hieght_or)
    t2=time.time()
    ''' find connected component and push into point array A '''
    num_objects, labels = cv2.connectedComponents(hieght_or,connectivity=4)

    # label_mask=labels>0
    # label_mask=label_mask.astype('uint8')
    # kernel = np.ones((3,3), np.uint8)
    # label_mask = cv2.erode(label_mask, kernel, iterations = 1)
    
    # labels[label_mask==0]=0


    labels_spr=coo_matrix(labels)
    labels_spr=np.asarray([labels_spr.row,labels_spr.col,labels_spr.data])
    labels_spr=labels_spr.T
    labels_spr=labels_spr[labels_spr[:,2].argsort()]
    centre_x_list = []
    centre_y_list = []
    radius_r_list = []
    circle_bd = np.zeros(hieght_or.shape, dtype=np.uint8)
    # print('>>>>num_objects:',num_objects)
    t3=time.time()
    i_index_start=0
    i_index_end=0
    A_3=-1.0/(labels_spr[:,0]*labels_spr[:,0]+labels_spr[:,1]*labels_spr[:,1])
    A_1=labels_spr[:,0]*A_3
    A_2=labels_spr[:,1]*A_3
    A_all=np.array([A_1,A_2,A_3]).T

    i_max=len(A_1)

    tv=cv2.cvtColor(tv*30, cv2.COLOR_GRAY2BGR)
    for i in range(1,num_objects):
        A = []
        t31=time.time()
        while (labels_spr[i_index_end,2]==i):
            if i_index_end<i_max-1:
                i_index_end=i_index_end+1
            else:
                break
        
        # print(i_index_end-i_index_start)
        if i_index_end-i_index_start<20:
            i_index_start=i_index_end
            continue
        A=A_all[i_index_start:i_index_end,:]
        # A_all=A_all[i_index_end:,:]
        i_index_start=i_index_end


        # for x in range(LOCAL_MAP_X_SIZE):
        #     for y in range(LOCAL_MAP_Y_SIZE):
        #         if labels[x][y] == i+1:
        #             A.append(np.array([-1.0*x/(x*x+y*y), -1.0*y/(x*x+y*y), -1.0/(x*x+y*y)]))
        # A = np.asarray(A)
        # print('# of points: ',A.shape)

        t32=time.time()
        try:
            k = np.linalg.inv(np.dot(A.T,A))
        except:
            continue
        k = np.dot(k,A.T)
        k = np.dot(k,np.ones((k.shape[1],1)))

        t33=time.time()
        centre_x = k[0][0]/(-2)
        centre_y = k[1][0]/(-2)
        radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2][0])

        # Convert pixel to mm
        centre_x_mm=centre_x*MAP_RESOLUTION
        centre_y_mm=(centre_y-LOCAL_MAP_Y_SIZE*0.5)*MAP_RESOLUTION
        radius_r_mm=radius_r*MAP_RESOLUTION 


        # print('x_mm,y_mm,r_mm: ', centre_x_mm, centre_y_mm, radius_r_mm)
        # print('x_pix,y_pix,r_pix: ', centre_x, centre_y, radius_r)
        # 
        
        tv=cv2.circle(tv,(int(centre_y+0.5), int(centre_x+0.5)), int(radius_r+0.5), (100,100,0), 2)
        
        centre_x_list.append(centre_x_mm)
        centre_y_list.append(centre_y_mm)
        radius_r_list.append(radius_r_mm)
        t34=time.time()
        # print(t34-t31,t32-t31,t33-t32,t34-t33)
    t4=time.time()
    # print(t4-t1,t2-t1,t3-t2,t4-t3)

    return centre_x_list,centre_y_list,radius_r_list,tv

def circle_filter(centre_x_list,centre_y_list,radius_r_list):
    centre_x_list=np.array(centre_x_list)
    centre_y_list=np.array(centre_y_list)
    radius_r_list=np.array(radius_r_list)
    n=len(centre_x_list)

    mask=np.ones((n,))

    mask[centre_x_list<500]=0
    mask[radius_r_list>2000]=0
    mask[radius_r_list<100]=0
    centre_x_list=centre_x_list[~(mask==0)]
    centre_y_list=centre_y_list[~(mask==0)]    
    radius_r_list=radius_r_list[~(mask==0)]    
    return centre_x_list,centre_y_list,radius_r_list

def cerate_map():
    return np.zeros((MAP_X_SIZE,MAP_Y_SIZE)).astype("uint8")

class trj:
    def __init__(self):
        self.map=np.zeros((MAP_X_SIZE,MAP_Y_SIZE,3)).astype("uint8")
        self.have_to_draw=0

    
    def add_car(self,x,y,theta):
        if self.have_to_draw==0:
            self.map=draw_arrowed(x,y,theta,self.map,color=(255,0,0))
            self.have_to_draw=10
        else:
            self.have_to_draw=self.have_to_draw-1

    def add_img(self,img):
        return cv2.add(self.map,img)


if __name__=="__main__":

    file_path = "/home/yuqiang/catkin_car/src/depth_car/src/syn_rosbag1/"
    map=cerate_map()
    car1=trj()
    with open(file_path + 'cb_pose.csv', 'r') as csvfile:
        robot_pose = list( csv.reader(csvfile, delimiter=',') )
        robot_pose = np.array(robot_pose).astype(float)
    for AA in range(1622):
        npDepth = np.load(file_path+"depth/"+str(AA)+".npy")
        cv2.imshow("Depth",npDepth)
        npColor = np.load(file_path+"color/"+str(AA)+".npy")
        npColor=cv2.cvtColor(npColor,cv2.COLOR_BGR2RGB)
        npColor=cv2.putText(npColor,"Frame= "+str(AA),(100,100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        cv2.imshow("color",npColor)

        t1=time.time()
        npPointX,npHeight,npDepth=depth_to_3DPoint(npDepth)

        t2=time.time()
        tm=get_tree_mask(npPointX,npHeight,npDepth)
        t3=time.time()
        # cv2.imshow("tree_mask",tm.astype("uint8")*255)
        # cv2.imshow("Depth_f",img_mask(npDepth,tm).astype("uint16"))

        z_depth, x_depth = depth_filter(npPointX,npHeight,npDepth,tm)
        t4=time.time()

        g = depth_to_topView(z_depth, x_depth)
        t5=time.time()

        centre_x_list,centre_y_list,radius_r_list, g2 = topView_to_circle(g)

        # centre_x_list,centre_y_list,radius_r_list = circle_filter(centre_x_list,centre_y_list,radius_r_list)

        print(centre_x_list,centre_y_list,radius_r_list)
        t6=time.time()
        
        map=into_world_map(map,z_depth, x_depth,robot_pose[AA,0]*1000,robot_pose[AA,1]*1000,robot_pose[AA,2]-1.570796)

        map=circle_to_world(map,centre_y_list,centre_x_list,radius_r_list,robot_pose[AA,0]*1000,robot_pose[AA,1]*1000,robot_pose[AA,2]-1.570796)
        t7=time.time()

        
        map_rgb=cv2.cvtColor(map*30, cv2.COLOR_GRAY2BGR)
        car1.add_car(robot_pose[AA,0]*1000,robot_pose[AA,1]*1000,robot_pose[AA,2])
        map_rgb=draw_arrowed(robot_pose[AA,0]*1000,robot_pose[AA,1]*1000,robot_pose[AA,2],map_rgb)
        map_rgb=car1.add_img(map_rgb)
        cv2.imwrite("map.png",map_rgb)
        map_rgb=cv2.resize(map_rgb, (1000,1000))
        cv2.imshow("map",map_rgb)
        # cv2.imshow("grid",g*255)
        cv2.imshow("grid2",g2*255)
        # cv2.imwrite("each/"+str(AA)+".png",g2*255)
        print(t7-t1,t2-t1,t3-t2,t4-t3,t5-t4,t6-t5,t7-t6)


        cv2.waitKey(1)