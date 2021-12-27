'''math tool'''
import csv
import math
import numpy as np
from numpy import cos, sin, arctan2
from scipy.spatial import distance as dist
from datetime import datetime

'''plot tool'''
import matplotlib.pyplot as plt
from matplotlib import animation
world_bounds_x = [352880,352910]
world_bounds_y = [2767650,2767680]
# fig, ax = plt.subplots(figsize=(10,10),dpi=60)
# ax = plt.axes(fig,xlim=world_bounds_x, ylim=world_bounds_y)
fig, ax = plt.subplots(figsize=(10,10))
def plot_traj_for_demo(markers, markers_in_map, np_z_hat, np_z_true, bel_pose, cols, icp_flag, robot_utm_urs):
    global fig,ax

    # x_gps, y_gps, th_gps = gps_states
    # x_guess, y_guess, theta_guess = belief_states
    
    radius = 0.5
    
    # world_bounds_x = [352840,352870]
    # world_bounds_y = [2767700,2767730]
    # world_bounds_x = [352880,352910]
    # world_bounds_y = [2767650,2767680]
    # fig, ax = plt.subplots(figsize=(10,10),dpi=60)
    # ax = plt.axes(fig,xlim=world_bounds_x, ylim=world_bounds_y)
    ax.set_aspect('equal')

    '''plot landmarkers'''
    plt.scatter(markers_in_map[0], markers_in_map[1], marker='X',s=100, color='g', label='ref landmarks')
    if cols is not None:
        ax.scatter(markers_in_map[0][cols], markers_in_map[1][cols], marker='X',s=100, color='orange', label='matched landmarks')
    # plt.scatter(markers[0], markers[1], marker='X',s=100, color='r', label='obs landmarks')
    number_of_point=12
    piece_rad = np.pi/(number_of_point/2)
    
    for j in range( len(markers_in_map[0]) ):
        neg_bd = []
        for i in range(number_of_point):
            neg_bd.append((markers_in_map[0][j]+markers_in_map[2][j]*np.cos(piece_rad*i), markers_in_map[1][j]+markers_in_map[2][j]*np.sin(piece_rad*i)))
        neg_bd=np.asarray(neg_bd)
        ax.scatter(neg_bd[:,0], neg_bd[:,1], c='k', s=10)

    '''plot traj
    plt.scatter(bel_pose[0], bel_pose[1], color='b', label="GPS", s=10)'''

      
    '''plot now state'''
    ax.scatter(robot_utm_urs[0],robot_utm_urs[1], s=500, color='y', label='urs pose')
    ax.scatter(bel_pose[0], bel_pose[1], s=300, color='lightblue', label='Pose now')
    ax.plot( [ bel_pose[0],  bel_pose[0] - radius*sin(bel_pose[2]) ], 
               [ bel_pose[1], bel_pose[1] + radius*cos(bel_pose[2]) ], color='r' )
    
    # plt.plot( [x_guess[0][-1], x_guess[0][-1] + radius*cos(theta_guess[0][-1]) ], 
    #           [y_guess[0][-1], y_guess[0][-1] + radius*sin(theta_guess[0][-1]) ], color='k' )
    
    '''plot observation z'''
    
    
    if icp_flag == True: 
        # plt.text(352855, 2767703, 'icp matched points: '+str(cols),fontsize=14)
        ax.text(352895, 2767660, 'icp matched points: '+str(cols),fontsize=14)
        # plot_measured_landmarks(np_z_hat, np_z_true, bel_pose)
        bel_x, bel_y, bel_theta = bel_pose
        # updated_x, updated_y, updated_theta = updated_pose
        radius = 0.5

        # world_bounds = [-15,10]
        # fig, ax = plt.subplots(figsize=(10,10),dpi=120)
        # ax = plt.axes(xlim=world_bounds, ylim=world_bounds)
        # ax.set_aspect('equal')

        '''plot state
        plt.scatter(bel_x, bel_y, s=300, color='lightyellow', ec='k', label='z_hat pose')
        plt.plot( [bel_x, bel_x + radius*cos(bel_theta) ], 
                [bel_y, bel_y + radius*sin(bel_theta) ], color='k' )'''
        
        '''plot observation'''
        number_of_point=12
        piece_rad = np.pi/(number_of_point/2)
        print('np_z_hat number', np_z_hat.shape[1])
        print('np_z_hat: ', np_z_hat)
        for i in range( np_z_hat.shape[1] ):
            r_x = np_z_hat[0][i] * cos(np_z_hat[1][i])
            r_y = np_z_hat[0][i] * sin(np_z_hat[1][i])
            wr_x =  cos(bel_theta)*r_x - sin(bel_theta)*r_y
            # wr_x += bel_x
            # wr_x += bel_y
            wr_yy = bel_y+wr_x
            wr_y = sin(bel_theta)*r_x + cos(bel_theta)*r_y
            # wr_y += bel_y
            # wr_y -= bel_x
            wr_xx = bel_x-wr_y
            ax.plot([ bel_x,wr_xx ],
                    [ bel_y,wr_yy ], color='k', linestyle='--', label='predi_observing')

            r_x = np_z_true[0][i] * cos(np_z_true[1][i])
            r_y = np_z_true[0][i] * sin(np_z_true[1][i])
            wr_x =  cos(bel_theta)*r_x - sin(bel_theta)*r_y
            # wr_x += updated_x
            wr_yy = bel_y+wr_x
            wr_y = sin(bel_theta)*r_x + cos(bel_theta)*r_y
            wr_xx = bel_x-wr_y
            ax.plot([ bel_x, wr_xx ],
                    [ bel_y, wr_yy ], color='b', linestyle='-', label='updated-real_observing')
            print('np_z_true:', [ wr_xx, wr_yy ],
                    np_z_true[0][i], np_z_true[1][i], np_z_true[2][i])
            
            neg_bd = []
            ax.scatter(wr_xx, wr_yy, marker='X',s=100, color='b', label='obs landmarks')
            for j in range(number_of_point+1):
                neg_bd.append( (wr_xx+np_z_true[2][i]*np.cos(piece_rad*j), wr_yy+np_z_true[2][i]*np.sin(piece_rad*j)) )
            neg_bd=np.asarray(neg_bd)
            ax.plot(neg_bd[:,0], neg_bd[:,1], c='lightgray', marker='.')
            # plt.text(wr_xx, wr_yy, (round(np_z_true[0,i], 3), round(np_z_true[1,i], 3), round(np_z_true[2,i], 3)), fontsize=14)
            # plt.text(wr_xx, wr_yy-1, (round(np_z_hat[0,i], 3), round(np_z_hat[1,i], 3), round(np_z_hat[2,i], 3)), fontsize=14)
    else:
        ax.text(352895, 2767660, 'maximum matched point',fontsize=14)
        # plt.text(352855, 2767703, 'maximum matched point',fontsize=14)

    '''plot obs lm
    for i in range( len(markers[0]) ):
        plt.plot([ markers[0][i],x_tr[0][index] ],
                [ markers[1][i],y_tr[0][index] ], color='k')'''
    # plt.scatter(x_guess[0][:index+1], y_guess[0][:index+1], color='r', label="Predicted", s=10)
    # plt.title('update times: '+str(index)+'/200', fontsize=25)
    # ax.set_xticklabels(fontsize=20)
    # ax.set_yticklabels(fontsize=20)
    # plt.legend(fontsize=15)
    plt.draw()
    # fig.savefig('/home/yuqiang/211125/' + datetime.now().strftime("%d-%m-%Y %H:%M:%S")+'.png', )
    plt.pause(0.5)
    # plt.close(fig)
    

def plot_measured_landmarks(np_z_hat, np_z_true, bel_pose):#, updated_pose
    bel_x, bel_y, bel_theta = bel_pose
    # updated_x, updated_y, updated_theta = updated_pose
    radius = 0.5

    # world_bounds = [-15,10]
    # fig, ax = plt.subplots(figsize=(10,10),dpi=120)
    # ax = plt.axes(xlim=world_bounds, ylim=world_bounds)
    # ax.set_aspect('equal')

    '''plot state
    plt.scatter(bel_x, bel_y, s=300, color='lightyellow', ec='k', label='z_hat pose')
    plt.plot( [bel_x, bel_x + radius*cos(bel_theta) ], 
            [bel_y, bel_y + radius*sin(bel_theta) ], color='k' )'''
    
    '''plot observation'''
    number_of_point=12
    piece_rad = np.pi/(number_of_point/2)
    for i in range( np_z_hat.shape[1] ):
        r_x = np_z_hat[0][i] * cos(np_z_hat[1][i])
        r_y = np_z_hat[0][i] * sin(np_z_hat[1][i])
        wr_x =  cos(bel_theta)*r_x - sin(bel_theta)*r_y
        # wr_x += bel_x
        # wr_x += bel_y
        wr_yy = bel_y+wr_x
        wr_y = sin(bel_theta)*r_x + cos(bel_theta)*r_y
        # wr_y += bel_y
        # wr_y -= bel_x
        wr_xx = bel_x-wr_y
        plt.plot([ bel_x,wr_xx ],
                [ bel_y,wr_yy ], color='k', linestyle='--', label='predi_observing')

        r_x = np_z_true[0][i] * cos(np_z_true[1][i])
        r_y = np_z_true[0][i] * sin(np_z_true[1][i])
        wr_x =  cos(bel_theta)*r_x - sin(bel_theta)*r_y
        # wr_x += updated_x
        wr_yy = bel_y+wr_x
        wr_y = sin(bel_theta)*r_x + cos(bel_theta)*r_y
        wr_xx = bel_x-wr_y
        plt.plot([ bel_x, wr_xx ],
                [ bel_y, wr_yy ], color='b', linestyle='-', label='updated-real_observing')
        print('here:',np_z_hat.shape[1], [ bel_x, wr_xx ],
                [ bel_y, wr_yy ], np_z_true[0][i], np_z_true[1][i], np_z_true[2][i])
        
        neg_bd = []
        plt.scatter(wr_xx, wr_yy, marker='X',s=100, color='b', label='obs landmarks')
        for j in range(number_of_point+1):
            neg_bd.append( (wr_xx+np_z_true[2][i]*np.cos(piece_rad*j), wr_yy+np_z_true[2][i]*np.sin(piece_rad*j)) )
        neg_bd=np.asarray(neg_bd)
        plt.plot(neg_bd[:,0], neg_bd[:,1], c='lightgray', marker='.')
        # plt.text(wr_xx, wr_yy, (round(np_z_true[0,i], 3), round(np_z_true[1,i], 3), round(np_z_true[2,i], 3)), fontsize=14)
        # plt.text(wr_xx, wr_yy-1, (round(np_z_hat[0,i], 3), round(np_z_hat[1,i], 3), round(np_z_hat[2,i], 3)), fontsize=14)
    
        

def plot_measured_landmarks_for_sim(np_z_hat, np_z_true, bel_pose, real_pose):
    bel_x, bel_y, bel_theta = bel_pose
    real_x, real_y, real_theta = real_pose
    radius = 0.5

    # world_bounds = [-15,10]
    # fig, ax = plt.subplots(figsize=(10,10),dpi=120)
    # ax = plt.axes(xlim=world_bounds, ylim=world_bounds)
    # ax.set_aspect('equal')

    '''plot state
    plt.scatter(bel_x, bel_y, s=300, color='lightyellow', ec='k', label='z_hat pose')
    plt.plot( [bel_x, bel_x + radius*cos(bel_theta) ], 
               [bel_y, bel_y + radius*sin(bel_theta) ], color='k' )'''
    
    '''plot observation'''
    
    for i in range( np_z_hat.shape[1] ):
        r_x = np_z_hat[0][i] * cos(np_z_hat[1][i])
        r_y = np_z_hat[0][i] * sin(np_z_hat[1][i])
        wr_x =  cos(bel_theta)*r_x - sin(bel_theta)*r_y
        wr_x += bel_x
        wr_y = sin(bel_theta)*r_x + cos(bel_theta)*r_y
        wr_y += bel_y
        plt.plot([ bel_x,wr_x ],
                 [ bel_y,wr_y ], color='k', linestyle='--', label='predi_observing')

        r_x = np_z_true[0][i] * cos(np_z_true[1][i])
        r_y = np_z_true[0][i] * sin(np_z_true[1][i])
        wr_x =  cos(real_theta)*r_x - sin(real_theta)*r_y
        wr_x += real_x
        wr_y = sin(real_theta)*r_x + cos(real_theta)*r_y
        wr_y += real_y
        plt.plot([ real_x,wr_x ],
                 [ real_y,wr_y ], color='b', linestyle='-', label='real_observing')

def plot_transformed(P, U, robot_pose, theta, count):
    # robot_x, robot_y, robot_theta = robot_pose
    radius = 0.5

    world_bounds = [-15,10]
    fig, ax = plt.subplots(figsize=(10,10),dpi=120)
    ax = plt.axes(xlim=world_bounds, ylim=world_bounds)
    ax.set_aspect('equal')

    '''plot state'''
    plt.scatter(robot_pose[0,0], robot_pose[1,0], s=300, color='lightblue', ec='k', label='robot_true pose')
    plt.plot( [robot_pose[0,0], robot_pose[0,0] + radius*cos(theta) ], 
              [robot_pose[1,0], robot_pose[1,0] + radius*sin(theta) ], color='k' )
    
    '''plot transformed observation'''
    plt.scatter(P[0,:], P[1,:], c='g',s=300, marker="X",label='ref lms')
    plt.scatter(U[0,:], U[1,:], c='r',s=300, label='transformed lms')

    plt.title('process of transforming, iteration time: '+str(count), fontsize=25)
    plt.yticks(fontsize=20)
    plt.xticks(fontsize=20)
    plt.legend(fontsize=15)
    plt.show()
    
def plot_respect_to_time_mu( mu_hat, mu_hat_lm, mu, obs_lm_number):
    (mu_hat_x, mu_hat_y, mu_hat_theta), (mu_hat_lm_x, mu_hat_lm_y, mu_hat_lm_theta), (mu_x, mu_y, mu_theta) \
        =  mu_hat, mu_hat_lm, mu
    # timestamp = np.array(range(mu_hat_x.shape[1]-1)) + 1
    timestamp = np.array(range(3625)) + 1
    timestamp1 = np.array(range(100-1,1627-1)) + 1

    fig, ax = plt.subplots(figsize=(25,8),dpi=160)
        
    # plt.scatter(timestamp, mu_x[0][99:1627], color='b', s=64, label='mu')
    # plt.scatter(timestamp1, mu_hat_lm_x[0][100:1627], color='g', s=32, label='mu_hat_lm')
    # plt.scatter(timestamp1, mu_hat_x[0][100:1627], color='r', s=8, label='mu_hat')
    plt.scatter(timestamp, obs_lm_number[0][:3625], color='y', s=13, label='obs_lm_number')

    plt.title('x vs. time', fontsize=25)
    plt.yticks(fontsize=20)
    plt.xticks(fontsize=20)
    plt.legend(fontsize=15)
    plt.show()

def plot_respect_to_time(data):
    
    # timestamp = np.array(range(mu_hat_x.shape[1]-1)) + 1
    timestamp = np.array(range( len(data) )) + 0
    timestamp1 = np.array(range(100-1,1627-1)) + 1

    fig, ax = plt.subplots(figsize=(25,8),dpi=160)
        
    # plt.scatter(timestamp, mu_x[0][99:1627], color='b', s=64, label='mu')
    # plt.scatter(timestamp1, mu_hat_lm_x[0][100:1627], color='g', s=32, label='mu_hat_lm')
    # plt.scatter(timestamp1, mu_hat_x[0][100:1627], color='r', s=8, label='mu_hat')
    plt.scatter(timestamp, data, color='y', s=13, label='angle diff')

    plt.title('angle diff vs. time', fontsize=25)
    plt.yticks(fontsize=20)
    plt.xticks(fontsize=20)
    plt.legend(fontsize=15)
    plt.show()