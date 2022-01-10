import os
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
import numpy as np

file_name="/home/yuqiang/20220110_17-32-20/shapefiles/waypoint/waypoint.txt"
file_new_name="/home/yuqiang/20220110_17-32-20/shapefiles/waypoint/waypoint_new.txt"

if os.path.isfile(file_name) is True:
    print('waypoint_utm exist!')
    utm_x_waypoints_list = []
    utm_y_waypoints_list = []
    with open(file_name) as f:
        for line in f.readlines():
            s = line.split(' ')
            utm_x_waypoints_list.append(float(s[0]))
            utm_y_waypoints_list.append(float(s[1]))
    utm_x_waypoints = np.asarray(utm_x_waypoints_list)
    utm_y_waypoints = np.asarray(utm_y_waypoints_list)
fig, ax = plt.subplots(figsize=(10,10))

wp_x=utm_x_waypoints_list
wp_y=utm_y_waypoints_list
point_list=[ax.plot(wp_x[i],wp_y[i],"*",c="red",ms=10)[0] for i in range(len(wp_x))]
sel=[1 for i in range(len(wp_x))]
plt.plot(wp_x,wp_y)
plt.title("Press the right button to delete, press the left button to restore\n Red: exists, Black: does not exist\n Press x to save and exit")

def add_or_remove_point2(event):
    if event.xdata is None or event.ydata is None:
        return
    xdata_click = event.xdata
    ydata_click = event.ydata
    i=(np.sqrt((wp_x-xdata_click)**2+(wp_y-ydata_click)**2)).argmin()
    print(xdata_click,ydata_click,i)
    if event.button == 1:
        sel[i]=1
        point_list[i].set_color("red")
    elif event.button == 3:
        sel[i]=0
        point_list[i].set_color("black")
    
    plt.draw()

def save_wp(event):
    if not event.key == 'x':
        return 
    print("save_wp")
    for i in range(len(wp_x)):
        if sel[i]==1:
            with open(file_new_name,"a") as f:
                f.write(str(wp_x[i])+" "+str(wp_y[i])+"\n")
    plt.close('all')
    

fig.canvas.mpl_connect('button_press_event',add_or_remove_point2)
fig.canvas.mpl_connect('key_press_event',save_wp)
plt.show()