import rosbag
import re
path='/media/yuqiang/2CFC30D1FC309754/BAG/220127_DaHu/bag'
filename='2022-01-27-14-24-06'
bag = rosbag.Bag(path+"/"+filename+".bag", 'r')
print(bag)

topic_list=list(bag.get_type_and_topic_info().topics.keys())
print(topic_list)
topic_list2=[]
for i in topic_list:
    if re.match("/lm.*",i) or re.match("/ctrl.*",i) or re.match("/tree.*",i):
            pass
    else:
        topic_list2.append(i)        
print(topic_list2)


bag_out = rosbag.Bag(path+"/"+filename+"built in.bag", 'w')

for topic, msg, t in bag.read_messages():
    if re.match("/lm.*",topic) or re.match("/ctrl.*",topic) or re.match("/tree.*",topic):
        pass
    else:
        bag_out.write(topic, msg, t)
    print(t)
bag.close()
bag_out.close()


'''
rosbag filter 2022-01-27-14-24-06.bag 2022-01-27-14-24-06_in.bag "topic in ['/cmd_loop', '/tf', '/gps/vel', '/outdoor_waypoint_nav/odometry/gps', '/outdoor_waypoint_nav/move_basic/plan', '/imu_filter/rpy/raw', '/tf_static', '/move_base_simple/goal', '/gps/qual', '/imu_filter/parameter_updates', '/husky_velocity_controller/cmd_vel', '/imu_filter/rpy/filtered', '/imu_filter/parameter_descriptions', '/cmd_vel', '/imu/data', '/imu_filter/data', '/imu/mag', '/scan_filtered', '/outdoor_waypoint_nav/gps/filtered', '/husky_velocity_controller/odom', '/gps/time_reference', '/outdoor_waypoint_nav/move_basic/obstacle_viz', '/outdoor_waypoint_nav/odometry/filtered_map', '/navsat/fix', '/outdoor_waypoint_nav/odometry/filtered', '/outdoor_waypoint_nav/move_basic/obstacle_distance', '/scan']"
'''
