#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

count_vel = 0
count_seg = 0
count_odom = 0

def cb_vel(msg): global count_vel; count_vel += 1
def cb_seg(msg): global count_seg; count_seg += 1
def cb_odom(msg): global count_odom; count_odom += 1

rospy.init_node('test_count')
rospy.Subscriber('/velodyne_points', PointCloud2, cb_vel)
rospy.Subscriber('/segmented_cloud', PointCloud2, cb_seg)
rospy.Subscriber('/laser_odom_to_init', Odometry, cb_odom)
rospy.sleep(20)
print(f"velodyne_points: {count_vel}")
print(f"segmented_cloud: {count_seg}")
print(f"laser_odom_to_init: {count_odom}")
